/*
 * Copyright (C) 2007 - 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 */

#include <linux/kernel.h>
#include <linux/proc_fs.h>
//#include <linux/smp_lock.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include "cpcap_audio_driver.h"
#include <linux/spi/cpcap.h>
#include <linux/resource.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>

#define  CPCAP_AUDIO_DEBUG 1

#define SLEEP_ACTIVATE_POWER 2

#define CLOCK_TREE_RESET_TIME 1

#define SLEEP_OUTPUT_AMPS_ENABLE 1

#ifdef CPCAP_AUDIO_DEBUG
#define CPCAP_AUDIO_DEBUG_LOG(args...)  \
				printk(KERN_INFO "CPCAP_AUDIO_DRIVER:" args)
#else
#define CPCAP_AUDIO_DEBUG_LOG(args...)
#endif

#define CPCAP_AUDIO_ERROR_LOG(args...)  \
			printk(KERN_ERR "CPCAP_AUDIO_DRIVER: Error " args)

#define ERROR_EXIT _err
#define TRY(a)  if (unlikely(a)) goto ERROR_EXIT;

static struct cpcap_audio_state previous_state_struct = {
	NULL,
	CPCAP_AUDIO_MODE_NORMAL,
	CPCAP_AUDIO_CODEC_OFF,
	CPCAP_AUDIO_CODEC_RATE_8000_HZ,
	CPCAP_AUDIO_CODEC_MUTE,
	CPCAP_AUDIO_STDAC_OFF,
	CPCAP_AUDIO_STDAC_RATE_8000_HZ,
	CPCAP_AUDIO_STDAC_MUTE,
	CPCAP_AUDIO_ANALOG_SOURCE_OFF,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_OUT_NONE,
	CPCAP_AUDIO_BALANCE_NEUTRAL,
	CPCAP_AUDIO_BALANCE_NEUTRAL,
	CPCAP_AUDIO_BALANCE_NEUTRAL,
	0,			/* output gain */
	CPCAP_AUDIO_IN_NONE,
	0,			/* input_gain */
	CPCAP_AUDIO_RAT_NONE
};

/* Define regulator to turn on the audio portion of cpcap */
struct regulator *audio_regu;

/*is headset mic driver present on GPIO4?*/
static inline int cpcap_audio_hw_has_hs_driver(void)
{
        if (machine_is_etna())
	        return 0;
	if (cpcap_audio_is_cdma_shadow())
		return 0;
	return 1;
}

static inline int cpcap_audio_hw_has_mic3_switch(void)
{
        if (machine_is_etna())
	        return 0;
	if (cpcap_audio_is_cdma_shadow())
		return 1;
	return 0;
}


static inline int needs_cpcap_mic1(int microphone)
{
	if ((microphone & CPCAP_AUDIO_IN_HANDSET) != 0 ||
		(microphone	& CPCAP_AUDIO_IN_HEADSET) != 0 ||
		(microphone	& CPCAP_AUDIO_IN_DUAL_EXTERNAL) != 0)
		return 1;
	return 0;
}

static inline int needs_cpcap_mic2(int microphone)
{
	/*CPCAP MIC2 is secondary *or* tertiary internal if we can get
	 *away with not turning the ADC on*/
	if ((microphone & CPCAP_AUDIO_IN_DUAL_EXTERNAL) != 0 ||
		(microphone & CPCAP_AUDIO_IN_SECONDARY_INTERNAL) != 0 ||
		((microphone & CPCAP_AUDIO_IN_TERTIARY_INTERNAL) != 0 &&
		 (microphone & CPCAP_AUDIO_IN_QUATERNARY_INTERNAL) == 0))
		return 1;
	return 0;
}

static inline int needs_outboard_adc(int microphone)
{
	/*need ADC if mic4 and/or both mic2 and mic3 are turned on*/
	if (((microphone & CPCAP_AUDIO_IN_QUATERNARY_INTERNAL) != 0) ||
		((microphone & CPCAP_AUDIO_IN_SECONDARY_INTERNAL) != 0 &&
		 (microphone & CPCAP_AUDIO_IN_TERTIARY_INTERNAL) != 0))
		return 1;
	return 0;
}

static inline int needs_cpcap_mic3_switch(int microphone)
{
	if ((microphone & CPCAP_AUDIO_IN_TERTIARY_INTERNAL) != 0 &&
		(microphone & CPCAP_AUDIO_IN_SECONDARY_INTERNAL) == 0 &&
		(microphone	& CPCAP_AUDIO_IN_QUATERNARY_INTERNAL) == 0)

		return 1;
	return 0;
}

static inline int is_codec_changed(struct cpcap_audio_state *state,
					struct cpcap_audio_state *prev_state)
{
	/*TODO: microphone check is overzealous. optimize.*/
	/*TODO: need to also check for speaker turning on/off */
	if (state->codec_mode != prev_state->codec_mode ||
		state->codec_rate != prev_state->codec_rate ||
		state->rat_type != prev_state->rat_type ||
		state->microphone != prev_state->microphone ||
		state->dai_config != prev_state->dai_config)
		return 1;


	return 0;
}

static inline int is_stdac_changed(struct cpcap_audio_state *state,
					struct cpcap_audio_state *prev_state)
{
	if (state->stdac_mode != prev_state->stdac_mode ||
		state->rat_type != prev_state->rat_type ||
		state->stdac_rate != prev_state->stdac_rate ||
		state->dai_config != prev_state->dai_config)

		return 1;
	return 0;
}

static inline int is_output_bt_only(struct cpcap_audio_state *state)
{
	if (state->codec_primary_speaker == CPCAP_AUDIO_OUT_BT_MONO
		&& state->codec_secondary_speaker == CPCAP_AUDIO_OUT_NONE)
		return 1;

	if (state->stdac_primary_speaker == CPCAP_AUDIO_OUT_BT_MONO
	      && state->stdac_secondary_speaker == CPCAP_AUDIO_OUT_NONE)
		return 1;

	if (state->ext_primary_speaker == CPCAP_AUDIO_OUT_BT_MONO
		&& state->ext_secondary_speaker == CPCAP_AUDIO_OUT_NONE)
		return 1;

	return 0;
}

static inline int is_output_headset(struct cpcap_audio_state *state)
{
	if (state->codec_primary_speaker == CPCAP_AUDIO_OUT_STEREO_HEADSET
	|| state->codec_primary_speaker == CPCAP_AUDIO_OUT_MONO_HEADSET
	|| state->codec_secondary_speaker == CPCAP_AUDIO_OUT_STEREO_HEADSET
	|| state->codec_secondary_speaker == CPCAP_AUDIO_OUT_MONO_HEADSET)
		return 1;

	if (state->stdac_primary_speaker == CPCAP_AUDIO_OUT_STEREO_HEADSET
	|| state->stdac_primary_speaker == CPCAP_AUDIO_OUT_MONO_HEADSET
	|| state->stdac_secondary_speaker == CPCAP_AUDIO_OUT_STEREO_HEADSET
	|| state->stdac_secondary_speaker == CPCAP_AUDIO_OUT_MONO_HEADSET)
		return 1;

	if (state->ext_primary_speaker == CPCAP_AUDIO_OUT_STEREO_HEADSET
	|| state->ext_primary_speaker == CPCAP_AUDIO_OUT_MONO_HEADSET
	|| state->ext_secondary_speaker == CPCAP_AUDIO_OUT_STEREO_HEADSET
	|| state->ext_secondary_speaker == CPCAP_AUDIO_OUT_MONO_HEADSET)
		return 1;

	return 0;
}

static inline int is_speaker_turning_off(struct cpcap_audio_state *state,
					struct cpcap_audio_state *prev_state)
{
	if ((prev_state->codec_primary_speaker != CPCAP_AUDIO_OUT_NONE &&
		state->codec_primary_speaker == CPCAP_AUDIO_OUT_NONE) ||
		(prev_state->codec_secondary_speaker != CPCAP_AUDIO_OUT_NONE &&
		state->codec_secondary_speaker == CPCAP_AUDIO_OUT_NONE) ||
		(prev_state->stdac_primary_speaker != CPCAP_AUDIO_OUT_NONE &&
		state->stdac_primary_speaker == CPCAP_AUDIO_OUT_NONE) ||
		(prev_state->stdac_secondary_speaker != CPCAP_AUDIO_OUT_NONE &&
		state->stdac_secondary_speaker == CPCAP_AUDIO_OUT_NONE) ||
		(prev_state->ext_primary_speaker != CPCAP_AUDIO_OUT_NONE &&
		state->ext_primary_speaker == CPCAP_AUDIO_OUT_NONE) ||
		(prev_state->ext_secondary_speaker != CPCAP_AUDIO_OUT_NONE &&
		state->ext_secondary_speaker == CPCAP_AUDIO_OUT_NONE))
		return 1;

	return 0;
}

static inline int is_output_changed(struct cpcap_audio_state *state,
					struct cpcap_audio_state *prev_state)
{
	if (state->codec_primary_speaker != prev_state->codec_primary_speaker
	|| state->codec_primary_balance != prev_state->codec_primary_balance
		|| state->codec_secondary_speaker !=
					prev_state->codec_secondary_speaker)
		return 1;

	if (state->stdac_primary_speaker != prev_state->stdac_primary_speaker
	|| state->stdac_primary_balance != prev_state->stdac_primary_balance
	|| state->stdac_secondary_speaker !=
		prev_state->stdac_secondary_speaker)
		return 1;

	if (state->ext_primary_speaker != prev_state->ext_primary_speaker
	|| state->ext_primary_balance != prev_state->ext_primary_balance
	|| state->ext_secondary_speaker != prev_state->ext_secondary_speaker)
		return 1;

	return 0;
}

static void
audioic_state_dump
(
	struct cpcap_audio_state *state
)
{
	CPCAP_AUDIO_DEBUG_LOG("***************************************\n");
	CPCAP_AUDIO_DEBUG_LOG("state->mode = %d\n",  state->mode);
	CPCAP_AUDIO_DEBUG_LOG("state->codec_mode = %d\n", state->codec_mode);
	CPCAP_AUDIO_DEBUG_LOG("state->codec_rate = %d\n", state->codec_rate);
	CPCAP_AUDIO_DEBUG_LOG("state->codec_mute = %d\n", state->codec_mute);
	CPCAP_AUDIO_DEBUG_LOG("state->stdac_mode = %d\n", state->stdac_mode);
	CPCAP_AUDIO_DEBUG_LOG("state->stdac_rate = %d\n", state->stdac_rate);
	CPCAP_AUDIO_DEBUG_LOG("state->stdac_mute = %d\n", state->stdac_mute);
	CPCAP_AUDIO_DEBUG_LOG("state->analog_source = %d\n",
						state->analog_source);
	CPCAP_AUDIO_DEBUG_LOG("state->codec_primary_speaker = %d\n",
						state->codec_primary_speaker);
	CPCAP_AUDIO_DEBUG_LOG("state->stdac_primary_speaker = %d\n",
						state->stdac_primary_speaker);
	CPCAP_AUDIO_DEBUG_LOG("state->ext_primary_speaker = %d\n",
						state->ext_primary_speaker);
	CPCAP_AUDIO_DEBUG_LOG("state->codec_secondary_speaker = %d\n",
						state->codec_secondary_speaker);
	CPCAP_AUDIO_DEBUG_LOG("state->stdac_secondary_speaker = %d\n",
						state->stdac_secondary_speaker);
	CPCAP_AUDIO_DEBUG_LOG("state->ext_secondary_speaker   = %d\n",
						state->ext_secondary_speaker);
	CPCAP_AUDIO_DEBUG_LOG("state->stdac_primary_balance   = %d\n",
						state->stdac_primary_balance);
	CPCAP_AUDIO_DEBUG_LOG("state->ext_primary_balance     = %d\n",
						state->ext_primary_balance);
	CPCAP_AUDIO_DEBUG_LOG("state->output_gain             = %d\n",
						state->output_gain);
	CPCAP_AUDIO_DEBUG_LOG("state->microphone              = %d\n",
						state->microphone);
	CPCAP_AUDIO_DEBUG_LOG("state->input_gain              = %d\n",
						state->input_gain);
	CPCAP_AUDIO_DEBUG_LOG("state->rat_type                = %d\n",
						state->rat_type);
	CPCAP_AUDIO_DEBUG_LOG("***************************************\n");
}

/* this is only true for audio registers, but those are the only ones we use */
#define CPCAP_REG_FOR_POWERIC_REG(a) ((a) + (0x200 - CPCAP_REG_VAUDIOC))

static void logged_cpcap_write(struct cpcap_device *cpcap, unsigned int reg,
			unsigned short int value, unsigned short int mask)
{

    int ret_val = 0;
	if (mask == 0)
		return;

#ifdef CPCAP_AUDIO_SPI_LOG
	printk(KERN_INFO
		"CPCAP_AUDIO_SPI_WRITE: reg %u, value 0x%x, mask 0x%x\n",
	       CPCAP_REG_FOR_POWERIC_REG(reg), value, mask);
#endif
	ret_val = cpcap_regacc_write(cpcap, reg, value, mask);
	if (ret_val != 0)
		CPCAP_AUDIO_ERROR_LOG(
			"Write to register %u failed: error %d \n",
			reg, ret_val);

#ifdef CPCAP_AUDIO_SPI_READBACK
	ret_val = cpcap_regacc_read(cpcap, reg, &value);
	if (ret_val == 0)
		printk(KERN_INFO
			"CPCAP_AUDIO_SPI_VERIFY reg %u: value 0x%x \n",
			CPCAP_REG_FOR_POWERIC_REG(reg), value);
	else
		printk(KERN_ERR
			"CPCAP_AUDIO_SPI_VERIFY reg %u FAILED\n",
			CPCAP_REG_FOR_POWERIC_REG(reg));
#endif
}



static unsigned short int cpcap_audio_get_codec_output_amp_switches(
						int speaker, int balance)
{
	unsigned short int value = CPCAP_BIT_PGA_CDC_EN;

	CPCAP_AUDIO_DEBUG_LOG("%s() called with speaker = %d\n", __func__,
			  speaker);

	switch (speaker) {
	case CPCAP_AUDIO_OUT_HANDSET:
		value |= CPCAP_BIT_A1_EAR_CDC_SW;
		break;

	case CPCAP_AUDIO_OUT_LOUDSPEAKER:
		value |= CPCAP_BIT_A2_LDSP_R_CDC_SW;

		break;

	case CPCAP_AUDIO_OUT_MONO_HEADSET:
	case CPCAP_AUDIO_OUT_STEREO_HEADSET:
		if (balance != CPCAP_AUDIO_BALANCE_L_ONLY)
			value |= CPCAP_BIT_ARIGHT_HS_CDC_SW;
		if (balance != CPCAP_AUDIO_BALANCE_R_ONLY)
			value |= CPCAP_BIT_ALEFT_HS_CDC_SW;
		break;

	case CPCAP_AUDIO_OUT_LINEOUT:
		value |= CPCAP_BIT_A4_LINEOUT_R_CDC_SW |
			CPCAP_BIT_A4_LINEOUT_L_CDC_SW;
		break;

	case CPCAP_AUDIO_OUT_BT_MONO:
	default:
		value = 0;
		break;
	}

	CPCAP_AUDIO_DEBUG_LOG("Exiting %s() with return value = %d\n", __func__,
			  value);
	return value;
}

static unsigned short int cpcap_audio_get_stdac_output_amp_switches(
						int speaker, int balance)
{
	unsigned short int value = CPCAP_BIT_PGA_DAC_EN;

	CPCAP_AUDIO_DEBUG_LOG("%s() called with speaker = %d\n", __func__,
			  speaker);

	switch (speaker) {
	case CPCAP_AUDIO_OUT_HANDSET:
		value |= CPCAP_BIT_A1_EAR_DAC_SW | CPCAP_BIT_MONO_DAC0 |
			CPCAP_BIT_MONO_DAC1;
		break;

	case CPCAP_AUDIO_OUT_MONO_HEADSET:
	case CPCAP_AUDIO_OUT_STEREO_HEADSET:
		if (balance != CPCAP_AUDIO_BALANCE_R_ONLY)
			value |= CPCAP_BIT_ALEFT_HS_DAC_SW;
		if (balance != CPCAP_AUDIO_BALANCE_L_ONLY)
			value |= CPCAP_BIT_ARIGHT_HS_DAC_SW;
		break;

	case CPCAP_AUDIO_OUT_LOUDSPEAKER:
		value |= CPCAP_BIT_A2_LDSP_R_DAC_SW | CPCAP_BIT_MONO_DAC0 |

			CPCAP_BIT_MONO_DAC1;
		break;

	case CPCAP_AUDIO_OUT_LINEOUT:
		value |= CPCAP_BIT_A4_LINEOUT_R_DAC_SW |
			CPCAP_BIT_A4_LINEOUT_L_DAC_SW;
		break;

	case CPCAP_AUDIO_OUT_BT_MONO:
	default:
		value = 0;
		break;
	}

	CPCAP_AUDIO_DEBUG_LOG("Exiting %s() with return value = %d\n", __func__,
			  value);
	return value;
}

static unsigned short int cpcap_audio_get_ext_output_amp_switches(
						int speaker,
						int balance)
{
	unsigned short int value = 0;
	CPCAP_AUDIO_DEBUG_LOG("%s() called with speaker %d\n", __func__,
								speaker);

	switch (speaker) {
	case CPCAP_AUDIO_OUT_HANDSET:
		value = CPCAP_BIT_A1_EAR_EXT_SW | CPCAP_BIT_PGA_EXT_R_EN;
		break;

	case CPCAP_AUDIO_OUT_MONO_HEADSET:
	case CPCAP_AUDIO_OUT_STEREO_HEADSET:
		if (balance != CPCAP_AUDIO_BALANCE_L_ONLY)
			value = CPCAP_BIT_ARIGHT_HS_EXT_SW |
				CPCAP_BIT_PGA_EXT_R_EN;
		if (balance != CPCAP_AUDIO_BALANCE_R_ONLY)
			value |= CPCAP_BIT_ALEFT_HS_EXT_SW |
				CPCAP_BIT_PGA_EXT_L_EN;
		break;

	case CPCAP_AUDIO_OUT_LOUDSPEAKER:
		value = CPCAP_BIT_A2_LDSP_R_EXT_SW | CPCAP_BIT_PGA_EXT_R_EN;
		break;

	case CPCAP_AUDIO_OUT_LINEOUT:
		value = CPCAP_BIT_A4_LINEOUT_R_EXT_SW |
			CPCAP_BIT_A4_LINEOUT_L_EXT_SW |
			CPCAP_BIT_PGA_EXT_L_EN | CPCAP_BIT_PGA_EXT_R_EN;
		break;

	case CPCAP_AUDIO_OUT_BT_MONO:
	default:
		value = 0;
		break;
	}

	CPCAP_AUDIO_DEBUG_LOG("Exiting %s() with return value = %d\n", __func__,
			  value);
	return value;
}

static void cpcap_audio_set_output_amp_switches(struct cpcap_audio_state *state)
{
	static unsigned int 	codec_prev_settings = 0,
				stdac_prev_settings = 0,
				ext_prev_settings = 0;

	struct cpcap_regacc reg_changes;
	unsigned short int value1 = 0, value2 = 0;

	/* First set codec output amp switches */
	value1 =
	    cpcap_audio_get_codec_output_amp_switches(state->
				codec_primary_speaker,
				state->codec_primary_balance);
	value2 =
	    cpcap_audio_get_codec_output_amp_switches(state->
				codec_secondary_speaker,
				state->codec_primary_balance);

	reg_changes.mask = value1 | value2 | codec_prev_settings;
	reg_changes.value = value1 | value2;
	codec_prev_settings = reg_changes.value;

	logged_cpcap_write(state->cpcap, CPCAP_REG_RXCOA,
		reg_changes.value, reg_changes.mask);

	/* Second Stdac switches */
	value1 =
	    cpcap_audio_get_stdac_output_amp_switches(state->
				stdac_primary_speaker,
				state->stdac_primary_balance);
	value2 =
	    cpcap_audio_get_stdac_output_amp_switches(state->
				stdac_secondary_speaker,
				state->stdac_primary_balance);

	reg_changes.mask = value1 | value2 | stdac_prev_settings;
	reg_changes.value = value1 | value2;

	if ((state->stdac_primary_speaker == CPCAP_AUDIO_OUT_STEREO_HEADSET &&
		state->stdac_secondary_speaker == CPCAP_AUDIO_OUT_LOUDSPEAKER)
		|| (state->stdac_primary_speaker == CPCAP_AUDIO_OUT_LOUDSPEAKER
		&& state->stdac_secondary_speaker ==
						CPCAP_AUDIO_OUT_STEREO_HEADSET))
		reg_changes.value &= ~(CPCAP_BIT_MONO_DAC0 |
					CPCAP_BIT_MONO_DAC1);

	stdac_prev_settings = reg_changes.value;

	logged_cpcap_write(state->cpcap, CPCAP_REG_RXSDOA,
			reg_changes.value, reg_changes.mask);

	/* Last External source switches */
	value1 =
	    cpcap_audio_get_ext_output_amp_switches(state->
				ext_primary_speaker,
				state->ext_primary_balance);
	value2 =
	    cpcap_audio_get_ext_output_amp_switches(state->
				ext_secondary_speaker,
				state->ext_primary_balance);

	reg_changes.mask = value1 | value2 | ext_prev_settings;
	reg_changes.value = value1 | value2;
	ext_prev_settings = reg_changes.value;

	logged_cpcap_write(state->cpcap, CPCAP_REG_RXEPOA,
			reg_changes.value, reg_changes.mask);
}

static bool cpcap_audio_set_bits_for_speaker(int speaker, int balance,
						unsigned short int *message)
{
	CPCAP_AUDIO_DEBUG_LOG("%s() called with speaker = %d\n", __func__,
			  speaker);

	/* Get the data required to enable each possible path */
	switch (speaker) {
	case CPCAP_AUDIO_OUT_HANDSET:
		(*message) |= CPCAP_BIT_A1_EAR_EN;
		break;

	case CPCAP_AUDIO_OUT_MONO_HEADSET:
	case CPCAP_AUDIO_OUT_STEREO_HEADSET:
		if (balance != CPCAP_AUDIO_BALANCE_R_ONLY)
			(*message) |= CPCAP_BIT_HS_L_EN;
		if (balance != CPCAP_AUDIO_BALANCE_L_ONLY)
			(*message) |= CPCAP_BIT_HS_R_EN;
		break;

	case CPCAP_AUDIO_OUT_LOUDSPEAKER:
		(*message) |= CPCAP_BIT_A2_LDSP_R_EN;
		break;

	case CPCAP_AUDIO_OUT_LINEOUT:
		(*message) |= CPCAP_BIT_A4_LINEOUT_R_EN |
				CPCAP_BIT_A4_LINEOUT_L_EN;
		break;

	case CPCAP_AUDIO_OUT_BT_MONO:
	default:
		(*message) |= 0;
		break;
	}

	return false; /* There is no external loudspeaker on this product */
}

static void cpcap_audio_configure_aud_mute(struct cpcap_audio_state *state,
				struct cpcap_audio_state *prev_state)
{
	struct cpcap_regacc reg_changes = { 0 };
	unsigned short int value1 = 0, value2 = 0;

	if (state->codec_mute != prev_state->codec_mute) {
		value1 = cpcap_audio_get_codec_output_amp_switches(
				prev_state->codec_primary_speaker,
				prev_state->codec_primary_balance);

		value2 = cpcap_audio_get_codec_output_amp_switches(
				prev_state->codec_secondary_speaker,
				prev_state->codec_primary_balance);

		reg_changes.mask = value1 | value2 | CPCAP_BIT_CDC_SW;

		if (state->codec_mute == CPCAP_AUDIO_CODEC_UNMUTE)
			reg_changes.value = reg_changes.mask;

		logged_cpcap_write(state->cpcap, CPCAP_REG_RXCOA,
					reg_changes.value, reg_changes.mask);
	}

	if (state->stdac_mute != prev_state->stdac_mute) {
		value1 = cpcap_audio_get_stdac_output_amp_switches(
				prev_state->stdac_primary_speaker,
				prev_state->stdac_primary_balance);

		value2 = cpcap_audio_get_stdac_output_amp_switches(
				prev_state->stdac_secondary_speaker,
				prev_state->stdac_primary_balance);

		reg_changes.mask = value1 | value2 | CPCAP_BIT_ST_DAC_SW;

		if (state->stdac_mute == CPCAP_AUDIO_STDAC_UNMUTE)
			reg_changes.value = reg_changes.mask;

		logged_cpcap_write(state->cpcap, CPCAP_REG_RXSDOA,
					reg_changes.value, reg_changes.mask);
	}

/*hack*/
               logged_cpcap_write(state->cpcap, CPCAP_REG_RXCOA,0x1e02,0x1FFF );
}


static void cpcap_audio_configure_codec(struct cpcap_audio_state *state,
				struct cpcap_audio_state *previous_state) {
	const unsigned int CODEC_FREQ_MASK = CPCAP_BIT_CDC_CLK0
		| CPCAP_BIT_CDC_CLK1 | CPCAP_BIT_CDC_CLK2;
	const unsigned int CODEC_RESET_FREQ_MASK = CODEC_FREQ_MASK
		| CPCAP_BIT_CDC_CLOCK_TREE_RESET;

	static unsigned int prev_codec_data = 0x0, prev_cdai_data = 0x0;

	if (is_codec_changed(state, previous_state)) {
		unsigned int temp_codec_rate = state->codec_rate;

		struct cpcap_regacc cdai_changes = { 0 };
		struct cpcap_regacc codec_changes = { 0 };
		int codec_freq_config = 0;

		if (state->rat_type == CPCAP_AUDIO_RAT_CDMA)
			codec_freq_config = (CPCAP_BIT_CDC_CLK0
					| CPCAP_BIT_CDC_CLK1) ; /* 19.2Mhz */
		else
			codec_freq_config = CPCAP_BIT_CDC_CLK2 ; /* 26Mhz */

		/* If a codec is already in use, reset codec to initial state */

		if (previous_state->codec_mode != CPCAP_AUDIO_CODEC_OFF) {
			codec_changes.mask = prev_codec_data
				| CPCAP_BIT_DF_RESET
				| CPCAP_BIT_CDC_CLOCK_TREE_RESET;

			logged_cpcap_write(state->cpcap, CPCAP_REG_CC,
				codec_changes.value, codec_changes.mask);

			prev_codec_data = 0;
			previous_state->codec_mode = CPCAP_AUDIO_CODEC_OFF;
		}

		temp_codec_rate &= 0x0000000F;
		temp_codec_rate = temp_codec_rate << 9;

		switch (state->codec_mode) {
		case CPCAP_AUDIO_CODEC_LOOPBACK:
		case CPCAP_AUDIO_CODEC_ON:
			if (state->codec_primary_speaker !=

					CPCAP_AUDIO_OUT_NONE &&
				state->dai_config ==
					CPCAP_AUDIO_DAI_CONFIG_NORMAL) {
				codec_changes.value |= CPCAP_BIT_CDC_EN_RX;
			}

			if (needs_cpcap_mic1(state->microphone))
				codec_changes.value |= CPCAP_BIT_MIC1_CDC_EN;

			if (needs_cpcap_mic2(state->microphone))
				codec_changes.value |= CPCAP_BIT_MIC2_CDC_EN;

			/* use input HPF if either mic is enabled */
			if ((codec_changes.value &
					(CPCAP_BIT_MIC1_CDC_EN |
						CPCAP_BIT_MIC2_CDC_EN)) != 0)
				codec_changes.value |= CPCAP_BIT_AUDIHPF_0 |
				CPCAP_BIT_AUDIHPF_1;


		/* falling through intentionally */
		case CPCAP_AUDIO_CODEC_CLOCK_ONLY:
			codec_changes.value |=
				(codec_freq_config | temp_codec_rate |
				CPCAP_BIT_DF_RESET);
			cdai_changes.value |= CPCAP_BIT_CDC_CLK_EN;
			break;

		case CPCAP_AUDIO_CODEC_OFF:
			cdai_changes.value |= CPCAP_BIT_SMB_CDC;
			break;

		default:
			break;
		}

		/* Multimedia uses CLK_IN0, incall uses CLK_IN1 */
//		if (state->rat_type != CPCAP_AUDIO_RAT_NONE)
//			cdai_changes.value |= CPCAP_BIT_CLK_IN_SEL;


		/* Bus bus config: Codec is always master*/
		cdai_changes.value |= CPCAP_BIT_CDC_DIG_AUD_FS0;

		/*Codec uses independent PLL in normal operation*/
		if (state->dai_config == CPCAP_AUDIO_DAI_CONFIG_NORMAL)
			cdai_changes.value |= CPCAP_BIT_CDC_PLL_SEL;

		if (state->rat_type == CPCAP_AUDIO_RAT_CDMA) {
			/* CDMA BP requires I2S mode */
			cdai_changes.value |= CPCAP_BIT_CDC_DIG_AUD_FS1 |
							CPCAP_BIT_CLK_INV;
		} else if (state->rat_type == CPCAP_AUDIO_RAT_UMTS) {
			/* UMTS BP requires network mode */
			cdai_changes.value |= CPCAP_BIT_MIC2_TIMESLOT0;
		} else {
			if (state->dai_config ==
					CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_1) {
				/* duplex I2S operation on DAI1 (untested) */
				cdai_changes.value |= CPCAP_BIT_DIG_AUD_IN |
					CPCAP_BIT_CDC_DIG_AUD_FS1 | CPCAP_BIT_CLK_INV;

			}



			if (needs_outboard_adc(state->microphone)) {
				/*if we're using the outboard ADC, then it
				 * has slots 0 and 1 so cpcap must be on
				 * 2 and 3. */
				cdai_changes.value |=
						CPCAP_BIT_MIC1_RX_TIMESLOT1 |
						CPCAP_BIT_MIC2_TIMESLOT0 |
						CPCAP_BIT_MIC2_TIMESLOT1;
			} else {
				if (needs_cpcap_mic2(state->microphone) &&
					!needs_cpcap_mic1(state->microphone)) {
					/*if we're using *only* cpcap mic2,
					 * put it on slot0 for mono capture.
					 * This requires mic1 be on a slot other
					 * than 0 due to cpcap contention logic.
					 */
					cdai_changes.value |=
						CPCAP_BIT_MIC1_RX_TIMESLOT0;
				} else {
					/*otherwise put MIC1 on slot0 and MIC2
					 * on slot 1 for either MIC1 mono or
					 * MIC1+MIC2 stereo configurations
					 */
					cdai_changes.value |=
						CPCAP_BIT_MIC2_TIMESLOT0;
				}
			}
		}

		/* FIRST, make sure the frequency config is right... */
		logged_cpcap_write(state->cpcap, CPCAP_REG_CC,
					codec_freq_config, CODEC_FREQ_MASK);


		/* Next, write the CDAI if it's changed */
		if (prev_cdai_data != cdai_changes.value) {
			cdai_changes.mask = cdai_changes.value
				| prev_cdai_data;
			prev_cdai_data = cdai_changes.value;


			logged_cpcap_write(state->cpcap, CPCAP_REG_CDI,
					cdai_changes.value, cdai_changes.mask);

			/* Clock tree change -- reset and wait */
			codec_freq_config |= CPCAP_BIT_CDC_CLOCK_TREE_RESET;

			logged_cpcap_write(state->cpcap, CPCAP_REG_CC,
				codec_freq_config, CODEC_RESET_FREQ_MASK);


			/* Wait for clock tree reset to complete */
			mdelay(CLOCK_TREE_RESET_TIME);
		}

		/* Clear old settings */
		codec_changes.mask = codec_changes.value | prev_codec_data;
		prev_codec_data    = codec_changes.value;

		logged_cpcap_write(state->cpcap, CPCAP_REG_CC,

				codec_changes.value, codec_changes.mask);

	}
}
static void cpcap_audio_configure_stdac(struct cpcap_audio_state *state,
				struct cpcap_audio_state *previous_state)
{
	const unsigned int SDAC_FREQ_MASK = CPCAP_BIT_ST_DAC_CLK0
			| CPCAP_BIT_ST_DAC_CLK1 | CPCAP_BIT_ST_DAC_CLK2;
	const unsigned int SDAC_RESET_FREQ_MASK = SDAC_FREQ_MASK
					| CPCAP_BIT_ST_CLOCK_TREE_RESET;

	static unsigned int prev_stdac_data, prev_sdai_data;

	if (is_stdac_changed(state, previous_state)) {
		unsigned int temp_stdac_rate = state->stdac_rate;
		struct cpcap_regacc sdai_changes = { 0 };
		struct cpcap_regacc stdac_changes = { 0 };
		int stdac_freq_config = 0;

		if (state->rat_type == CPCAP_AUDIO_RAT_CDMA)
			stdac_freq_config = (CPCAP_BIT_ST_DAC_CLK0
					| CPCAP_BIT_ST_DAC_CLK1) ; /*19.2Mhz*/
		else
			stdac_freq_config = CPCAP_BIT_ST_DAC_CLK2 ; /* 26Mhz */

		/* We need to turn off stdac before changing its settings */
		if (previous_state->stdac_mode != CPCAP_AUDIO_STDAC_OFF) {
			stdac_changes.mask = prev_stdac_data |
					CPCAP_BIT_DF_RESET_ST_DAC |
					CPCAP_BIT_ST_CLOCK_TREE_RESET;

			logged_cpcap_write(state->cpcap, CPCAP_REG_SDAC,
				stdac_changes.value, stdac_changes.mask);

			prev_stdac_data = 0;
			previous_state->stdac_mode = CPCAP_AUDIO_STDAC_OFF;
		}

		temp_stdac_rate &= 0x0000000F;
		temp_stdac_rate = temp_stdac_rate << 4;

		switch (state->stdac_mode) {
		case CPCAP_AUDIO_STDAC_ON:
			stdac_changes.value |= CPCAP_BIT_ST_DAC_EN;

		/* falling through intentionally */
		case CPCAP_AUDIO_STDAC_CLOCK_ONLY:
			stdac_changes.value |= temp_stdac_rate |
				CPCAP_BIT_DF_RESET_ST_DAC | stdac_freq_config;
			sdai_changes.value |= CPCAP_BIT_ST_CLK_EN;
			break;

		case CPCAP_AUDIO_STDAC_OFF:
		default:
			break;
		}

		if (state->rat_type != CPCAP_AUDIO_RAT_NONE)
			sdai_changes.value |= CPCAP_BIT_ST_DAC_CLK_IN_SEL;


		if (state->dai_config == CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_0) {
			/* when using DAI0, follow the codec configuration:
			 * 4-slot network, R on slot 0, L on slot 1 */
			sdai_changes.value |= CPCAP_BIT_ST_DIG_AUD_FS0
					| CPCAP_BIT_ST_L_TIMESLOT0;
		} else {
			/* -True I2S mode
			 * -STDAC is Master of Fsync Bclk
			 * -STDAC uses DAI1 */
			sdai_changes.value |= CPCAP_BIT_DIG_AUD_IN_ST_DAC
					| CPCAP_BIT_ST_DIG_AUD_FS0
					| CPCAP_BIT_ST_DIG_AUD_FS1
					| CPCAP_BIT_ST_CLK_INV;
		}

		logged_cpcap_write(state->cpcap, CPCAP_REG_SDAC,
					stdac_freq_config, SDAC_FREQ_MASK);

		/* Next, write the SDACDI if it's changed */
		if (prev_sdai_data != sdai_changes.value) {
			sdai_changes.mask = sdai_changes.value
						| prev_sdai_data;
			prev_sdai_data = sdai_changes.value;

			logged_cpcap_write(state->cpcap, CPCAP_REG_SDACDI,
					sdai_changes.value, sdai_changes.mask);

			/* Clock tree change -- reset and wait */
			stdac_freq_config |= CPCAP_BIT_ST_CLOCK_TREE_RESET;

			logged_cpcap_write(state->cpcap, CPCAP_REG_SDAC,
				stdac_freq_config, SDAC_RESET_FREQ_MASK);


			/* Wait for clock tree reset to complete */
			mdelay(CLOCK_TREE_RESET_TIME);
		}

		/* Clear old settings */
		stdac_changes.mask = stdac_changes.value | prev_stdac_data;
		prev_stdac_data = stdac_changes.value;

		logged_cpcap_write(state->cpcap, CPCAP_REG_SDAC,
			stdac_changes.value, stdac_changes.mask);
	}
}

static void cpcap_audio_configure_analog_source(
	struct cpcap_audio_state *state,
	struct cpcap_audio_state *previous_state)
{
	if (state->analog_source != previous_state->analog_source) {
		struct cpcap_regacc ext_changes = { 0 };
		static unsigned int prev_ext_data;
		switch (state->analog_source) {
		case CPCAP_AUDIO_ANALOG_SOURCE_STEREO:
			ext_changes.value |= CPCAP_BIT_MONO_EXT0 |
			CPCAP_BIT_PGA_IN_R_SW | CPCAP_BIT_PGA_IN_L_SW;
			break;
		case CPCAP_AUDIO_ANALOG_SOURCE_L:
			ext_changes.value |= CPCAP_BIT_MONO_EXT1 |
						CPCAP_BIT_PGA_IN_L_SW;
			break;
		case CPCAP_AUDIO_ANALOG_SOURCE_R:
			ext_changes.value |= CPCAP_BIT_MONO_EXT1 |
						CPCAP_BIT_PGA_IN_R_SW;
			break;
		default:
			break;
		}

		ext_changes.mask = ext_changes.value | prev_ext_data;

		prev_ext_data = ext_changes.value;

		logged_cpcap_write(state->cpcap, CPCAP_REG_RXEPOA,
				ext_changes.value, ext_changes.mask);
	}
}
static void cpcap_audio_configure_input_gains(
	struct cpcap_audio_state *state,
	struct cpcap_audio_state *previous_state)
{
	if (state->input_gain != previous_state->input_gain) {
		struct cpcap_regacc reg_changes = { 0 };
		unsigned int temp_input_gain = state->input_gain & 0x0000001F;

		reg_changes.value |= ((temp_input_gain << 5) | temp_input_gain);

		reg_changes.mask = 0x3FF;

		logged_cpcap_write(state->cpcap, CPCAP_REG_TXMP,
				reg_changes.value, reg_changes.mask);
	}
}

static void cpcap_audio_configure_output_gains(
	struct cpcap_audio_state *state,
	struct cpcap_audio_state *previous_state)
{
	if (state->output_gain != previous_state->output_gain) {
		struct cpcap_regacc reg_changes = { 0 };
		unsigned int temp_output_gain = state->output_gain & 0x0000000F;

		reg_changes.value |=
		    ((temp_output_gain << 2) | (temp_output_gain << 8) |
		     (temp_output_gain << 12));

		reg_changes.mask = 0xFF3C;

		logged_cpcap_write(state->cpcap, CPCAP_REG_RXVC,
				reg_changes.value, reg_changes.mask);
	}
}

static void cpcap_audio_configure_output(
	struct cpcap_audio_state *state,
	struct cpcap_audio_state *previous_state)
{
	static unsigned int prev_aud_out_data;

	if (is_output_changed(previous_state, state) ||
	    is_codec_changed(previous_state, state) ||
	    is_stdac_changed(previous_state, state)) {
		bool activate_ext_loudspeaker = false;
		struct cpcap_regacc reg_changes = { 0 };


		cpcap_audio_set_output_amp_switches(state);

		activate_ext_loudspeaker = cpcap_audio_set_bits_for_speaker(
						state->codec_primary_speaker,
						 state->codec_primary_balance,
						 &(reg_changes.value));

		activate_ext_loudspeaker = activate_ext_loudspeaker ||
					cpcap_audio_set_bits_for_speaker(
						state->codec_secondary_speaker,
						 CPCAP_AUDIO_BALANCE_NEUTRAL,
						 &(reg_changes.value));

		activate_ext_loudspeaker = activate_ext_loudspeaker ||
					 cpcap_audio_set_bits_for_speaker(
						state->stdac_primary_speaker,
						 state->stdac_primary_balance,
						 &(reg_changes.value));

		activate_ext_loudspeaker = activate_ext_loudspeaker ||
					cpcap_audio_set_bits_for_speaker(
						state->stdac_secondary_speaker,
						 CPCAP_AUDIO_BALANCE_NEUTRAL,
						 &(reg_changes.value));

		activate_ext_loudspeaker = activate_ext_loudspeaker ||
					cpcap_audio_set_bits_for_speaker(
						state->ext_primary_speaker,
						 state->ext_primary_balance,
						 &(reg_changes.value));

		activate_ext_loudspeaker = activate_ext_loudspeaker ||
					cpcap_audio_set_bits_for_speaker(
						state->ext_secondary_speaker,
						 CPCAP_AUDIO_BALANCE_NEUTRAL,
						 &(reg_changes.value));

		reg_changes.mask = reg_changes.value | prev_aud_out_data;

		prev_aud_out_data = reg_changes.value;

		logged_cpcap_write(state->cpcap, CPCAP_REG_RXOA,
				reg_changes.value, reg_changes.mask);
               /* enable speaker power */
               logged_cpcap_write(state->cpcap, CPCAP_REG_VAUDIOC,0, CPCAP_BIT_AUDIO_LOW_PWR);
/*hack*/
               logged_cpcap_write(state->cpcap, CPCAP_REG_RXOA,0x202,0x202 );
/*hack*/
               logged_cpcap_write(state->cpcap, CPCAP_REG_RXCOA,0x1e02,0x1FFF );



	}
}

#define CODEC_LOOPBACK_CHANGED() \
	((state->codec_mode != previous_state->codec_mode) && \
	 (state->codec_mode == CPCAP_AUDIO_CODEC_LOOPBACK || \
	  previous_state->codec_mode == CPCAP_AUDIO_CODEC_LOOPBACK))

static void cpcap_audio_configure_input(
	struct cpcap_audio_state *state,
	struct cpcap_audio_state *previous_state) {
	static unsigned int prev_input_data = 0x0;
	struct cpcap_regacc reg_changes = { 0 };

	if (state->microphone != previous_state->microphone ||
		CODEC_LOOPBACK_CHANGED()) {

		if (state->codec_mode == CPCAP_AUDIO_CODEC_LOOPBACK)
			reg_changes.value |= CPCAP_BIT_DLM;

		if (cpcap_audio_hw_has_hs_driver() &&
			(previous_state->microphone &
				CPCAP_AUDIO_IN_HEADSET) != 0) {
				logged_cpcap_write(state->cpcap,
						CPCAP_REG_GPIO4,
						0, CPCAP_BIT_GPIO4DRV);
		}

		if ((state->microphone & CPCAP_AUDIO_IN_DUAL_EXTERNAL) != 0) {
			reg_changes.value |= CPCAP_BIT_RX_R_ENCODE
				| CPCAP_BIT_RX_L_ENCODE;
		} else {
			/*HANDSET and HEADSET mics are mutually exclusive.
			 *if both are selected, HANDSET will win
			 */
			if ((state->microphone & CPCAP_AUDIO_IN_HANDSET) != 0) {
				reg_changes.value |= CPCAP_BIT_MB_ON1R
						| CPCAP_BIT_MIC1_MUX |
						CPCAP_BIT_MIC1_PGA_EN;
			} else if ((state->microphone &
					CPCAP_AUDIO_IN_HEADSET) != 0) {
				reg_changes.value |= CPCAP_BIT_HS_MIC_MUX
						| CPCAP_BIT_MIC1_PGA_EN;
				if (cpcap_audio_hw_has_hs_driver()) {
					logged_cpcap_write(state->cpcap,
						CPCAP_REG_GPIO4,
						CPCAP_BIT_GPIO4DRV,
						CPCAP_BIT_GPIO4DRV);
				}
			}

			/*NOTE an implementation quirk: SECONDARY and TERTIARY
			 * are equivalent on 2-mic hardware.
			 * This is unintentional but harmless */
			if (needs_cpcap_mic2(state->microphone)) {
				reg_changes.value |= CPCAP_BIT_MB_ON1L
						| CPCAP_BIT_MIC2_MUX |
						CPCAP_BIT_MIC2_PGA_EN;
			}

			if (cpcap_audio_hw_has_mic3_switch()) {
				if (needs_cpcap_mic3_switch(
						state->microphone)) {
					logged_cpcap_write(state->cpcap,
							CPCAP_REG_GPIO4,
							0, CPCAP_BIT_GPIO4DRV);
				} else {
					logged_cpcap_write(state->cpcap,
							CPCAP_REG_GPIO4,
							CPCAP_BIT_GPIO4DRV,
							CPCAP_BIT_GPIO4DRV);
				}
			}

		}

		reg_changes.mask = reg_changes.value | prev_input_data;
		prev_input_data = reg_changes.value;

		logged_cpcap_write(state->cpcap, CPCAP_REG_TXI,
					reg_changes.value, reg_changes.mask);
	}
}

static void cpcap_audio_configure_power(int power)
{
	static int previous_power = -1;

	CPCAP_AUDIO_DEBUG_LOG("%s() called with power= %d\n", __func__, power);

	if (power != previous_power) {

		if (IS_ERR(audio_regu)) {
			CPCAP_AUDIO_ERROR_LOG("audio_regu not valid for"
							"regulator setup\n");
			return;
		}

		if (power) {
			regulator_enable(audio_regu);
			regulator_set_mode(audio_regu, REGULATOR_MODE_NORMAL);
		} else {
			printk(KERN_INFO "turning off regulator\n");
			regulator_set_mode(audio_regu, REGULATOR_MODE_STANDBY);
			regulator_disable(audio_regu);
		}

		previous_power = power;

		if (power)
			mdelay(SLEEP_ACTIVATE_POWER);
	}

}

static void cpcap_audio_reguister_dump(struct cpcap_audio_state *state)
{
	unsigned short reg_val = 0;

	cpcap_regacc_read(state->cpcap, CPCAP_REG_VAUDIOC, &reg_val);
	printk(KERN_INFO "0x200 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_CC, &reg_val);
	printk(KERN_INFO "0x201 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_CDI, &reg_val);
	printk(KERN_INFO "0x202 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_SDAC, &reg_val);
	printk(KERN_INFO "0x203 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_SDACDI, &reg_val);
	printk(KERN_INFO "0x204 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_TXI, &reg_val);
	printk(KERN_INFO "0x205 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_TXMP, &reg_val);
	printk(KERN_INFO "0x206 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXOA, &reg_val);
	printk(KERN_INFO "0x207 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXVC, &reg_val);
	printk(KERN_INFO "0x208 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXCOA, &reg_val);
	printk(KERN_INFO "0x209 = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXSDOA, &reg_val);
	printk(KERN_INFO "0x20A = %x\n", reg_val);
	cpcap_regacc_read(state->cpcap, CPCAP_REG_RXEPOA, &reg_val);
	printk(KERN_INFO "0x20B = %x\n", reg_val);
}

void cpcap_audio_set_audio_state(struct cpcap_audio_state *state)
{
	struct cpcap_audio_state *previous_state = &previous_state_struct;

	if (state->codec_mute == CPCAP_AUDIO_CODEC_BYPASS_LOOP)
		state->codec_mode = CPCAP_AUDIO_CODEC_ON;

	if (state->codec_mode == CPCAP_AUDIO_CODEC_OFF ||
	    state->codec_mode == CPCAP_AUDIO_CODEC_CLOCK_ONLY)
		state->codec_mute = CPCAP_AUDIO_CODEC_MUTE;
	else
		state->codec_mute = CPCAP_AUDIO_CODEC_UNMUTE;

	if (state->stdac_mode != CPCAP_AUDIO_STDAC_ON)
		state->stdac_mute = CPCAP_AUDIO_STDAC_MUTE;
	else
		state->stdac_mute = CPCAP_AUDIO_STDAC_UNMUTE;

	/* @TODO: STDAC_CLOCK_ONLY can be used when AUX_I2S is the
	 * only speaker to save some current.  AUX_I2S is to stdac
	 * as BT_MONO is to phone codec.
	 */
	if (state->stdac_mode == CPCAP_AUDIO_STDAC_CLOCK_ONLY)
		state->stdac_mode = CPCAP_AUDIO_STDAC_ON;

	if ((state->codec_mode != CPCAP_AUDIO_CODEC_OFF &&
	     state->codec_mode != CPCAP_AUDIO_CODEC_CLOCK_ONLY) ||
	    state->stdac_mode != CPCAP_AUDIO_STDAC_OFF ||
	    (state->codec_primary_speaker != CPCAP_AUDIO_OUT_NONE &&
	     state->codec_primary_speaker != CPCAP_AUDIO_OUT_BT_MONO) ||
	    state->stdac_primary_speaker != CPCAP_AUDIO_OUT_NONE ||
	    state->ext_primary_speaker != CPCAP_AUDIO_OUT_NONE ||
	    (state->microphone != CPCAP_AUDIO_IN_NONE &&
		state->microphone != CPCAP_AUDIO_IN_BT_MONO))
		cpcap_audio_configure_power(1);

	if (is_speaker_turning_off(state, previous_state))
		cpcap_audio_configure_output(state, previous_state);

	if (is_codec_changed(state, previous_state)) {
		int codec_mute = state->codec_mute;

		state->codec_mute = CPCAP_AUDIO_CODEC_MUTE;

		cpcap_audio_configure_aud_mute(state, previous_state);

		previous_state->codec_mute = state->codec_mute;

		state->codec_mute = codec_mute;

		cpcap_audio_configure_codec(state, previous_state);
	}
	if (is_stdac_changed(state, previous_state)) {
		int stdac_mute = state->stdac_mute;

		state->stdac_mute = CPCAP_AUDIO_STDAC_MUTE;

		cpcap_audio_configure_aud_mute(state, previous_state);

		previous_state->stdac_mute = state->stdac_mute;

		state->stdac_mute = stdac_mute;

		cpcap_audio_configure_stdac(state, previous_state);
	}


	cpcap_audio_configure_analog_source(state, previous_state);

	cpcap_audio_configure_input(state, previous_state);

	cpcap_audio_configure_input_gains(state, previous_state);

	cpcap_audio_configure_output(state, previous_state);

	cpcap_audio_configure_output_gains(state, previous_state);

	cpcap_audio_configure_aud_mute(state, previous_state);

	if ((state->codec_mode == CPCAP_AUDIO_CODEC_OFF ||
	     state->codec_mode == CPCAP_AUDIO_CODEC_CLOCK_ONLY) &&
	    state->stdac_mode == CPCAP_AUDIO_STDAC_OFF &&
	    (state->codec_primary_speaker == CPCAP_AUDIO_OUT_NONE ||
	     state->codec_primary_speaker == CPCAP_AUDIO_OUT_BT_MONO) &&
	    state->stdac_primary_speaker == CPCAP_AUDIO_OUT_NONE &&
	    state->ext_primary_speaker == CPCAP_AUDIO_OUT_NONE &&
	    (state->microphone == CPCAP_AUDIO_IN_NONE ||
		state->microphone == CPCAP_AUDIO_IN_BT_MONO))
		cpcap_audio_configure_power(0);

	previous_state_struct = *state;

	cpcap_audio_reguister_dump(state);
}

void cpcap_audio_init(struct cpcap_audio_state *state)
{
	CPCAP_AUDIO_DEBUG_LOG("%s() called\n", __func__);

	printk("%s() level 1\n",__func__);
	logged_cpcap_write(state->cpcap, CPCAP_REG_CC, 0, 0xFFFF);
	logged_cpcap_write(state->cpcap, CPCAP_REG_CDI, 0, 0xBFFF);
	logged_cpcap_write(state->cpcap, CPCAP_REG_SDAC, 0, 0xFFF);
	logged_cpcap_write(state->cpcap, CPCAP_REG_SDACDI, 0, 0x3FFF);
	logged_cpcap_write(state->cpcap, CPCAP_REG_TXI, 0, 0xFDF);
	logged_cpcap_write(state->cpcap, CPCAP_REG_TXMP, 0, 0xFFF);
	logged_cpcap_write(state->cpcap, CPCAP_REG_RXOA, 0, 0x1FF);
	/* logged_cpcap_write(state->cpcap, CPCAP_REG_RXVC, 0, 0xFFF); */
	logged_cpcap_write(state->cpcap, CPCAP_REG_RXCOA, 0, 0x7FF);
	logged_cpcap_write(state->cpcap, CPCAP_REG_RXSDOA, 0, 0x1FFF);
	logged_cpcap_write(state->cpcap, CPCAP_REG_RXEPOA, 0, 0x7FFF);

	printk("%s() level 2\n",__func__);

	if (cpcap_audio_hw_has_hs_driver() || cpcap_audio_hw_has_mic3_switch())
	        logged_cpcap_write(state->cpcap, CPCAP_REG_GPIO4,
			   CPCAP_BIT_GPIO4DIR, CPCAP_BIT_GPIO4DIR);
/*hack*/
	printk("%s() level 3\n",__func__);
        logged_cpcap_write(state->cpcap, CPCAP_REG_RXCOA,0x1e02,0x1FFF );
	audio_regu = regulator_get(NULL, "vaudio");
	printk("%s() level 4\n",__func__);
	if (IS_ERR(audio_regu))
		CPCAP_AUDIO_ERROR_LOG("could not get regulator for audio\n");
}
int cpcap_audio_is_cdma_shadow(void)
{
	/*@TODO FIX ME!
	  need to discriminate between Shadow and Droid2*/
	//return is_cdma_phone();
	return 0;
}

