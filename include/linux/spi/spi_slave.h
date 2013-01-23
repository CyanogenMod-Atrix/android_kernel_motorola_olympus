#ifndef __SPI_SLAVE_H
#define __SPI_SLAVE_H

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>

extern struct bus_type spi_slave_bus_type;

struct spi_slave_device {
	struct device		dev;
	struct spi_slave	*slave;
	u32			max_speed_hz;
	u8			chip_select;
	u8			mode;
#define	SPI_CPHA	0x01			/* clock phase */
#define	SPI_CPOL	0x02			/* clock polarity */
#define	SPI_MODE_0	(0|0)			/* (original MicroWire) */
#define	SPI_MODE_1	(0|SPI_CPHA)
#define	SPI_MODE_2	(SPI_CPOL|0)
#define	SPI_MODE_3	(SPI_CPOL|SPI_CPHA)
#define	SPI_CS_HIGH	0x04			/* chipselect active high? */
#define	SPI_LSB_FIRST	0x08			/* per-word bits-on-wire */
#define	SPI_3WIRE	0x10			/* SI/SO signals shared */
#define	SPI_LOOP	0x20			/* loopback mode */
#define	SPI_NO_CS	0x40			/* 1 dev/bus, no chipselect */
#define	SPI_READY	0x80			/* slave pulls low to pause */
	u8			bits_per_word;
	int			irq;
	void			*controller_state;
	void			*controller_data;
	char			modalias[SPI_NAME_SIZE];
};

static inline struct spi_slave_device *to_spi_slave_device(struct device *dev)
{
	return dev ? container_of(dev, struct spi_slave_device, dev) : NULL;
}

/* most drivers won't need to care about device refcounting */
static inline struct spi_slave_device *spi_slave_dev_get(struct spi_slave_device *spi)
{
	return (spi && get_device(&spi->dev)) ? spi : NULL;
}

static inline void spi_slave_dev_put(struct spi_slave_device *spi)
{
	if (spi)
		put_device(&spi->dev);
}

static inline void *spi_slave_get_ctldata(struct spi_slave_device *spi)
{
	return spi->controller_state;
}

static inline void spi_slave_set_ctldata(struct spi_slave_device *spi, void *state)
{
	spi->controller_state = state;
}

static inline void spi_slave_set_drvdata(struct spi_slave_device *spi, void *data)
{
	dev_set_drvdata(&spi->dev, data);
}

static inline void *spi_slave_get_drvdata(struct spi_slave_device *spi)
{
	return dev_get_drvdata(&spi->dev);
}

struct spi_slave_message;

struct spi_slave_driver {
	const struct spi_device_id *id_table;
	int			(*probe)(struct spi_slave_device *spi);
	int			(*remove)(struct spi_slave_device *spi);
	void			(*shutdown)(struct spi_slave_device *spi);
	int			(*suspend)(struct spi_slave_device *spi, pm_message_t mesg);
	int			(*resume)(struct spi_slave_device *spi);
	struct device_driver	driver;
};

static inline struct spi_slave_driver *to_spi_slave_driver(struct device_driver *drv)
{
	return drv ? container_of(drv, struct spi_slave_driver, driver) : NULL;
}

extern int spi_slave_register_driver(struct spi_slave_driver *sdrv);

static inline void spi_slave_unregister_driver(struct spi_slave_driver *sdrv)
{
	if (sdrv)
		driver_unregister(&sdrv->driver);
}

struct spi_slave {
	struct device	dev;
	s16			bus_num;
	u16			num_chipselect;
	u16			dma_alignment;
	u16			mode_bits;
	u16			flags;
#define SPI_SLAVE_HALF_DUPLEX	BIT(0)		/* can't do full duplex */
#define SPI_SLAVE_NO_RX	BIT(1)		/* can't do buffer read */
#define SPI_SLAVE_NO_TX	BIT(2)		/* can't do buffer write */
	int			(*setup)(struct spi_slave_device *spi);
	int			(*transfer)(struct spi_slave_device *spi,
						struct spi_slave_message *mesg);
	void			(*cleanup)(struct spi_slave_device *spi);
};

static inline void *spi_slave_get_devdata(struct spi_slave *slave)
{
	return dev_get_drvdata(&slave->dev);
}

static inline void spi_slave_set_devdata(struct spi_slave *slave, void *data)
{
	dev_set_drvdata(&slave->dev, data);
}

static inline struct spi_slave *spi_slave_get(struct spi_slave *slave)
{
	if (!slave || !get_device(&slave->dev))
		return NULL;
	return slave;
}

static inline void spi_slave_put(struct spi_slave *slave)
{
	if (slave)
		put_device(&slave->dev);
}

extern struct spi_slave *
spi_alloc_slave(struct device *host, unsigned size);

extern int spi_register_slave(struct spi_slave *slave);
extern void spi_unregister_slave(struct spi_slave *slave);

extern struct spi_slave *spi_busnum_to_slave(u16 busnum);


struct spi_slave_transfer {
	u8	*tx_buf;
	u8		*rx_buf;
	unsigned	len;

	dma_addr_t	tx_dma;
	dma_addr_t	rx_dma;

	unsigned	cs_change:1;
	u8		bits_per_word;
	u16		delay_usecs;
	u32		speed_hz;

	struct list_head transfer_list;
};

struct spi_slave_message {
	struct list_head	transfers;
	struct spi_slave_device	*spi;
	unsigned		is_dma_mapped:1;
	void			(*complete)(void *context);
	void			*context;
	unsigned		actual_length;
	int			status;
	struct list_head	queue;
	void			*state;
};

extern void spi_slave_message_init(struct spi_slave_message *m);

extern void
spi_slave_message_add_tail(struct spi_slave_transfer *t, struct spi_slave_message *m);

static inline void
spi_slave_transfer_del(struct spi_slave_transfer *t)
{
	list_del(&t->transfer_list);
}

static inline struct spi_slave_message *spi_slave_message_alloc(unsigned ntrans, gfp_t flags)
{
	struct spi_slave_message *m;

	m = kzalloc(sizeof(struct spi_slave_message)
			+ ntrans * sizeof(struct spi_slave_transfer),
			flags);
	if (m) {
		int i;
		struct spi_slave_transfer *t = (struct spi_slave_transfer *)(m + 1);

		INIT_LIST_HEAD(&m->transfers);
		for (i = 0; i < ntrans; i++, t++)
			spi_slave_message_add_tail(t, m);
	}
	return m;
}

static inline void spi_slave_message_free(struct spi_slave_message *m)
{
	kfree(m);
}

extern int spi_slave_setup(struct spi_slave_device *spi);
extern int spi_slave_async(struct spi_slave_device *spi, struct spi_slave_message *message);
extern int spi_slave_sync(struct spi_slave_device *spi, struct spi_slave_message *message);

static inline int
spi_slave_write(struct spi_slave_device *spi, u8 *buf, size_t len)
{
	struct spi_slave_transfer	t = {
			.tx_buf		= buf,
			.len		= len,
		};
	struct spi_slave_message	m;

	spi_slave_message_init(&m);
	spi_slave_message_add_tail(&t, &m);
	return spi_slave_sync(spi, &m);
}

static inline int
spi_slave_read(struct spi_slave_device *spi, u8 *buf, size_t len)
{
	struct spi_slave_transfer	t = {
			.rx_buf		= buf,
			.len		= len,
		};
	struct spi_slave_message	m;

	spi_slave_message_init(&m);
	spi_slave_message_add_tail(&t, &m);
	return spi_slave_sync(spi, &m);
}

extern int spi_slave_write_then_read(struct spi_slave_device *spi,
		u8 *txbuf, unsigned n_tx,
		u8 *rxbuf, unsigned n_rx);

static inline ssize_t spi_slave_w8r8(struct spi_slave_device *spi, u8 cmd)
{
	ssize_t			status;
	u8			result;

	status = spi_slave_write_then_read(spi, &cmd, 1, &result, 1);

	return (status < 0) ? status : result;
}

static inline ssize_t spi_slave_w8r16(struct spi_slave_device *spi, u8 cmd)
{
	ssize_t			status;
	u16			result;

	status = spi_slave_write_then_read(spi, &cmd, 1, (u8 *) &result, 2);

	return (status < 0) ? status : result;
}


struct spi_slave_board_info {
	char		modalias[SPI_NAME_SIZE];
	const void	*platform_data;
	void		*controller_data;
	int		irq;
	u32		max_speed_hz;
	u16		bus_num;
	u16		chip_select;
	u8		mode;
};

#ifdef CONFIG_SPI_SLAVE
extern int
spi_slave_register_board_info(struct spi_slave_board_info const *info, unsigned n);
#else
static inline int spi_slave_register_board_info(struct spi_slave_board_info const *info, unsigned n)
{return 0;}
#endif

extern struct spi_slave_device *
spi_slave_alloc_device(struct spi_slave *slave);

extern int
spi_slave_add_device(struct spi_slave_device *spi);

extern struct spi_slave_device *
spi_slave_new_device(struct spi_slave *, struct spi_slave_board_info *);

static inline void
spi_slave_unregister_device(struct spi_slave_device *spi)
{
	if (spi)
		device_unregister(&spi->dev);
}

extern const struct spi_device_id *
spi_get_slave_device_id(const struct spi_slave_device *sdev);

#endif /* __SPI_SLAVE_H */

