/*
 * Sample kset and ktype implementation
 *
 * Copyright (C) 2004-2007 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (C) 2007 Novell Inc.
 *
 * Released under the GPL version 2 only.
 *
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>

/*
 * This is our "object" that we will create a few of and register them with
 * sysfs.
 */

int sfs_hR2S = 4;
EXPORT_SYMBOL(sfs_hR2S);
int sfs_vR2S = 1;
EXPORT_SYMBOL(sfs_vR2S);
int sfs_hSW = 16;
EXPORT_SYMBOL(sfs_hSW);
int sfs_vSW = 1;
EXPORT_SYMBOL(sfs_vSW);
int sfs_hBP = 32;
EXPORT_SYMBOL(sfs_hBP);
int sfs_vBP = 1;
EXPORT_SYMBOL(sfs_vBP);
int sfs_hFP = 32;
EXPORT_SYMBOL(sfs_hFP);
int sfs_vFP = 2;
EXPORT_SYMBOL(sfs_vFP);

bool dsi_param_changed = false;
EXPORT_SYMBOL(dsi_param_changed);

struct dsi_obj {
	struct kobject kobj;
	int h_ref_to_sync;
	int v_ref_to_sync;
	int h_sync_width;
	int v_sync_width;
	int h_back_porch;
	int v_back_porch;
	int h_front_porch;
	int v_front_porch;
};
#define to_dsi_obj(x) container_of(x, struct dsi_obj, kobj)

/* a custom attribute that works just for a struct dsi_obj. */
struct dsi_attribute {
	struct attribute attr;
	ssize_t (*show)(struct dsi_obj *dsi, struct dsi_attribute *attr, char *buf);
	ssize_t (*store)(struct dsi_obj *dsi, struct dsi_attribute *attr, const char *buf, size_t count);
};
#define to_dsi_attr(x) container_of(x, struct dsi_attribute, attr)

/*
 * The default show function that must be passed to sysfs.  This will be
 * called by sysfs for whenever a show function is called by the user on a
 * sysfs file associated with the kobjects we have registered.  We need to
 * transpose back from a "default" kobject to our custom struct dsi_obj and
 * then call the show function for that specific object.
 */
static ssize_t dsi_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct dsi_attribute *attribute;
	struct dsi_obj *dsi;

	attribute = to_dsi_attr(attr);
	dsi = to_dsi_obj(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(dsi, attribute, buf);
}

/*
 * Just like the default show function above, but this one is for when the
 * sysfs "store" is requested (when a value is written to a file.)
 */
static ssize_t dsi_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct dsi_attribute *attribute;
	struct dsi_obj *dsi;

	attribute = to_dsi_attr(attr);
	dsi = to_dsi_obj(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(dsi, attribute, buf, len);
}

/* Our custom sysfs_ops that we will associate with our ktype later on */
static const struct sysfs_ops dsi_sysfs_ops = {
	.show = dsi_attr_show,
	.store = dsi_attr_store,
};

/*
 * The release function for our object.  This is REQUIRED by the kernel to
 * have.  We free the memory held in our object here.
 *
 * NEVER try to get away with just a "blank" release function to try to be
 * smarter than the kernel.  Turns out, no one ever is...
 */
static void dsi_release(struct kobject *kobj)
{
	struct dsi_obj *dsi;

	dsi = to_dsi_obj(kobj);
	kfree(dsi);
}

/*
 * The "dsi" file where the .dsi variable is read from and written to.
 */
static ssize_t dsi_show(struct dsi_obj *dsi_obj, struct dsi_attribute *attr,
			char *buf)
{
	int var = -1;

	if (strcmp(attr->attr.name, "h_ref_to_sync") == 0)
		var = dsi_obj->h_ref_to_sync;
	else if (strcmp(attr->attr.name, "v_ref_to_sync") == 0)
		var = dsi_obj->v_ref_to_sync;
	else if (strcmp(attr->attr.name, "h_sync_width") == 0)
		var = dsi_obj->h_sync_width;
	else if (strcmp(attr->attr.name, "v_sync_width") == 0)
		var = dsi_obj->v_sync_width;
	else if (strcmp(attr->attr.name, "h_back_porch") == 0)
		var = dsi_obj->h_back_porch;
	else if (strcmp(attr->attr.name, "v_back_porch") == 0)
		var = dsi_obj->v_back_porch;
	else if (strcmp(attr->attr.name, "h_front_porch") == 0)
		var = dsi_obj->h_front_porch;
	else if (strcmp(attr->attr.name, "v_front_porch") == 0)
		var = dsi_obj->v_front_porch;

	return sprintf(buf, "%d\n", var);
}

static ssize_t dsi_store(struct dsi_obj *dsi_obj, struct dsi_attribute *attr,
			 const char *buf, size_t count)
{
	int var;

	sscanf(buf, "%du", &var);
	if ((var > 0) && (var <= 64)) {
		if (strcmp(attr->attr.name, "h_ref_to_sync") == 0) {
			dsi_obj->h_ref_to_sync = var;
			sfs_hR2S = var;
		}
		else if (strcmp(attr->attr.name, "v_ref_to_sync") == 0) {
			dsi_obj->v_ref_to_sync = var;
			sfs_vR2S = var;
		}
		else if (strcmp(attr->attr.name, "h_sync_width") == 0) {
			dsi_obj->h_sync_width = var;
			sfs_hSW = var;
		}
		else if (strcmp(attr->attr.name, "v_sync_width") == 0) {
			dsi_obj->v_sync_width = var;
			sfs_vSW = var;
		}
		else if (strcmp(attr->attr.name, "h_back_porch") == 0) {
			dsi_obj->h_back_porch = var;
			sfs_hBP = var;
		}
		else if (strcmp(attr->attr.name, "v_back_porch") == 0) {
			dsi_obj->v_back_porch = var;
			sfs_vBP = var;
		}
		else if (strcmp(attr->attr.name, "h_front_porch") == 0) {
			dsi_obj->h_front_porch = var;
			sfs_hFP = var;
		}
		else if (strcmp(attr->attr.name, "v_front_porch") == 0) {
			dsi_obj->v_front_porch = var;
			sfs_vFP = var;
		}
		dsi_param_changed = true;
	}

	return count;
}

static struct dsi_attribute hR2S_attribute =
	__ATTR(h_ref_to_sync, 0666, dsi_show, dsi_store);
static struct dsi_attribute vR2S_attribute =
	__ATTR(v_ref_to_sync, 0666, dsi_show, dsi_store);
static struct dsi_attribute hSW_attribute =
	__ATTR(h_sync_width, 0666, dsi_show, dsi_store);
static struct dsi_attribute vSW_attribute =
	__ATTR(v_sync_width, 0666, dsi_show, dsi_store);
static struct dsi_attribute hBP_attribute =
	__ATTR(h_back_porch, 0666, dsi_show, dsi_store);
static struct dsi_attribute vBP_attribute =
	__ATTR(v_back_porch, 0666, dsi_show, dsi_store);
static struct dsi_attribute hFP_attribute =
	__ATTR(h_front_porch, 0666, dsi_show, dsi_store);
static struct dsi_attribute vFP_attribute =
	__ATTR(v_front_porch, 0666, dsi_show, dsi_store);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *dsi_default_attrs[] = {
	&hR2S_attribute.attr,
	&vR2S_attribute.attr,
	&hSW_attribute.attr,
	&vSW_attribute.attr,
	&hBP_attribute.attr,
	&vBP_attribute.attr,
	&hFP_attribute.attr,
	&vFP_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * Our own ktype for our kobjects.  Here we specify our sysfs ops, the
 * release function, and the set of default attributes we want created
 * whenever a kobject of this type is registered with the kernel.
 */
static struct kobj_type dsi_ktype = {
	.sysfs_ops = &dsi_sysfs_ops,
	.release = dsi_release,
	.default_attrs = dsi_default_attrs,
};

static struct kset *olympus_kset;
static struct dsi_obj *DSI_obj;

static struct dsi_obj *create_dsi_obj(const char *name)
{
	struct dsi_obj *dsi;
	int retval;

	/* allocate the memory for the whole object */
	dsi = kzalloc(sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return NULL;

	/*
	 * As we have a kset for this kobject, we need to set it before calling
	 * the kobject core.
	 */
	dsi->kobj.kset = olympus_kset;

	/*
	 * Initialize and add the kobject to the kernel.  All the default files
	 * will be created here.  As we have already specified a kset for this
	 * kobject, we don't have to set a parent for the kobject, the kobject
	 * will be placed beneath that kset automatically.
	 */

	dsi->h_ref_to_sync = sfs_hR2S;
	dsi->v_ref_to_sync = sfs_vR2S;
	dsi->h_sync_width = sfs_hSW;
	dsi->v_sync_width = sfs_vSW;
	dsi->h_back_porch = sfs_hBP;
	dsi->v_back_porch = sfs_vBP;
	dsi->h_front_porch = sfs_hFP;
	dsi->v_front_porch = sfs_vFP;
	
	retval = kobject_init_and_add(&dsi->kobj, &dsi_ktype, NULL, "%s", name);
	if (retval) {
		kobject_put(&dsi->kobj);
		return NULL;
	}

	/*
	 * We are always responsible for sending the uevent that the kobject
	 * was added to the system.
	 */
	kobject_uevent(&dsi->kobj, KOBJ_ADD);

	return dsi;
}

static void destroy_dsi_obj(struct dsi_obj *dsi)
{
	kobject_put(&dsi->kobj);
}

static int __init olympus_sysfs_init(void)
{
	/*
	 * Create a kset with the name of "kset_example",
	 * located under /sys/kernel/
	 */
	olympus_kset = kset_create_and_add("olympus", NULL, kernel_kobj);
	if (!olympus_kset)
		return -ENOMEM;

	/*
	 * Create object and register them with our kset
	 */
	DSI_obj = create_dsi_obj("dsi");
	if (!DSI_obj)
		goto DSI_error;

	return 0;

DSI_error:
	return -EINVAL;
}

static void __exit olympus_sysfs_exit(void)
{
	destroy_dsi_obj(DSI_obj);
	kset_unregister(olympus_kset);
}

module_init(olympus_sysfs_init);
module_exit(olympus_sysfs_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Krystian Plonski <szeejk@gmail.com>");
