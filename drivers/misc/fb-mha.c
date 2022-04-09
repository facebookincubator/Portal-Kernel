/*
 * DSPG DBMDX MH Acousting interface driver
 *
 * Copyright (c) 2019, Facebook Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>

#include <sound/fb-mha-adapter-if.h>

#include <uapi/sound/fb-mha.h>

#ifdef MHA_DEBUG
#define MHA_HEX_DUMP(_name_, _buf_, _size_) \
	print_hex_dump(KERN_ERR, _name_, DUMP_PREFIX_OFFSET, \
		32, 1, (u8 *)(_buf_), (_size_), false)
#else
#define MHA_HEX_DUMP(_name_, _buf_, _size_)	do {} while (0)
#endif

/* Driver flags */
#define MHA_FLG_ENABLED			0 /* Shows when algorithm is enabled */
#define MHA_FLG_FW_NEW_GROUP_ID		1 /* New firmware use group ID */
#define MHA_FLG_BLOB_SIZE_VALID		2 /* Adapter blob size is valid */

#define MHA_FW_FOURCC \
	(u32)('M' << 24 | 'H' << 16 | 'F' << 8 | 'W')

#define MHA_FW_VERSION_1_1		0x0101
#define MHA_FW_VERSION_1_2		0x0102

#define MHA_BLOB_MAX_SIZE		(1024 * 1024)
#define MHA_BLOB_MAX_PAYLOAD_SIZE	1024

struct mha_blob_value {
	int8_t channel;
	uint8_t type;
	uint8_t len;
	uint32_t param;
	char value[0];
} __packed;

struct mha_fw_header {
	u32 fourcc;		/* Fourcc code of this FW: 'MHFW' */
	u32 version;		/* Version of the firmeware: 0x0101 => 1.0 */
	u32 size;		/* Total size of the FW */
} __packed;

struct mha_fw_header_v1_1 {
	struct mha_fw_header hdr;
	u32 params_cnt;			/* Count of parameters */
	u32 values_cnt;			/* Count of values */
	struct mha_parameter params[];
} __packed;

struct mha_fw_header_v1_2 {
	struct mha_fw_header hdr;
	u32 params_cnt;			/* Count of parameters */
	u32 values_cnt;			/* Count of values */
	u32 default_cnt;		/* Count of default groups */
	struct mha_parameter params[];
} __packed;

struct mha_param;

struct mha_param_val {
	unsigned int payload_size:16;	/* Payload size */
	unsigned int group_id:16;	/* Group_id */
	struct list_head parent;	/* Item of parent list */
	struct list_head item;		/* Item of list for runtime work */
	struct mha_value v;		/* Current selected value */
};

struct mha_param {
	struct list_head param;		/* Item from list of parameters */
	struct mha_parameter p;		/* Parameter description */
};

struct mha_metadata {
	struct list_head params;	/* List of struct mha_param */
	struct list_head values;	/* List of struct mha_param_val */
	struct mha_list_id *def_grp;	/* Default group Id */
	int max_payload_size;
};

struct mha_ctx {
	struct mutex lock;
	struct miscdevice mdev;
	struct mha_adapter_ops *adapter;
	struct mha_metadata md_algo;
	struct mha_metadata md_fw;
	struct mha_metadata md_fw_new;
	struct work_struct w_crash;
	struct work_struct w_enable;
	struct work_struct w_disable;
	unsigned long flags;
	size_t blob_size;
	int fw_new_group_id;
};

struct mha_blob_buffer {
	struct mha_value *value;		/* Auxilrary value buffer */
	int payload_size;			/* Max. avail. payload size */
	size_t size;				/* Total size of buffer */
	size_t pos_w;				/* End point of buffer */
	loff_t pos_r;				/* Start point of buffer */
	int count;				/* Cound of values */
	u8 *buf;				/* Pointer to the buffer */
};

struct mha_file_ctx {
	struct mha_ctx *mha;			/* Context of MHA core driver */
	struct mha_metadata *md;		/* Local used metadata */
	struct mha_blob_buffer blob;		/* Local blob buffer */
	int enum_type;
	int enum_param;
};

struct mha_data_type mha_data_types[] = {
	{ mkBool, 1, "mkBool" },
	{ mkChar8, 1, "mkChar8" },
	{ mkChar16, 2, "mkChar16" },
	{ mkString8, 1, "mkString8" },
	{ mkString16, 2, "mkString16" },
	{ mkInt16, 2, "mkInt16" },
	{ mkInt32, 4, "mkInt32" },
	{ mkInt64, 8, "mkInt64" },
	{ mkFloat16, 2, "mkFloat16" },
	{ mkFloat32, 4, "mkFloat32" },
	{ mkFloat64, 8, "mkFloat64" },
	{ mkInt16C, 2, "mkInt16C" },
	{ mkInt32C, 4, "mkInt32C" },
	{ mkInt64C, 8, "mkInt64C" },
	{ mkFloat16C, 2, "mkFloat16C" },
	{ mkFloat32C, 4, "mkFloat32C" },
	{ mkFloat64C, 8, "mkFloat64C" },
	{ mkVoid32Ptr, 4, "mkVoid32Ptr" },
	{ mkVoid64Ptr, 8, "mkVoid64Ptr" },
};

static struct mha_ctx mha_core;

static char *mha_strtok(const char *str, char delim)
{
	char *ch = strchr(str, delim);

	if (ch)
		*ch++ = 0;

	return ch;
}

static int mha_type_size(enum mha_data_types type)
{
	if (type >= 0 && type < ARRAY_SIZE(mha_data_types))
		return mha_data_types[type].size;

	return -EINVAL;
}

static struct mha_data_type *mha_type_by_id(int type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mha_data_types); i++)
		if (type == mha_data_types[i].type)
			return &mha_data_types[i];

	return NULL;
}

static struct mha_data_type *mha_type_by_name(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mha_data_types); i++)
		if (!strcmp(name, mha_data_types[i].name))
			return &mha_data_types[i];

	return NULL;
}

static struct mha_data_type *mha_type_by_index(int index)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mha_data_types); i++)
		if (!index--)
			return &mha_data_types[i];

	return NULL;
}

static int mha_adapter_check(struct mha_adapter_ops *adap)
{
	if (!adap)
		return -ENODEV;

	return adap->is_ready(adap) ? 0 : -EPERM;
}

int mha_payload_size(enum mha_data_types type, int count)
{
	struct mha_data_type *dt;

	dt = mha_type_by_id(type);

	return dt ? dt->size * count : -EINVAL;
}

static int mha_val_payload_size(struct mha_value *v)
{
	return v ? mha_payload_size(v->type, v->count) : -EINVAL;
}

static int mha_param_payload_size(struct mha_parameter *p)
{
	return p ? mha_payload_size(p->type, p->count) : -EINVAL;
}

static int mha_param_idx_cnt(struct mha_metadata *metadata)
{
	struct mha_param *p;
	int count = 0;

	list_for_each_entry(p, &metadata->params, param)
		count++;

	return count;
}

static struct mha_param *mha_param_by_id(struct mha_metadata *metadata,
	int id)
{
	struct mha_param *p;

	list_for_each_entry(p, &metadata->params, param)
		if (p->p.param == id)
			return p;

	return NULL;
}

static struct mha_param *mha_param_by_name(struct mha_metadata *metadata,
	const char *name)
{
	struct mha_param *p;

	list_for_each_entry(p, &metadata->params, param)
		if (!strcmp(name, p->p.name))
			return p;

	return NULL;
}

static struct mha_param *mha_param_by_index(struct mha_metadata *metadata,
	int index)
{
	struct mha_param *p;

	list_for_each_entry(p, &metadata->params, param)
		if (!index--)
			return p;

	return NULL;
}

static struct mha_param *mha_param_by_val(struct mha_metadata *metadata,
	struct mha_value *val)
{
	struct mha_param *p;

	p = mha_param_by_id(metadata, val->param);
	if (!p)
		return ERR_PTR(-ENOENT);

	if (p->p.type != val->type)
		return ERR_PTR(-EBADTYPE);

	if (p->p.count != val->count)
		return ERR_PTR(-EBADF);

	return p;
}

static struct mha_param_val *mha_param_val_add(struct mha_metadata *metadata,
	struct mha_param *p, int channel)
{
	struct mha_param_val *pv;
	struct mha_value *v;
	int payload_size = mha_type_size(p->p.type) * p->p.count;

	pv = kcalloc(1, sizeof(*pv) + payload_size, GFP_KERNEL);
	if (!pv)
		return NULL;

	INIT_LIST_HEAD(&pv->parent);
	INIT_LIST_HEAD(&pv->item);
	if (metadata)
		list_add_tail(&pv->parent, &metadata->values);

	v = &pv->v;

	pv->payload_size = payload_size;
	v->param = p->p.param;
	v->count = p->p.count;
	v->type = p->p.type;
	v->channel = channel;

	return pv;
}

static int mha_param_val_free(struct mha_param_val *pv)
{
	if (!pv)
		return -ENOENT;

	list_del_init(&pv->parent);
	list_del_init(&pv->item);
	kfree(pv);

	return 0;
}

static int mha_param_val_free_all(struct mha_metadata *metadata)
{
	struct mha_param_val *pv, *pvtmp;

	list_for_each_entry_safe(pv, pvtmp, &metadata->values, parent)
		mha_param_val_free(pv);

	return 0;
}

static void mha_param_free(struct mha_param *p)
{
	list_del_init(&p->param);
	kfree(p);
}

static int mha_param_free_all(struct mha_metadata *metadata)
{
	struct mha_param *p, *ptmp;

	list_for_each_entry_safe(p, ptmp, &metadata->params, param)
		mha_param_free(p);

	return 0;
}

static int mha_param_add(struct mha_metadata *metadata,
	struct mha_parameter *param)
{
	struct mha_param *p;
	struct list_head *item = NULL;

	list_for_each_entry(p, &metadata->params, param) {
		if (p->p.param < param->param)
			item = &p->param;
		else
			break;
	}

	if (param->ch_cnt > MHA_MAX_CHANNELS)
		return -EINVAL;

	if (!param->ch_cnt) {
		/* Set default support */
		param->ch_cnt = 1;
		param->ch_map[0] = -1;
	}

	p = kcalloc(1, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	INIT_LIST_HEAD(&p->param);
	list_add(&p->param, item ?: &metadata->params);

	p->p = *param;

	return 0;
}

static int mha_param_del_by_id(struct mha_metadata *metadata, int id)
{
	struct mha_param *p;
	struct mha_param_val *pv, *pvtmp;

	p = mha_param_by_id(metadata, id);
	if (!p)
		return -ENOENT;

	mha_param_free(p);

	list_for_each_entry_safe(pv, pvtmp, &metadata->values, parent)
		if (pv->v.param == id)
			mha_param_val_free(pv);

	return 0;
}

static int mha_list_id_free(struct mha_list_id **lid)
{
	if (!lid)
		return -EINVAL;

	kfree(*lid);
	*lid = NULL;

	return 0;
}

static struct mha_list_id *mha_list_id_create(int count, uint16_t *group_id)
{
	struct mha_list_id *lid;
	int i, size = sizeof(*lid) + sizeof(lid->id[0]) * count;

	if (count <= 0 || !group_id)
		return NULL;

	lid = kcalloc(1, size, GFP_KERNEL);
	if (!lid)
		return NULL;

	lid->count = count;
	for (i = 0; i < count; i++)
		lid->id[i] = group_id[i];

	return lid;
}

static int mha_metadata_free(struct mha_metadata *metadata)
{
	mha_param_free_all(metadata);
	mha_param_val_free_all(metadata);
	mha_list_id_free(&metadata->def_grp);

	return 0;
}

static int mha_metadata_init(struct mha_metadata *metadata)
{
	memset(metadata, 0x00, sizeof(*metadata));
	INIT_LIST_HEAD(&metadata->params);
	INIT_LIST_HEAD(&metadata->values);

	return 0;
}

static int mha_blob_clean(struct mha_blob_buffer *blob)
{
	blob->pos_w = 0;
	blob->pos_r = 0;
	blob->count = 0;

	return 0;
}

static int mha_blob_free(struct mha_blob_buffer *blob)
{
	if (blob->value)
		vfree(blob->value);

	if (blob->buf)
		vfree(blob->buf);

	blob->value = NULL;
	blob->buf = NULL;
	blob->size = 0;
	blob->count = 0;
	blob->pos_r = 0;
	blob->pos_w = 0;

	return 0;
}

static int mha_blob_alloc(struct mha_blob_buffer *blob,
	size_t blob_size, size_t max_payload_size)
{
	mha_blob_free(blob);

	if (blob_size > MHA_BLOB_MAX_SIZE)
		return -EINVAL;

	if (max_payload_size > MHA_BLOB_MAX_PAYLOAD_SIZE)
		return -EINVAL;

	blob->buf = vmalloc(blob_size);
	if (!blob->buf)
		return -ENOMEM;

	blob->size = blob_size;

	blob->value = vmalloc(sizeof(blob->value) + max_payload_size);
	if (!blob->value) {
		vfree(blob->buf);
		return -ENOMEM;
	}

	blob->payload_size = max_payload_size;

	mha_blob_clean(blob);

	return 0;
}

static int mha_blob_reinit(struct mha_blob_buffer *blob,
	void *buffer, size_t size)
{
	mha_blob_free(blob);
	mha_blob_clean(blob);

	blob->buf = buffer;
	blob->size = size;
	blob->count = 0;
	blob->pos_r = 0;
	blob->pos_w = size;

	return 0;
}

static int mha_blob_put(struct mha_blob_buffer *blob,
	struct mha_value *v, bool payload)
{
	struct mha_blob_value *bv = (void *)(blob->buf + blob->pos_w);
	size_t payload_size = mha_val_payload_size(v);
	size_t sz = payload_size + sizeof(*bv);

	if (sz + blob->pos_w > blob->size)
		return -ENOSPC;

	MHA_HEX_DUMP("mha: put: value: ", v, sizeof(*v) + payload_size);

	bv->channel = v->channel;
	bv->type = v->type;
	bv->len = v->count;
	bv->param = v->param;
	if (payload)
		memcpy(bv->value, v->data, payload_size);
	else
		memset(bv->value, 0x00, payload_size);
	blob->pos_w += sz;
	blob->count++;

	MHA_HEX_DUMP("mha: put: blob: ", bv, sizeof(*bv) + payload_size);

	return sz;
}

static int mha_blob_get(struct mha_blob_buffer *blob)
{
	struct mha_blob_value *bv = (void *)(blob->buf + blob->pos_r);
	struct mha_value *v = blob->value;
	int max_payload_size = blob->payload_size;
	int payload_size = mha_payload_size(bv->type, bv->len);
	int sz = payload_size + sizeof(*bv);

	if (sz + blob->pos_r > blob->pos_w)
		return -ENOBUFS;

	if (payload_size > max_payload_size)
		return -ENOBUFS;

	if (blob->count <= 0)
		return -ENOENT;

	MHA_HEX_DUMP("mha: get: blob:  ", bv, sizeof(*bv) + payload_size);

	memcpy(v->data, bv->value, payload_size);

	MHA_HEX_DUMP("mha: get: value: ", v, sizeof(*v) + payload_size);

	blob->pos_r += sz;
	blob->count--;

	return sz;
}

static int mha_blob_peek(struct mha_blob_buffer *blob)
{
	struct mha_blob_value *bv = (void *)(blob->buf + blob->pos_r);
	struct mha_value *v = blob->value;
	int payload_size, sz = sizeof(*bv);

	if (sz + blob->pos_r > blob->pos_w)
		return -ENOBUFS;

	if (blob->count <= 0)
		return -ENOENT;

	v->channel = bv->channel;
	v->type = bv->type;
	v->count = bv->len;
	v->param = bv->param;

	payload_size = mha_val_payload_size(v);
	if (payload_size < 0)
		return -EIO;

	sz += payload_size;
	if (sz + blob->pos_r > blob->pos_w)
		return -ENOBUFS;

	return sz;
}

static int mha_blob_count(struct mha_metadata *md,
	struct mha_blob_buffer *blob)
{
	struct mha_param *p;
	int rval, count = 0;

	blob->count++;
	while ((rval = mha_blob_peek(blob)) > 0) {
		p = mha_param_by_val(md, blob->value);
		if (!p)
			break;

		blob->pos_r += rval;
		blob->count++;
		count++;
	}
	blob->count--;

	return count;
}

static int mha_blob_xfer(struct mha_adapter_ops *adap,
	struct mha_blob_buffer *blob, bool snd_and_rcv)
{
	int size, count, rval;
	void *buf;

	rval = mha_adapter_check(adap);
	if (rval)
		return rval;

	if (!adap->is_active(adap))
		return -EPERM;

	buf = blob->buf + blob->pos_r;
	size = blob->pos_w - blob->pos_r;
	count = blob->count;

	MHA_HEX_DUMP("mha: xfer: snd: ", buf, size);

	rval = adap->blob_xfer(adap, buf, size, count, snd_and_rcv);

	if (snd_and_rcv)
		MHA_HEX_DUMP("mha: xfer: rcv: ", buf, size);

	return rval;
}

int mha_fw_reload_v1_1(struct device *dev, struct mha_ctx *mha,
	const struct mha_fw_header_v1_1 *fw, ssize_t size)
{
	struct mha_parameter *p;
	struct mha_metadata *md_fw = &mha->md_fw;
	int count, max_ps, rval = 0;
	char *values;

	mha_metadata_free(md_fw);

	max_ps = 0;
	count = fw->params_cnt;
	p = (struct mha_parameter *)&fw->params[0];
	while (count--) {
		max_ps = max_t(int, max_ps, mha_param_payload_size(p));
		mha_param_add(md_fw, p++);
	}
	md_fw->max_payload_size = max_ps;

	count = 0;
	values = (char *)p;
	size -= sizeof(*fw) + fw->params_cnt * sizeof(fw->params[0]);
	while (count < fw->values_cnt && size > sizeof(struct mha_value)) {
		struct mha_param *param;
		struct mha_value *v = (struct mha_value *)values;
		struct mha_param_val *pv_fw = NULL;

		param = mha_param_by_id(md_fw, v->param);
		if (IS_ERR_OR_NULL(param))
			continue;

		pv_fw = mha_param_val_add(md_fw, param, v->channel);
		if (IS_ERR_OR_NULL(pv_fw)) {
			rval = pv_fw ? PTR_ERR(pv_fw) : -EINVAL;
			break;
		}

		memcpy(&pv_fw->v.data[0], &v->data[0], pv_fw->payload_size);
		values += sizeof(*v) + pv_fw->payload_size;
		size -= sizeof(*v) + pv_fw->payload_size;
		count++;
	}

	dev_info(dev, "MHA FW v1.1: parameters %d ; values %d / %d ; %s\n",
		fw->params_cnt, count, fw->values_cnt,
		rval ? "FAILED" : "SUCCESS");

	return rval;
}

int mha_fw_reload_v1_2(struct device *dev, struct mha_ctx *mha,
	const struct mha_fw_header_v1_2 *fw, ssize_t size)
{
	struct mha_metadata *md_fw = &mha->md_fw;
	struct mha_parameter *p;
	int count, sz, max_ps, rval = 0;
	u16 group_id;
	const u8 *buff;

	mha_metadata_free(md_fw);

	count = fw->params_cnt;
	p = (struct mha_parameter *)&fw->params[0];
	max_ps = 0;
	while (count--) {
		max_ps = max_t(int, max_ps, mha_param_payload_size(p));
		mha_param_add(md_fw, p++);
	}
	md_fw->max_payload_size = max_ps;

	count = 0;
	buff = (const char *)p;
	size -= sizeof(*fw) + fw->params_cnt * sizeof(fw->params[0]);
	while (count < fw->values_cnt && size > sizeof(struct mha_value)) {
		struct mha_param *param;
		struct mha_value *v = (struct mha_value *)buff;
		struct mha_param_val *pv_fw = NULL;

		param = mha_param_by_id(md_fw, v->param);
		if (IS_ERR_OR_NULL(param))
			continue;

		pv_fw = mha_param_val_add(md_fw, param, v->channel);
		if (IS_ERR_OR_NULL(pv_fw)) {
			rval = pv_fw ? PTR_ERR(pv_fw) : -EINVAL;
			break;
		}

		memcpy(&pv_fw->v.data[0], &v->data[0], pv_fw->payload_size);
		sz = sizeof(*v) + pv_fw->payload_size;
		buff += sz;
		size -= sz;
		count++;

		sz = sizeof(group_id);
		memcpy(&group_id, buff, sz);
		pv_fw->group_id = group_id;
		buff += sz;
		size -= sz;
	}

	md_fw->def_grp = mha_list_id_create(fw->default_cnt, (void *)buff);
	sz = sizeof(md_fw->def_grp->id[0]) * fw->default_cnt;
	buff += sz;
	size -= sz;

	dev_info(dev,
		"MHA FW v1.2: parameters %d ; values %d / %d ; default %d\n",
		fw->params_cnt, count, fw->values_cnt, fw->default_cnt);

	return rval;
}

int mha_fw_put_by_group(struct mha_blob_buffer *blob_orig, int group_id,
	struct mha_metadata *md_fw)
{
	struct mha_blob_buffer blob = *blob_orig;
	struct mha_param_val *pv;
	int size, rval = 0;

	list_for_each_entry(pv, &md_fw->values, parent) {
		if (pv->group_id != group_id)
			continue;

		rval = mha_blob_put(&blob, &pv->v, true);
		if (rval < 0)
			break;
	}

	if (rval < 0)
		return rval;

	size = blob.pos_w - blob_orig->pos_w;
	*blob_orig = blob;

	return rval < 0 ? rval : size;
}

int mha_fw_populate(struct mha_ctx *mha)
{
	struct device *dev = mha->mdev.this_device;
	struct mha_metadata *md_src = &mha->md_fw;
	struct mha_metadata *md_dst = &mha->md_algo;
	struct mha_adapter_ops *adap = mha->adapter;
	struct mha_list_id *lid = md_src->def_grp;
	struct mha_blob_buffer blob = { .buf = NULL, .value = NULL };
	struct mha_param_val *v;
	struct mha_param *p;
	int cnt_p = 0, cnt_v = 0;
	int i, rval;

	rval = mha_adapter_check(adap);
	if (rval) {
		dev_err(dev, "MHA FW: Adapter is not ready!\n");
		return rval;
	}

	if (!adap->is_active(adap)) {
		dev_err(dev, "MHA FW: Adapter is not active!\n");
		rval = -EBUSY;
		return rval;
	}

	dev_info(dev, "MHA FW: Store FW parameters and default values!\n");

	mha->blob_size = adap->blob_size(adap);
	set_bit(MHA_FLG_BLOB_SIZE_VALID, &mha->flags);

	/* Remove all current parameters */
	mha_metadata_free(md_dst);

	/* Clone all FW parameters as current */
	list_for_each_entry(p, &md_src->params, param)
		mha_param_add(md_dst, &p->p);

	md_dst->max_payload_size = md_src->max_payload_size;

	/* Calculating count of parameters */
	list_for_each_entry(p, &md_src->params, param)
		cnt_p++;

	/* Calculating count of default values */
	list_for_each_entry(v, &md_src->values, parent)
		cnt_v++;

	/* Notify to start providing the firmware */
	if (adap->fw_start)
		adap->fw_start(adap, cnt_p, cnt_v);

	/* Send all parameters */
	if (adap->fw_param) {
		list_for_each_entry(p, &md_src->params, param)
			adap->fw_param(adap, &p->p);
	}

	/* Notify to finish providing the firmware */
	if (adap->fw_finish)
		adap->fw_finish(adap);

	if (!adap->fw_tuning || adap->fw_tuning(adap, true) < 0)
		return rval;

	memset(&blob, 0x00, sizeof(blob));
	mha_blob_alloc(&blob, mha->blob_size, md_src->max_payload_size);

	for (i = 0; blob.buf && i < (lid ? lid->count : 1); i++) {
		int gid = lid ? lid->id[i] : 0;

		mha_blob_clean(&blob);

		rval = mha_fw_put_by_group(&blob, gid, md_src);
		if (rval < 0) {
			dev_err(dev, "Failed to serialize Group Id %d\n", gid);
			continue;
		}

		rval = mha_blob_xfer(adap, &blob, false);
		if (rval < 0) {
			dev_err(dev, "Failed to transmit Group Id %d\n", gid);
			continue;
		}

		dev_info(dev, "FW: Successful store FW Group ID %d\n", gid);
	}

	if (adap->fw_tuning)
		adap->fw_tuning(adap, false);

	mha_blob_free(&blob);

	return rval;
}

static int mha_fw_prepare_v1_1(struct mha_file_ctx *fmha)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_fw_header_v1_1 *fw;
	struct mha_metadata *md_fw = &mha->md_fw_new;
	struct mha_param_val *pv;
	struct mha_parameter *pr;
	struct mha_param *p;
	int size, pcnt, vcnt = 0;
	char *buf;

	pcnt = mha_param_idx_cnt(md_fw);
	size = sizeof(*fw);
	size += pcnt * sizeof(struct mha_parameter);
	list_for_each_entry(pv, &md_fw->values, parent) {
		size += sizeof(pv->v) + pv->payload_size;
		vcnt++;
	}

	fw = vmalloc(size);
	if (!fw)
		return -ENOMEM;

	fw->hdr.fourcc = MHA_FW_FOURCC;
	fw->hdr.version = MHA_FW_VERSION_1_1;
	fw->hdr.size = size;
	fw->params_cnt = pcnt;
	fw->values_cnt = vcnt;

	pr = &fw->params[0];
	list_for_each_entry(p, &md_fw->params, param)
		memcpy(pr++, &p->p, sizeof(*pr));

	buf = (void *)pr;
	list_for_each_entry(pv, &md_fw->values, parent) {
		size = sizeof(pv->v) + pv->payload_size;
		memcpy(buf, &pv->v, size);
		buf += size;
	}

	mha_blob_reinit(&fmha->blob, fw, fw->hdr.size);

	return 0;
}

static int mha_fw_prepare_v1_2(struct mha_file_ctx *fmha)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_fw_header_v1_2 *fw;
	struct mha_metadata *md_fw = &mha->md_fw_new;
	struct mha_list_id *lid = md_fw->def_grp;
	struct mha_param_val *pv;
	struct mha_parameter *pr;
	struct mha_param *p;
	int size, size_default, pcnt, vcnt = 0;
	u16 group_id;
	u8 *buf;

	pcnt = mha_param_idx_cnt(md_fw);
	size = sizeof(*fw);
	size += pcnt * sizeof(struct mha_parameter);
	list_for_each_entry(pv, &md_fw->values, parent) {
		size += sizeof(pv->v) + pv->payload_size + sizeof(group_id);
		vcnt++;
	}

	size_default = lid ? sizeof(lid->id[0]) * lid->count : 0;
	size += size_default;

	fw = vmalloc(size);
	if (!fw)
		return -ENOMEM;

	fw->hdr.fourcc = MHA_FW_FOURCC;
	fw->hdr.version = MHA_FW_VERSION_1_2;
	fw->hdr.size = size;
	fw->params_cnt = pcnt;
	fw->values_cnt = vcnt;
	fw->default_cnt = lid->count;

	pr = &fw->params[0];
	list_for_each_entry(p, &md_fw->params, param)
		memcpy(pr++, &p->p, sizeof(*pr));

	buf = (void *)pr;
	list_for_each_entry(pv, &md_fw->values, parent) {
		size = sizeof(pv->v) + pv->payload_size;
		memcpy(buf, &pv->v, size);
		buf += size;

		group_id = pv->group_id;
		memcpy(buf, &group_id, sizeof(group_id));
		buf += sizeof(group_id);
	}

	if (lid) {
		memcpy(buf, &lid->id[0], size_default);
		buf += size_default;
	}

	mha_blob_reinit(&fmha->blob, fw, fw->hdr.size);

	return 0;
}

static int mha_ioctl_get_version(struct mha_file_ctx *fmha,
	uint __user *data)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_adapter_ops *adap = mha->adapter;
	uint version;
	u32 ver;
	int rval;

	rval = mha_adapter_check(adap);
	if (rval)
		return rval;

	rval = adap->get_version(adap, &ver);
	if (rval)
		return rval;

	version = ver;
	if (put_user(version, data))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_type_by_id(struct mha_file_ctx *fmha,
	struct mha_data_type __user *data)
{
	struct mha_data_type p1, *p2;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&p1, data, sizeof(*data)))
		return -EFAULT;

	p2 = mha_type_by_id(p1.type);
	if (!p2)
		return -ENOENT;

	if (copy_to_user(data, p2, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_type_by_name(struct mha_file_ctx *fmha,
	struct mha_data_type __user *data)
{
	struct mha_data_type p1, *p2;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&p1, data, sizeof(*data)))
		return -EFAULT;

	p2 = mha_type_by_name(p1.name);
	if (!p2)
		return -ENOENT;

	if (copy_to_user(data, p2, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_type_enum(struct mha_file_ctx *fmha,
	struct mha_data_type __user *data)
{
	struct mha_data_type *data_type;

	if (!data) {
		/* Reset enumeration */
		fmha->enum_type = 0;
		return 0;
	}

	data_type = mha_type_by_index(fmha->enum_type);
	if (!data_type)
		return -ENOENT;

	fmha->enum_type++;

	if (copy_to_user(data, data_type, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_param_cnt(struct mha_file_ctx *fmha,
	int __user *data)
{
	struct mha_metadata *md_curr = fmha->md;
	int rval;

	if (!data)
		return -EINVAL;

	rval = mha_param_idx_cnt(md_curr);
	if (rval < 0)
		return -ENOENT;

	if (put_user(rval, data))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_param_by_id(struct mha_file_ctx *fmha,
	struct mha_parameter __user *data)
{
	struct mha_metadata *md_curr = fmha->md;
	struct mha_param *param;
	struct mha_parameter p1;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&p1, data, sizeof(*data)))
		return -EFAULT;

	param = mha_param_by_id(md_curr, p1.param);
	if (!param)
		return -ENOENT;

	if (copy_to_user(data, &param->p, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_param_by_name(struct mha_file_ctx *fmha,
	struct mha_parameter __user *data)
{
	struct mha_metadata *md_curr = fmha->md;
	struct mha_param *param;
	struct mha_parameter p1;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&p1, data, sizeof(*data)))
		return -EFAULT;

	param = mha_param_by_name(md_curr, p1.name);
	if (!param)
		return -ENOENT;

	if (copy_to_user(data, &param->p, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_param_enum(struct mha_file_ctx *fmha,
	struct mha_parameter __user *data)
{
	struct mha_metadata *md_curr = fmha->md;
	struct mha_param *param;

	if (!data) {
		/* Reset enumeration */
		fmha->enum_param = 0;
		return 0;
	}

	param = mha_param_by_index(md_curr, fmha->enum_param);
	if (!param)
		return -ENOENT;

	fmha->enum_param++;

	if (copy_to_user(data, &param->p, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_param_add_metadata(struct mha_metadata *metadata,
	struct mha_parameter __user *data)
{
	struct mha_parameter param;
	struct mha_param *p;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&param, data, sizeof(*data)))
		return -EFAULT;

	p = mha_param_by_id(metadata, param.param);
	if (p)
		return -EBUSY;

	p = mha_param_by_name(metadata, param.name);
	if (p)
		return -EBUSY;

	return mha_param_add(metadata, &param);
}

static int mha_ioctl_param_add(struct mha_file_ctx *fmha,
	struct mha_parameter __user *data)
{
	struct mha_metadata *md_curr = fmha->md;

	return mha_ioctl_param_add_metadata(md_curr, data);
}

static int mha_ioctl_param_del_by_id(struct mha_file_ctx *fmha,
	int __user *data)
{
	struct mha_metadata *md_curr = fmha->md;
	int id;

	if (!data)
		return -EINVAL;

	if (get_user(id, data))
		return -EFAULT;

	return mha_param_del_by_id(md_curr, id);
}

static int mha_ioctl_param_del_all(struct mha_file_ctx *fmha)
{
	struct mha_metadata *md_curr = fmha->md;

	return mha_metadata_free(md_curr);
}

static struct mha_param *mha_ioctl_value(struct mha_file_ctx *fmha,
	struct mha_value __user *data, int access, bool payload)
{
	struct mha_metadata *md_curr = fmha->md;
	struct mha_value *v = fmha->blob.value;
	struct mha_param *p;

	if (!data)
		return ERR_PTR(-EINVAL);

	if (copy_from_user(v, data, sizeof(*data)))
		return ERR_PTR(-EFAULT);

	/* Verification parameter meta data */
	p = mha_param_by_val(md_curr, v);
	if (IS_ERR_OR_NULL(p))
		return p;

	if (access && !(access & p->p.flag))
		return ERR_PTR(-EPERM);

	if (payload) {
		int payload_size = mha_param_payload_size(&p->p);

		if (copy_from_user(v->data, data->data, payload_size))
			return ERR_PTR(-EFAULT);
	}

	return p;
}

static int mha_ioctl_value_set(struct mha_file_ctx *fmha,
	struct mha_value __user *data)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_blob_buffer tmpblob = fmha->blob;
	struct mha_adapter_ops *adap = mha->adapter;
	struct mha_param *p;
	int rval = 0;

	rval = mha_adapter_check(adap);
	if (rval)
		return rval;

	p = mha_ioctl_value(fmha, data, MHA_ACCESS_WRITE, true);
	if (IS_ERR_OR_NULL(p))
		return PTR_ERR(p);

	tmpblob.pos_r = tmpblob.pos_w;
	rval = mha_blob_put(&tmpblob, fmha->blob.value, true);
	if (rval < 0)
		return rval;

	rval = mha_blob_xfer(adap, &tmpblob, false);
	if (rval < 0)
		return rval;

	return 0;
}

static int mha_ioctl_value_get(struct mha_file_ctx *fmha,
	struct mha_value __user *data)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_blob_buffer tmpblob = fmha->blob;
	struct mha_adapter_ops *adap = mha->adapter;
	struct mha_param *p;
	int rval;

	rval = mha_adapter_check(adap);
	if (rval)
		return rval;

	p = mha_ioctl_value(fmha, data, MHA_ACCESS_READ, false);
	if (IS_ERR_OR_NULL(p))
		return PTR_ERR(p);

	tmpblob.pos_r = tmpblob.pos_w;
	rval = mha_blob_put(&tmpblob, fmha->blob.value, false);
	if (rval < 0)
		return rval;

	rval = mha_blob_xfer(adap, &tmpblob, true);
	if (rval < 0)
		return rval;

	rval = mha_blob_peek(&tmpblob);
	if (rval < 0)
		return rval;

	rval = mha_blob_get(&tmpblob);
	if (rval < 0)
		return rval;

	rval += sizeof(struct mha_value) - sizeof(struct mha_blob_value);
	if (copy_to_user(data, tmpblob.value, rval))
		return -EFAULT;

	return rval < 0 ? rval : 0;
}

static int mha_ioctl_group_clean(struct mha_file_ctx *fmha)
{
	struct mha_blob_buffer *blob = &fmha->blob;

	return mha_blob_clean(blob);
}

static int mha_ioctl_group_state(struct mha_file_ctx *fmha,
	struct mha_blob_state __user *data)
{
	struct mha_blob_buffer *blob = &fmha->blob;
	struct mha_blob_state bstate;

	bstate.size = blob->size;
	bstate.count = blob->count;
	bstate.pos_r = blob->pos_r;
	bstate.pos_w = blob->pos_w;

	if (copy_to_user(data, &bstate, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int mha_ioctl_group_put(struct mha_file_ctx *fmha,
	struct mha_value __user *data)
{
	struct mha_blob_buffer *blob = &fmha->blob;
	struct mha_param *p;
	int psize, rval;

	p = mha_ioctl_value(fmha, data, 0, true);
	if (IS_ERR_OR_NULL(p))
		return PTR_ERR(p);

	psize = mha_param_payload_size(&p->p);

	/* serialize parameter value */
	rval = mha_blob_put(blob, blob->value, true);
	if (rval < 0)
		return rval;

	return 0;
}

static int mha_ioctl_group_get(struct mha_file_ctx *fmha,
	struct mha_value __user *data)
{
	struct mha_blob_buffer *blob = &fmha->blob;
	struct mha_metadata *md_curr = fmha->md;
	struct mha_param *p;
	int rval;

	rval = mha_blob_peek(blob);
	if (rval < 0)
		return -ENOENT;

	p = mha_param_by_val(md_curr, blob->value);
	if (IS_ERR(p))
		return PTR_ERR(p);

	/* HACK: Simulate one more entry in the blob if has correct parameter */
	if (!blob->count && p)
		blob->count++;

	rval = mha_blob_get(blob);
	if (rval < 0)
		return rval;

	rval += sizeof(struct mha_value) - sizeof(struct mha_blob_value);
	if (copy_to_user(data, blob->value, rval))
		return -EFAULT;

	return rval;
}

static int mha_ioctl_group_store(struct mha_file_ctx *fmha)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_adapter_ops *adap = mha->adapter;
	struct mha_blob_buffer *blob = &fmha->blob;

	return mha_blob_xfer(adap, blob, false);
}

static int mha_ioctl_group_load(struct mha_file_ctx *fmha)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_adapter_ops *adap = mha->adapter;
	struct mha_blob_buffer *blob = &fmha->blob;

	return mha_blob_xfer(adap, blob, true);
}

static int mha_ioctl_fw_new(struct mha_file_ctx *fmha)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_metadata *md_fw = &mha->md_fw_new;

	/* Set FW metadata as current for this file descriptor */
	fmha->md = md_fw;

	clear_bit(MHA_FLG_FW_NEW_GROUP_ID, &mha->flags);
	mha->fw_new_group_id = 0;

	mha_blob_free(&fmha->blob);

	return mha_metadata_free(md_fw);
}

static int mha_ioctl_fw_param(struct mha_file_ctx *fmha,
	struct mha_parameter __user *data)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_metadata *md_fw = &mha->md_fw_new;

	return mha_ioctl_param_add_metadata(md_fw, data);
}

static int mha_ioctl_fw_value(struct mha_file_ctx *fmha,
	struct mha_value __user *data)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_metadata *md_fw = &mha->md_fw_new;
	struct mha_param *p;
	struct mha_param_val *pv;
	struct mha_value value;

	if (copy_from_user(&value, data, sizeof(*data)))
		return -EPERM;

	p = mha_param_by_val(md_fw, &value);
	if (IS_ERR_OR_NULL(p))
		return PTR_ERR(p);

	pv = mha_param_val_add(md_fw, p, value.channel);
	if (IS_ERR_OR_NULL(pv))
		return PTR_ERR(pv);

	if (copy_from_user(&pv->v, data, pv->payload_size + sizeof(pv->v)))
		return -EFAULT;

	if (test_bit(MHA_FLG_FW_NEW_GROUP_ID, &mha->flags))
		pv->group_id = mha->fw_new_group_id;

	return 0;
}

static int mha_ioctl_fw_prepare(struct mha_file_ctx *fmha)
{
	struct mha_ctx *mha = fmha->mha;
	int rval;

	if (test_bit(MHA_FLG_FW_NEW_GROUP_ID, &mha->flags))
		rval = mha_fw_prepare_v1_2(fmha);
	else
		rval = mha_fw_prepare_v1_1(fmha);

	return rval;
}

static int mha_ioctl_fw_group_id(struct mha_file_ctx *fmha,
	int __user *data)
{
	struct mha_ctx *mha = fmha->mha;
	int group_id;

	if (!data)
		return -EINVAL;

	if (get_user(group_id, data))
		return -EFAULT;

	set_bit(MHA_FLG_FW_NEW_GROUP_ID, &mha->flags);
	mha->fw_new_group_id = group_id;

	return 0;
}

static int mha_ioctl_fw_default(struct mha_file_ctx *fmha,
	struct mha_list_id __user *data)
{
	struct mha_metadata *md = fmha->md;
	struct mha_list_id fdef;
	struct mha_list_id *fw_def;
	int size, rval = 0;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&fdef, data, sizeof(fdef)))
		return -EFAULT;

	size = sizeof(fdef) + sizeof(fdef.id[0]) * fdef.count;
	fw_def = kmalloc(size, GFP_KERNEL);
	if (!fw_def)
		return -ENOMEM;

	if (copy_from_user(fw_def, data, size)) {
		kfree(fw_def);
		return -EFAULT;
	}

	mha_list_id_free(&md->def_grp);
	md->def_grp = mha_list_id_create(fw_def->count, fw_def->id);
	if (!md->def_grp)
		rval = -EINVAL;

	kfree(fw_def);

	return rval;
}

static int mha_ioctl_group_id_put(struct mha_file_ctx *fmha,
	int __user *data)
{
	struct mha_ctx *mha = fmha->mha;
	struct mha_blob_buffer *blob = &fmha->blob;
	struct mha_metadata *md_fw = &mha->md_fw;
	int group_id;

	if (!data)
		return -EINVAL;

	if (get_user(group_id, data))
		return -EFAULT;

	return mha_fw_put_by_group(blob, group_id, md_fw);
}

static long mha_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct mha_file_ctx *fmha = file->private_data;
	struct mha_ctx *mha = fmha->mha;
	struct device *dev = mha->mdev.this_device;
	void __user *data;
	int rval = -EINVAL;

	if (is_compat_task())
		data = compat_ptr(arg);
	else
		data = (void __user *)arg;

	mutex_lock(&mha->lock);

	switch (cmd) {
	case MHA_IOCTL_VERSION:
		rval = mha_ioctl_get_version(fmha, data);
		break;
	case MHA_IOCTL_TYPE_BY_ID:
		rval = mha_ioctl_type_by_id(fmha, data);
		break;
	case MHA_IOCTL_TYPE_BY_NAME:
		rval = mha_ioctl_type_by_name(fmha, data);
		break;
	case MHA_IOCTL_TYPE_ENUM:
		rval = mha_ioctl_type_enum(fmha, data);
		break;
	case MHA_IOCTL_PARAM_COUNT:
		rval = mha_ioctl_param_cnt(fmha, data);
		break;
	case MHA_IOCTL_PARAM_BY_ID:
		rval = mha_ioctl_param_by_id(fmha, data);
		break;
	case MHA_IOCTL_PARAM_BY_NAME:
		rval = mha_ioctl_param_by_name(fmha, data);
		break;
	case MHA_IOCTL_PARAM_ENUM:
		rval = mha_ioctl_param_enum(fmha, data);
		break;
	case MHA_IOCTL_PARAM_ADD:
		rval = mha_ioctl_param_add(fmha, data);
		break;
	case MHA_IOCTL_PARAM_DEL_BY_ID:
		rval = mha_ioctl_param_del_by_id(fmha, data);
		break;
	case MHA_IOCTL_PARAM_DEL_ALL:
		rval = mha_ioctl_param_del_all(fmha);
		break;
	case MHA_IOCTL_VALUE_SET:
		rval = mha_ioctl_value_set(fmha, data);
		break;
	case MHA_IOCTL_VALUE_GET:
		rval = mha_ioctl_value_get(fmha, data);
		break;
	case MHA_IOCTL_GROUP_CLEAN:
		rval = mha_ioctl_group_clean(fmha);
		break;
	case MHA_IOCTL_GROUP_STATE:
		rval = mha_ioctl_group_state(fmha, data);
		break;
	case MHA_IOCTL_GROUP_STORE:
		rval = mha_ioctl_group_store(fmha);
		break;
	case MHA_IOCTL_GROUP_LOAD:
		rval = mha_ioctl_group_load(fmha);
		break;
	case MHA_IOCTL_GROUP_PUT:
		rval = mha_ioctl_group_put(fmha, data);
		break;
	case MHA_IOCTL_GROUP_GET:
		rval = mha_ioctl_group_get(fmha, data);
		break;
	case MHA_IOCTL_FW_NEW:
		rval = mha_ioctl_fw_new(fmha);
		break;
	case MHA_IOCTL_FW_PARAM:
		rval = mha_ioctl_fw_param(fmha, data);
		break;
	case MHA_IOCTL_FW_VALUE:
		rval = mha_ioctl_fw_value(fmha, data);
		break;
	case MHA_IOCTL_FW_PREPARE:
		rval = mha_ioctl_fw_prepare(fmha);
		break;
	case MHA_IOCTL_FW_GROUP_ID:
		rval = mha_ioctl_fw_group_id(fmha, data);
		break;
	case MHA_IOCTL_FW_DEFAULT:
		rval = mha_ioctl_fw_default(fmha, data);
		break;
	case MHA_IOCTL_GROUP_ID_PUT:
		rval = mha_ioctl_group_id_put(fmha, data);
		break;
	default:
		dev_err(dev, "Invalid ioctl %08X\n", cmd);
		rval = -EINVAL;
	}

	mutex_unlock(&mha->lock);

	return rval;
}

static ssize_t mha_read(struct file *file, char __user *ubuf,
			size_t count, loff_t *ppos)
{
	struct mha_file_ctx *fmha = file->private_data;
	struct mha_blob_buffer *blob = &fmha->blob;
	void *buf = blob->buf;
	size_t size = blob->pos_w;
	ssize_t rval;

	if (!buf)
		return -EPERM;

	*ppos = blob->pos_r;
	rval = simple_read_from_buffer(ubuf, count, ppos, buf, size);
	blob->pos_r = *ppos;

	return rval;
}

static ssize_t mha_write(struct file *file, const char __user *ubuf,
			size_t count, loff_t *ppos)
{
	struct mha_file_ctx *fmha = file->private_data;
	struct mha_metadata *md_curr = fmha->md;
	struct mha_blob_buffer cblob, *blob = &fmha->blob;
	struct mha_ctx *mha = fmha->mha;
	void *buf = blob->buf;
	size_t size = blob->size;
	ssize_t rval;

	if (!buf)
		return -EPERM;

	*ppos = blob->pos_w;

	rval = simple_write_to_buffer(buf, size, ppos, ubuf, count);
	if (rval <= 0)
		return rval;

	/* Lock used to protect against a race with mha_fw_populate */
	mutex_lock(&mha->lock);

	/* Compute and update blob counter */
	cblob = *blob;
	cblob.pos_r = cblob.pos_w;
	cblob.pos_w = *ppos;
	blob->count += mha_blob_count(md_curr, &cblob);
	blob->pos_w = *ppos = cblob.pos_r;
	rval = blob->pos_w - blob->pos_r;

	mutex_unlock(&mha->lock);

	return rval;
}

static loff_t mha_lseek(struct file *file, loff_t offset, int whence)
{
	struct mha_file_ctx *fmha = file->private_data;
	struct mha_blob_buffer *blob = &fmha->blob;
	loff_t pos, max = blob->size - 1;

	switch (whence) {
	case SEEK_SET:
		pos = offset;
		break;
	case SEEK_CUR:
		pos = blob->pos_r + offset;
		break;
	case SEEK_END:
		pos = max - offset;
		break;
	default:
		return -EINVAL;
	}

	if (pos < 0 || pos > max)
		return -EINVAL;

	file->f_pos = pos;
	blob->pos_r = pos;

	return pos;
}

static int mha_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct mha_ctx *mha = dev_get_drvdata(mdev->this_device);
	struct mha_metadata *md_curr = &mha->md_algo;
	struct mha_file_ctx *fmha;
	size_t blob_size, max_payload_size;
	int rval;

	if (!test_bit(MHA_FLG_BLOB_SIZE_VALID, &mha->flags))
		return -EPERM;

	fmha = kcalloc(1, sizeof(*fmha), GFP_KERNEL);
	if (!fmha)
		return -ENOMEM;

	file->private_data = fmha;

	fmha->mha = mha;
	fmha->md = md_curr;

	blob_size = mha->blob_size;
	max_payload_size = md_curr->max_payload_size;
	rval = mha_blob_alloc(&fmha->blob, blob_size, max_payload_size);
	if (rval)
		goto no_blob;

	return 0;

no_blob:
	kfree(fmha);

	return rval;
}

static int mha_release(struct inode *inode, struct file *file)
{
	struct mha_file_ctx *fmha = file->private_data;

	mha_blob_free(&fmha->blob);
	kfree(fmha);

	return 0;
}

static const struct file_operations mha_fops = {
	.owner = THIS_MODULE,
	.open = mha_open,
	.release = mha_release,
	.unlocked_ioctl = mha_ioctl,
	.compat_ioctl = mha_ioctl,
	.read = mha_read,
	.write = mha_write,
	.llseek = mha_lseek,
};

int mha_register(struct mha_adapter_ops *adap)
{
	struct mha_ctx *mha = &mha_core;
	int rval = -EINVAL;

	mutex_lock(&mha->lock);

	if (mha->adapter) {
		pr_err("mha: MHA Adapter is already registered!\n");
		rval = -EBUSY;
		goto exit;
	}

	if (!adap->is_ready)
		pr_err("mha: Invalid MHA Adapter callback is_ready\n");
	else if (!adap->is_active)
		pr_err("mha: Invalid MHA Adapter callback is_active\n");
	else if (!adap->get_version)
		pr_err("mha: Invalid MHA Adapter callback get_version\n");
	else if (!adap->blob_size)
		pr_err("mha: Invalid MHA Adapter callback blob_size\n");
	else if (!adap->blob_xfer)
		pr_err("mha: Invalid MHA Adapter callback blob_xfer\n");
	else {
		clear_bit(MHA_FLG_ENABLED, &mha->flags);
		mha->adapter = adap;
		rval = 0;
	}

exit:
	mutex_unlock(&mha->lock);

	return rval;
}
EXPORT_SYMBOL(mha_register);

int mha_unregister(struct mha_adapter_ops *adap)
{
	struct mha_ctx *mha = &mha_core;
	int rval = 0;

	mutex_lock(&mha->lock);

	rval = mha->adapter == adap ? 0 : -EINVAL;
	if (rval)
		pr_err("mha: Can't unregister because MHA handler is incorrect\n");
	else
		mha->adapter = NULL;

	mutex_unlock(&mha->lock);

	return 0;
}
EXPORT_SYMBOL(mha_unregister);

int mha_firmware(struct device *dev, struct mha_adapter_ops *mhadap,
	const struct mha_fw_header *fw, size_t size)
{
	struct mha_ctx *mha = &mha_core;
	int rval;

	if (!fw || !size)
		return -EINVAL;

	if (fw->fourcc != MHA_FW_FOURCC) {
		u32 id = fw->fourcc;

		dev_err(dev, "FW: FW id is wrong: %c%c%c%c\n",
			(id >> 24 & 0xFF), (id >> 16 & 0xFF),
			(id >>  8 & 0xFF), (id >>  0 & 0xFF));
		return -EINVAL;
	}

	if (fw->size != size) {
		dev_err(dev, "FW: Insufficient firmware size: %d / %zd\n",
			fw->size, size);
		return -EINVAL;
	}

	mutex_lock(&mha->lock);

	switch (fw->version) {
	case MHA_FW_VERSION_1_1:
		rval = mha_fw_reload_v1_1(dev, mha,
				(struct mha_fw_header_v1_1 *)fw, size);
		break;
	case MHA_FW_VERSION_1_2:
		rval = mha_fw_reload_v1_2(dev, mha,
				(struct mha_fw_header_v1_2 *)fw, size);
		break;
	default:
		dev_err(dev, "FW: Unsupported version 0x%04X\n", fw->version);
		rval = -EINVAL;
		break;
	}

	mutex_unlock(&mha->lock);

	return rval;
}

void mha_notify_crash(struct work_struct *work)
{
	struct mha_ctx *mha = container_of(work, typeof(*mha), w_crash);

	mutex_lock(&mha->lock);
	mutex_unlock(&mha->lock);
}


void mha_notify_enable(struct work_struct *work)
{
	struct mha_ctx *mha = container_of(work, typeof(*mha), w_enable);

	mutex_lock(&mha->lock);

	mha_fw_populate(mha);

	mutex_unlock(&mha->lock);
}

void mha_notify_disable(struct work_struct *work)
{
	struct mha_ctx *mha = container_of(work, typeof(*mha), w_disable);

	mutex_lock(&mha->lock);

	mutex_unlock(&mha->lock);
}

int mha_notify(struct mha_adapter_ops *adap, enum mha_adapter_notify notify)
{
	struct mha_ctx *mha = &mha_core;

	if (!mha->adapter || mha->adapter != adap)
		return -EINVAL;

	switch (notify) {
	case MHA_NOTIFY_NOTHING:
		break;
	case MHA_NOTIFY_CRASH:
		schedule_work(&mha->w_crash);
		break;
	case MHA_NOTIFY_START:
		if (!test_and_set_bit(MHA_FLG_ENABLED, &mha->flags))
			schedule_work(&mha->w_enable);
		break;
	case MHA_NOTIFY_STOP:
		if (test_and_clear_bit(MHA_FLG_ENABLED, &mha->flags))
			schedule_work(&mha->w_disable);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(mha_notify);

static ssize_t mha_attr_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mha_ctx *mha = dev_get_drvdata(dev);
	struct mha_adapter_ops *adap = mha->adapter;
	uint32_t ver;
	int rval;

	rval = adap->get_version(adap, &ver);
	if (rval)
		return rval;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ver);
}
static DEVICE_ATTR(version, S_IRUSR, mha_attr_version_show, NULL);

static ssize_t mha_attr_param_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mha_ctx *mha = dev_get_drvdata(dev);
	struct mha_metadata *md_curr = &mha->md_algo;

	return scnprintf(buf, PAGE_SIZE, "%d\n", mha_param_idx_cnt(md_curr));
}
static DEVICE_ATTR(param_count, S_IRUSR, mha_attr_param_count_show, NULL);

static ssize_t mha_attr_param_add_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "{%s}:{%s}:{%s}:{%s}\n",
		"index", "type", "count", "name");
}

static ssize_t mha_attr_param_add_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mha_ctx *mha = dev_get_drvdata(dev);
	struct mha_metadata *md_curr = &mha->md_algo;
	struct mha_data_type *data_type;
	struct mha_parameter p;
	int tmp, err;
	char *ch;

	memset(&p, 0x00, sizeof(p));

	/* Parse parameter index */
	ch = mha_strtok(buf, ':');
	if (!ch || sscanf(buf, "%d", &tmp) != 1)
		return -EINVAL;
	p.param = tmp;

	/* Parse parameter data type */
	buf = ch;
	ch = mha_strtok(buf, ':');
	data_type = mha_type_by_name(buf);
	if (!ch || !data_type) {
		if (!ch || sscanf(buf, "%d", &tmp) != 1)
			return -EINVAL;

		data_type = mha_type_by_id(tmp);
		if (!data_type)
			return -EINVAL;
	}
	p.type = data_type->type;

	/* Parse parameter elemt count */
	buf = ch;
	ch = mha_strtok(buf, ':');
	if (!ch || sscanf(buf, "%d", &tmp) != 1)
		return -EINVAL;
	p.count = tmp;

	/* Parse parameter name */
	buf = ch;
	ch += strlen(ch) - 1;
	while (ch != buf && (*ch == '\n' || *ch == '\r'))
		*ch-- = 0;
	strlcpy(p.name, buf, sizeof(p.name) - 1);

	/* Check if it already exists */
	if (mha_param_by_id(md_curr, p.param))
		return -EBUSY;

	err = mha_param_add(md_curr, &p);

	return err < 0 ? err : count;
}
static DEVICE_ATTR(param_add, S_IRWXU,
	mha_attr_param_add_show, mha_attr_param_add_store);

static ssize_t mha_attr_param_del_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mha_ctx *mha = dev_get_drvdata(dev);
	struct mha_metadata *md_curr = &mha->md_algo;
	struct mha_param *param;
	int id, err;

	param = mha_param_by_name(md_curr, buf);
	if (!param) {
		if (sscanf(buf, "%d", &id) != 1)
			return -EINVAL;

		param = mha_param_by_id(md_curr, id);
		if (!param)
			return -ENOENT;
	}

	err = mha_param_del_by_id(md_curr, param->p.param);

	return err < 0 ? err : count;
}
static DEVICE_ATTR(param_delete, S_IWUSR, NULL, mha_attr_param_del_store);

static ssize_t mha_attr_param_del_all_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mha_ctx *mha = dev_get_drvdata(dev);
	struct mha_metadata *md_curr = &mha->md_algo;
	int err;

	err = mha_metadata_free(md_curr);

	return err < 0 ? err : count;
}
static DEVICE_ATTR(param_delete_all, S_IWUSR, NULL,
	mha_attr_param_del_all_store);

static ssize_t mha_attr_param_list_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mha_ctx *mha = dev_get_drvdata(dev);
	struct mha_metadata *md_curr = &mha->md_algo;
	struct mha_data_type *dt;
	struct mha_param *p;
	int size, len = 0;

	size = scnprintf(buf, PAGE_SIZE, "%-5s %-12s %-3s %s\n",
		"Index", "Type", "Cnt", "Name");
	len += size;
	buf += size;

	list_for_each_entry(p, &md_curr->params, param) {
		dt = mha_type_by_id(p->p.type);

		size = scnprintf(buf, PAGE_SIZE - len, "%5d %-12s %3d %s\n",
			p->p.param, dt->name, p->p.count, p->p.name);
		len += size;
		buf += size;
		if (len >= PAGE_SIZE)
			break;
	}

	return len;
}
static DEVICE_ATTR(param_list, S_IRUSR, mha_attr_param_list_show, NULL);

static ssize_t mha_attr_param_types_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i, size, len = 0;

	for (i = 0; i < ARRAY_SIZE(mha_data_types); i++) {
		struct mha_data_type *dt = &mha_data_types[i];

		size = scnprintf(buf, PAGE_SIZE - len, "%2d %s\n",
			dt->type, dt->name);
		len += size;
		buf += size;
		if (len >= PAGE_SIZE)
			break;
	}

	return len;
}
static DEVICE_ATTR(param_types, S_IRUSR, mha_attr_param_types_show, NULL);

#if 0
static ssize_t mha_attr_data_show(enum mha_data_types type, char *data,
	char *buf, int size)
{
	int i, esz, sz, len = 0;

	switch (type) {
	case mkBool:
		sz = scnprintf(buf, size - len, " %s",
			*data ? "True" : "False");
		break;

	case mkChar8:
	case mkString8:
		sz = scnprintf(buf, size - len, "%c",
			*data >= 32 && *data <= 127 ? *data : '.');
		break;

	case mkInt16:
		sz = scnprintf(buf, size - len, " %d",
			(int)*((uint16_t *)data));
		break;

	case mkInt32:
		sz = scnprintf(buf, size - len, " %d",
			(int)*((uint32_t *)data));
		break;

	case mkInt64:
		sz = scnprintf(buf, size - len, " %lld",
			(long long)*((uint64_t *)data));
		break;

/*
	case mkChar16:
	case mkFloat16:
	case mkFloat32:
	case mkFloat64:
	case mkInt16C:
	case mkInt32C:
	case mkInt64C:
	case mkFloat16C:
	case mkFloat32C:
	case mkFloat64C:
*/
	case mkVoid32Ptr:
		sz = scnprintf(buf, size - len, " 0x%08x",
			(int)*((uint32_t *)data));
		break;

	case mkVoid64Ptr:
		sz = scnprintf(buf, size - len, " 0x%016llx",
			(long long)*((uint64_t *)data));
		break;

	default:
		esz = mha_type_size(type);
		for (i = 0; i < esz; i++, data++) {
			sz = scnprintf(buf, size - len, " %02x", *data);
			len += sz;
			buf += sz;
			if (len >= size)
				break;
		}
		sz = 0;
		break;
	}

	len += sz;
	buf += sz;

	return len;
}
#endif

static const struct attribute *mha_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_param_count.attr,
	&dev_attr_param_add.attr,
	&dev_attr_param_delete.attr,
	&dev_attr_param_delete_all.attr,
	&dev_attr_param_list.attr,
	&dev_attr_param_types.attr,
	NULL,
};

static struct mha_ctx mha_core = {
	.mdev = {
		.name = "mha-ctrl",
		.minor = MISC_DYNAMIC_MINOR,
		.fops = &mha_fops,
	},
};

static int __init mha_core_init(void)
{
	struct mha_ctx *mha = &mha_core;
	struct device *dev;
	int rval;

	mutex_init(&mha->lock);
	mha_metadata_init(&mha->md_algo);
	mha_metadata_init(&mha->md_fw);
	mha_metadata_init(&mha->md_fw_new);
	INIT_WORK(&mha->w_crash, mha_notify_crash);
	INIT_WORK(&mha->w_enable, mha_notify_enable);
	INIT_WORK(&mha->w_disable, mha_notify_disable);

	rval = misc_register(&mha->mdev);
	if (rval) {
		pr_err("Can't create MH Acousting misc device!\n");
		return rval;
	}


	dev = mha->mdev.this_device;
	dev_set_drvdata(dev, mha);

	if (sysfs_create_files(&dev->kobj, mha_attrs))
		dev_err(dev, "Failed sysfs_create_files\n");

	return 0;
}

static void __exit mha_core_exit(void)
{
	struct mha_ctx *mha = &mha_core;

	mha_metadata_free(&mha->md_algo);
	mha_metadata_free(&mha->md_fw);
	mha_metadata_free(&mha->md_fw_new);

	misc_deregister(&mha->mdev);
}

module_init(mha_core_init);
module_exit(mha_core_exit);

MODULE_DESCRIPTION("MH Acoustics driver");
MODULE_AUTHOR("Facebook");
MODULE_LICENSE("GPL v2");
