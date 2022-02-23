/*
 * 1p render interface driver
 *
 * Copyright (c) 2020, Facebook Inc. All rights reserved.
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
#include <sound/fb-render-adapter-if.h>
#include <uapi/sound/fb-render.h>

#ifdef REN_DEBUG
#define REN_HEX_DUMP(_name_, _buf_, _size_) \
	print_hex_dump(KERN_ERR, _name_, DUMP_PREFIX_OFFSET, \
		32, 1, (u8 *)(_buf_), (_size_), false)
#else
#define REN_HEX_DUMP(_name_, _buf_, _size_)	do {} while (0)
#endif

/* Driver flags */
#define REN_FLG_ENABLED			0 /* Shows when algorithm is enabled */
#define REN_FLG_FW_NEW_GROUP_ID		1 /* New firmware use group ID */
#define REN_FLG_BLOB_SIZE_VALID		2 /* Adapter blob size is valid */

#define REN_FW_FOURCC \
	(u32)('M' << 24 | 'H' << 16 | 'F' << 8 | 'W')

#define REN_FW_VERSION_1_1		0x0101
#define REN_FW_VERSION_1_2		0x0102

#define REN_BLOB_MAX_SIZE (1024*16)
#define REN_BLOB_MAX_PAYLOAD_SIZE (1024*16)

struct ren_blob_value {
	int8_t channel;
	uint8_t type;
	uint32_t len;
	uint32_t param;
	char value[0];
} __packed;

struct ren_fw_header {
	u32 fourcc;		/* Fourcc code of this FW: 'REN FW' */
	u32 version;		/* Version of the firmeware */
	u32 size;		/* Total size of the FW */
} __packed;

struct ren_fw_header_v1_1 {
	struct ren_fw_header hdr;
	u32 params_cnt;			/* Count of parameters */
	u32 values_cnt;			/* Count of values */
	struct ren_parameter params[];
} __packed;

struct ren_fw_header_v1_2 {
	struct ren_fw_header hdr;
	u32 params_cnt;			/* Count of parameters */
	u32 values_cnt;			/* Count of values */
	u32 default_cnt;		/* Count of default groups */
	struct ren_parameter params[];
} __packed;

struct ren_param;

struct ren_param_val {
	unsigned int payload_size:16;	/* Payload size */
	unsigned int group_id:16;	/* Group_id */
	struct list_head parent;	/* Item of parent list */
	struct list_head item;		/* Item of list for runtime work */
	struct ren_value v;		/* Current selected value */
};

struct ren_param {
	struct list_head param;		/* Item from list of parameters */
	struct ren_parameter p;		/* Parameter description */
};

struct ren_metadata {
	struct list_head params;	/* List of struct ren_param */
	struct list_head values;	/* List of struct ren_param_val */
	struct ren_list_id *def_grp;	/* Default group Id */
	int max_payload_size;
};

struct ren_ctx {
	struct mutex lock;
	struct miscdevice mdev;
	struct ren_adapter_ops *adapter;
	struct ren_metadata md_algo;
	struct ren_metadata md_fw;
	struct ren_metadata md_fw_new;
	struct work_struct w_crash;
	struct work_struct w_enable;
	struct work_struct w_disable;
	unsigned long flags;
	size_t blob_size;
	int fw_new_group_id;
};

struct ren_blob_buffer {
	struct ren_value *value;		/* Auxilrary value buffer */
	int payload_size;			/* Max. avail. payload size */
	size_t size;				/* Total size of buffer */
	size_t pos_w;				/* End point of buffer */
	loff_t pos_r;				/* Start point of buffer */
	int count;				/* Cound of values */
	u8 *buf;				/* Pointer to the buffer */
};

struct ren_file_ctx {
	struct ren_ctx *ren;			/* Context of REN core driver */
	struct ren_metadata *md;		/* Local used metadata */
	struct ren_blob_buffer blob;		/* Local blob buffer */
	int enum_type;
	int enum_param;
};

static struct ren_data_type ren_data_types[] = {
	{ mkBool, 1, "Bool" },
	{ mkChar8, 1, "Char8" },
	{ mkChar16, 2, "Char16" },
	{ mkString8, 1, "String8" },
	{ mkString16, 2, "String16" },
	{ mkInt16, 2, "Int16" },
	{ mkInt32, 4, "Int32" },
	{ mkInt64, 8, "Int64" },
	{ mkFloat16, 2, "Float16" },
	{ mkFloat32, 4, "Float32" },
	{ mkFloat64, 8, "Float64" },
	{ mkInt16C, 2, "Int16C" },
	{ mkInt32C, 4, "Int32C" },
	{ mkInt64C, 8, "Int64C" },
	{ mkFloat16C, 2, "Float16C" },
	{ mkFloat32C, 4, "Float32C" },
	{ mkFloat64C, 8, "Float64C" },
	{ mkVoid32Ptr, 4, "Void32Ptr" },
	{ mkVoid64Ptr, 8, "Void64Ptr" },
};

static struct ren_ctx ren_core;

static char *ren_strtok(const char *str, char delim)
{
	char *ch = strchr(str, delim);

	if (ch)
		*ch++ = 0;

	return ch;
}

static int ren_type_size(enum ren_data_types type)
{
	if (type >= 0 && type < ARRAY_SIZE(ren_data_types))
		return ren_data_types[type].size;

	return -EINVAL;
}

static struct ren_data_type *ren_type_by_id(int type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ren_data_types); i++)
		if (type == ren_data_types[i].type)
			return &ren_data_types[i];

	return NULL;
}

static struct ren_data_type *ren_type_by_name(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ren_data_types); i++)
		if (!strcmp(name, ren_data_types[i].name))
			return &ren_data_types[i];

	return NULL;
}

static struct ren_data_type *ren_type_by_index(int index)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ren_data_types); i++)
		if (!index--)
			return &ren_data_types[i];

	return NULL;
}

static int ren_adapter_check(struct ren_adapter_ops *adap)
{
	if (!adap)
		return -ENODEV;

	return adap->is_ready(adap) ? 0 : -EPERM;
}

int render_payload_size(enum ren_data_types type, int count)
{
	struct ren_data_type *dt;

	dt = ren_type_by_id(type);

	return dt ? dt->size * count : -EINVAL;
}

static int ren_val_payload_size(struct ren_value *v)
{
	return v ? render_payload_size(v->type, v->count) : -EINVAL;
}

static int ren_param_payload_size(struct ren_parameter *p)
{
	return p ? render_payload_size(p->type, p->count) : -EINVAL;
}

static int ren_param_idx_cnt(struct ren_metadata *metadata)
{
	struct ren_param *p;
	int count = 0;

	list_for_each_entry(p, &metadata->params, param)
		count++;

	return count;
}

static struct ren_param *ren_param_by_id(struct ren_metadata *metadata,
	int id)
{
	struct ren_param *p;

	list_for_each_entry(p, &metadata->params, param)
		if (p->p.param == id)
			return p;

	return NULL;
}

static struct ren_param *ren_param_by_name(struct ren_metadata *metadata,
	const char *name)
{
	struct ren_param *p;

	list_for_each_entry(p, &metadata->params, param)
		if (!strcmp(name, p->p.name))
			return p;

	return NULL;
}

static struct ren_param *ren_param_by_index(struct ren_metadata *metadata,
	int index)
{
	struct ren_param *p;

	list_for_each_entry(p, &metadata->params, param)
		if (!index--)
			return p;

	return NULL;
}

static struct ren_param *ren_param_by_val(struct ren_metadata *metadata,
	struct ren_value *val)
{
	struct ren_param *p;

	p = ren_param_by_id(metadata, val->param);
	if (!p)
		return ERR_PTR(-ENOENT);

	if (p->p.type != val->type)
		return ERR_PTR(-EBADTYPE);

	if (p->p.count != val->count)
		return ERR_PTR(-EBADF);

	return p;
}

static struct ren_param_val *ren_param_val_add(
		struct ren_metadata *metadata,
		struct ren_param *p, int channel)
{
	struct ren_param_val *pv;
	struct ren_value *v;
	int payload_size = ren_type_size(p->p.type) * p->p.count;

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

static int ren_param_val_free(struct ren_param_val *pv)
{
	if (!pv)
		return -ENOENT;

	list_del_init(&pv->parent);
	list_del_init(&pv->item);
	kfree(pv);

	return 0;
}

static int ren_param_val_free_all(struct ren_metadata *metadata)
{
	struct ren_param_val *pv, *pvtmp;

	list_for_each_entry_safe(pv, pvtmp, &metadata->values, parent)
		ren_param_val_free(pv);

	return 0;
}

static void ren_param_free(struct ren_param *p)
{
	list_del_init(&p->param);
	kfree(p);
}

static int ren_param_free_all(struct ren_metadata *metadata)
{
	struct ren_param *p, *ptmp;

	list_for_each_entry_safe(p, ptmp, &metadata->params, param)
		ren_param_free(p);

	return 0;
}

static int ren_param_add(struct ren_metadata *metadata,
	struct ren_parameter *param)
{
	struct ren_param *p;
	struct list_head *item = NULL;

	list_for_each_entry(p, &metadata->params, param) {
		if (p->p.param < param->param)
			item = &p->param;
		else
			break;
	}

	if (param->ch_cnt > REN_MAX_CHANNELS)
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

static int ren_param_del_by_id(struct ren_metadata *metadata, int id)
{
	struct ren_param *p;
	struct ren_param_val *pv, *pvtmp;

	p = ren_param_by_id(metadata, id);
	if (!p)
		return -ENOENT;

	ren_param_free(p);

	list_for_each_entry_safe(pv, pvtmp, &metadata->values, parent)
		if (pv->v.param == id)
			ren_param_val_free(pv);

	return 0;
}

static int ren_list_id_free(struct ren_list_id **lid)
{
	if (!lid)
		return -EINVAL;

	kfree(*lid);
	*lid = NULL;

	return 0;
}

static struct ren_list_id *ren_list_id_create(int count,
					uint16_t *group_id)
{
	struct ren_list_id *lid;
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

static int ren_metadata_free(struct ren_metadata *metadata)
{
	ren_param_free_all(metadata);
	ren_param_val_free_all(metadata);
	ren_list_id_free(&metadata->def_grp);

	return 0;
}

static int ren_metadata_init(struct ren_metadata *metadata)
{
	memset(metadata, 0x00, sizeof(*metadata));
	INIT_LIST_HEAD(&metadata->params);
	INIT_LIST_HEAD(&metadata->values);

	return 0;
}

static int ren_blob_clean(struct ren_blob_buffer *blob)
{
	blob->pos_w = 0;
	blob->pos_r = 0;
	blob->count = 0;

	return 0;
}

static int ren_blob_free(struct ren_blob_buffer *blob)
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

static int ren_blob_alloc(struct ren_blob_buffer *blob,
	size_t blob_size, size_t max_payload_size)
{
	ren_blob_free(blob);

	if (blob_size > REN_BLOB_MAX_SIZE)
		return -EINVAL;

	if (max_payload_size > REN_BLOB_MAX_PAYLOAD_SIZE)
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

	ren_blob_clean(blob);

	return 0;
}

static int ren_blob_reinit(struct ren_blob_buffer *blob,
	void *buffer, size_t size)
{
	ren_blob_free(blob);
	ren_blob_clean(blob);

	blob->buf = buffer;
	blob->size = size;
	blob->count = 0;
	blob->pos_r = 0;
	blob->pos_w = 0;

	return 0;
}

static int ren_blob_put(struct ren_blob_buffer *blob,
	struct ren_value *v, bool payload)
{
	struct ren_blob_value *bv = (void *)(blob->buf + blob->pos_w);
	size_t payload_size = ren_val_payload_size(v);
	size_t sz = payload_size + sizeof(*bv);

	if (sz + blob->pos_w > blob->size)
		return -ENOSPC;

	REN_HEX_DUMP("ren: put: value: ", v, sizeof(*v) + payload_size);

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

	REN_HEX_DUMP("ren: put: blob: ", bv, sizeof(*bv) + payload_size);

	return sz;
}

static int ren_blob_get(struct ren_blob_buffer *blob)
{
	struct ren_blob_value *bv = (void *)(blob->buf + blob->pos_r);
	struct ren_value *v = blob->value;
	int max_payload_size = blob->payload_size;
	int payload_size = render_payload_size(bv->type, bv->len);
	int sz = payload_size + sizeof(*bv);

	if (sz + blob->pos_r > blob->pos_w)
		return -ENOBUFS;

	if (payload_size > max_payload_size)
		return -ENOBUFS;

	if (blob->count <= 0)
		return -ENOENT;

	REN_HEX_DUMP("ren: get: blob:  ", bv, sizeof(*bv) + payload_size);

	memcpy(v->data, bv->value, payload_size);

	REN_HEX_DUMP("ren: get: value: ", v, sizeof(*v) + payload_size);

	blob->pos_r += sz;
	blob->count--;

	return sz;
}

static int ren_blob_peek(struct ren_blob_buffer *blob)
{
	struct ren_blob_value *bv = (void *)(blob->buf + blob->pos_r);
	struct ren_value *v = blob->value;
	int payload_size, sz = sizeof(*bv);

	if (sz + blob->pos_r > blob->pos_w)
		return -ENOBUFS;

	if (blob->count <= 0)
		return -ENOENT;

	v->channel = bv->channel;
	v->type = bv->type;
	v->count = bv->len;
	v->param = bv->param;

	payload_size = ren_val_payload_size(v);
	if (payload_size < 0)
		return -EIO;

	sz += payload_size;
	if (sz + blob->pos_r > blob->pos_w)
		return -ENOBUFS;

	return sz;
}

static int ren_blob_count(struct ren_metadata *md,
	struct ren_blob_buffer *blob)
{
	struct ren_param *p;
	int rval, count = 0;

	blob->count++;
	while ((rval = ren_blob_peek(blob)) > 0) {
		p = ren_param_by_val(md, blob->value);
		if (!p)
			break;

		blob->pos_r += rval;
		blob->count++;
		count++;
	}
	blob->count--;

	return count;
}

static int ren_blob_xfer(struct ren_adapter_ops *adap,
	struct ren_blob_buffer *blob, bool snd_and_rcv)
{
	int size, count, rval;
	void *buf;

	rval = ren_adapter_check(adap);
	if (rval)
		return rval;

	if (!adap->is_active(adap))
		return -EPERM;

	buf = blob->buf + blob->pos_r;
	size = blob->pos_w - blob->pos_r;
	count = blob->count;

	rval = adap->blob_xfer(adap, buf, size, count, snd_and_rcv);

	return rval;
}

int render_fw_reload_v1_1(struct device *dev, struct ren_ctx *ren,
	const struct ren_fw_header_v1_1 *fw, ssize_t size)
{
	struct ren_parameter *p;
	struct ren_metadata *md_fw = &ren->md_fw;
	int count, max_ps, rval = 0;
	char *values;

	ren_metadata_free(md_fw);

	max_ps = 0;
	count = fw->params_cnt;
	p = (struct ren_parameter *)&fw->params[0];
	while (count--) {
		max_ps = max_t(int, max_ps, ren_param_payload_size(p));
		ren_param_add(md_fw, p++);
	}
	md_fw->max_payload_size = max_ps;

	count = 0;
	values = (char *)p;
	size -= sizeof(*fw) + fw->params_cnt * sizeof(fw->params[0]);
	while (count < fw->values_cnt && size > sizeof(struct ren_value)) {
		struct ren_param *param;
		struct ren_value *v = (struct ren_value *)values;
		struct ren_param_val *pv_fw = NULL;

		param = ren_param_by_id(md_fw, v->param);
		if (IS_ERR_OR_NULL(param))
			continue;

		pv_fw = ren_param_val_add(md_fw, param, v->channel);
		if (IS_ERR_OR_NULL(pv_fw)) {
			rval = pv_fw ? PTR_ERR(pv_fw) : -EINVAL;
			break;
		}

		memcpy(&pv_fw->v.data[0], &v->data[0], pv_fw->payload_size);
		values += sizeof(*v) + pv_fw->payload_size;
		size -= sizeof(*v) + pv_fw->payload_size;
		count++;
	}

	dev_info(dev, "REN FW v1.1: parameters %d ; values %d / %d ; %s\n",
		fw->params_cnt, count, fw->values_cnt,
		rval ? "FAILED" : "SUCCESS");

	return rval;
}

int render_fw_reload_v1_2(struct device *dev, struct ren_ctx *ren,
	const struct ren_fw_header_v1_2 *fw, ssize_t size)
{
	struct ren_metadata *md_fw = &ren->md_fw;
	struct ren_parameter *p;
	int count, sz, max_ps, rval = 0;
	u16 group_id;
	const u8 *buff;

	ren_metadata_free(md_fw);

	count = fw->params_cnt;
	p = (struct ren_parameter *)&fw->params[0];
	max_ps = 0;
	while (count--) {
		max_ps = max_t(int, max_ps, ren_param_payload_size(p));
		ren_param_add(md_fw, p++);
	}
	md_fw->max_payload_size = max_ps;

	count = 0;
	buff = (const char *)p;
	size -= sizeof(*fw) + fw->params_cnt * sizeof(fw->params[0]);
	while (count < fw->values_cnt && size > sizeof(struct ren_value)) {
		struct ren_param *param;
		struct ren_value *v = (struct ren_value *)buff;
		struct ren_param_val *pv_fw = NULL;

		param = ren_param_by_id(md_fw, v->param);
		if (IS_ERR_OR_NULL(param))
			continue;

		pv_fw = ren_param_val_add(md_fw, param, v->channel);
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

	md_fw->def_grp = ren_list_id_create(fw->default_cnt, (void *)buff);
	sz = sizeof(md_fw->def_grp->id[0]) * fw->default_cnt;
	buff += sz;
	size -= sz;

	dev_info(dev,
		"REN FW v1.2: parameters %d ; values %d / %d ; default %d\n",
		fw->params_cnt, count, fw->values_cnt, fw->default_cnt);

	return rval;
}

int render_fw_put_by_group(struct ren_blob_buffer *blob_orig, int group_id,
	struct ren_metadata *md_fw)
{
	struct ren_blob_buffer blob = *blob_orig;
	struct ren_param_val *pv;
	int size, rval = 0;

	list_for_each_entry(pv, &md_fw->values, parent) {
		if (pv->group_id != group_id)
			continue;

		rval = ren_blob_put(&blob, &pv->v, true);
		if (rval < 0)
			break;
	}

	if (rval < 0)
		return rval;

	size = blob.pos_w - blob_orig->pos_w;
	*blob_orig = blob;

	return rval < 0 ? rval : size;
}

int render_fw_populate(struct ren_ctx *ren)
{
	struct device *dev = ren->mdev.this_device;
	struct ren_metadata *md_src = &ren->md_fw;
	struct ren_metadata *md_dst = &ren->md_algo;
	struct ren_adapter_ops *adap = ren->adapter;
	struct ren_list_id *lid = md_src->def_grp;
	struct ren_blob_buffer blob = { .buf = NULL, .value = NULL };
	struct ren_param_val *v;
	struct ren_param *p;
	int cnt_p = 0, cnt_v = 0;
	int i, rval;

	rval = ren_adapter_check(adap);
	if (rval) {
		dev_err(dev, "REN FW: Adapter is not ready!\n");
		return rval;
	}

	if (!adap->is_active(adap)) {
		dev_err(dev, "REN FW: Adapter is not active!\n");
		rval = -EBUSY;
		return rval;
	}

	dev_info(dev, "REN FW: Store FW parameters and default values!\n");

	ren->blob_size = adap->blob_size(adap);
	set_bit(REN_FLG_BLOB_SIZE_VALID, &ren->flags);

	/* Remove all current parameters */
	ren_metadata_free(md_dst);

	/* Clone all FW parameters as current */
	list_for_each_entry(p, &md_src->params, param)
		ren_param_add(md_dst, &p->p);

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

	memset(&blob, 0x00, sizeof(blob));
	ren_blob_alloc(&blob, ren->blob_size, md_src->max_payload_size);

	for (i = 0; blob.buf && i < (lid ? lid->count : 1); i++) {
		int gid = lid ? lid->id[i] : 0;

		ren_blob_clean(&blob);

		rval = render_fw_put_by_group(&blob, gid, md_src);
		if (rval < 0) {
			dev_err(dev, "Failed to serialize Group Id %d\n", gid);
			continue;
		}

		rval = ren_blob_xfer(adap, &blob, false);
		if (rval < 0) {
			dev_err(dev, "Failed to transmit Group Id %d\n", gid);
			continue;
		}

		dev_info(dev, "FW: Successful store FW Group ID %d\n", gid);
	}

	ren_blob_free(&blob);

	return rval;
}

static int ren_fw_prepare_v1_1(struct ren_file_ctx *fren)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_fw_header_v1_1 *fw;
	struct ren_metadata *md_fw = &ren->md_fw_new;
	struct ren_param_val *pv;
	struct ren_parameter *pr;
	struct ren_param *p;
	int size, pcnt, vcnt = 0;
	char *buf;

	pcnt = ren_param_idx_cnt(md_fw);
	size = sizeof(*fw);
	size += pcnt * sizeof(struct ren_parameter);
	list_for_each_entry(pv, &md_fw->values, parent) {
		size += sizeof(pv->v) + pv->payload_size;
		vcnt++;
	}

	fw = vmalloc(size);
	if (!fw)
		return -ENOMEM;

	fw->hdr.fourcc = REN_FW_FOURCC;
	fw->hdr.version = REN_FW_VERSION_1_1;
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

	ren_blob_reinit(&fren->blob, fw, fw->hdr.size);

	return 0;
}

static int ren_fw_prepare_v1_2(struct ren_file_ctx *fren)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_fw_header_v1_2 *fw;
	struct ren_metadata *md_fw = &ren->md_fw_new;
	struct ren_list_id *lid = md_fw->def_grp;
	struct ren_param_val *pv;
	struct ren_parameter *pr;
	struct ren_param *p;
	int size, size_default, pcnt, vcnt = 0;
	u16 group_id;
	u8 *buf;

	pcnt = ren_param_idx_cnt(md_fw);
	size = sizeof(*fw);
	size += pcnt * sizeof(struct ren_parameter);
	list_for_each_entry(pv, &md_fw->values, parent) {
		size += sizeof(pv->v) + pv->payload_size + sizeof(group_id);
		vcnt++;
	}

	size_default = lid ? sizeof(lid->id[0]) * lid->count : 0;
	size += size_default;

	fw = vmalloc(size);
	if (!fw)
		return -ENOMEM;

	fw->hdr.fourcc = REN_FW_FOURCC;
	fw->hdr.version = REN_FW_VERSION_1_2;
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

	ren_blob_reinit(&fren->blob, fw, fw->hdr.size);

	return 0;
}

static int ren_ioctl_get_version(struct ren_file_ctx *fren,
	uint __user *data)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_adapter_ops *adap = ren->adapter;
	uint version;
	u32 ver;
	int rval;

	rval = ren_adapter_check(adap);
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

static int ren_ioctl_type_by_id(struct ren_file_ctx *fren,
	struct ren_data_type __user *data)
{
	struct ren_data_type p1, *p2;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&p1, data, sizeof(*data)))
		return -EFAULT;

	p2 = ren_type_by_id(p1.type);
	if (!p2)
		return -ENOENT;

	if (copy_to_user(data, p2, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int ren_ioctl_type_by_name(struct ren_file_ctx *fren,
	struct ren_data_type __user *data)
{
	struct ren_data_type p1, *p2;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&p1, data, sizeof(*data)))
		return -EFAULT;

	p2 = ren_type_by_name(p1.name);
	if (!p2)
		return -ENOENT;

	if (copy_to_user(data, p2, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int ren_ioctl_type_enum(struct ren_file_ctx *fren,
	struct ren_data_type __user *data)
{
	struct ren_data_type *data_type;

	if (!data) {
		/* Reset enumeration */
		fren->enum_type = 0;
		return 0;
	}

	data_type = ren_type_by_index(fren->enum_type);
	if (!data_type)
		return -ENOENT;

	fren->enum_type++;

	if (copy_to_user(data, data_type, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int ren_ioctl_param_cnt(struct ren_file_ctx *fren,
	int __user *data)
{
	struct ren_metadata *md_curr = fren->md;
	int rval;

	if (!data)
		return -EINVAL;

	rval = ren_param_idx_cnt(md_curr);
	if (rval < 0)
		return -ENOENT;

	if (put_user(rval, data))
		return -EFAULT;

	return 0;
}

static int ren_ioctl_param_by_id(struct ren_file_ctx *fren,
	struct ren_parameter __user *data)
{
	struct ren_metadata *md_curr = fren->md;
	struct ren_param *param;
	struct ren_parameter p1;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&p1, data, sizeof(*data)))
		return -EFAULT;

	param = ren_param_by_id(md_curr, p1.param);
	if (!param)
		return -ENOENT;

	if (copy_to_user(data, &param->p, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int ren_ioctl_param_by_name(struct ren_file_ctx *fren,
	struct ren_parameter __user *data)
{
	struct ren_metadata *md_curr = fren->md;
	struct ren_param *param;
	struct ren_parameter p1;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&p1, data, sizeof(*data)))
		return -EFAULT;

	param = ren_param_by_name(md_curr, p1.name);
	if (!param)
		return -ENOENT;

	if (copy_to_user(data, &param->p, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int ren_ioctl_param_enum(struct ren_file_ctx *fren,
	struct ren_parameter __user *data)
{
	struct ren_metadata *md_curr = fren->md;
	struct ren_param *param;

	if (!data) {
		/* Reset enumeration */
		fren->enum_param = 0;
		return 0;
	}

	param = ren_param_by_index(md_curr, fren->enum_param);
	if (!param)
		return -ENOENT;

	fren->enum_param++;

	if (copy_to_user(data, &param->p, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int ren_ioctl_param_add_metadata(struct ren_metadata *metadata,
	struct ren_parameter __user *data)
{
	struct ren_parameter param;
	struct ren_param *p;

	if (!data)
		return -EINVAL;

	if (copy_from_user(&param, data, sizeof(*data)))
		return -EFAULT;

	p = ren_param_by_id(metadata, param.param);
	if (p)
		return -EBUSY;

	p = ren_param_by_name(metadata, param.name);
	if (p)
		return -EBUSY;

	return ren_param_add(metadata, &param);
}

static int ren_ioctl_param_add(struct ren_file_ctx *fren,
	struct ren_parameter __user *data)
{
	struct ren_metadata *md_curr = fren->md;

	return ren_ioctl_param_add_metadata(md_curr, data);
}

static int ren_ioctl_param_del_by_id(struct ren_file_ctx *fren,
	int __user *data)
{
	struct ren_metadata *md_curr = fren->md;
	int id;

	if (!data)
		return -EINVAL;

	if (get_user(id, data))
		return -EFAULT;

	return ren_param_del_by_id(md_curr, id);
}

static int ren_ioctl_param_del_all(struct ren_file_ctx *fren)
{
	struct ren_metadata *md_curr = fren->md;

	return ren_metadata_free(md_curr);
}

static struct ren_param *ren_ioctl_value(struct ren_file_ctx *fren,
	struct ren_value __user *data, int access, bool payload)
{
	struct ren_metadata *md_curr = fren->md;
	struct ren_value *v = fren->blob.value;
	struct ren_param *p;

	if (!data)
		return ERR_PTR(-EINVAL);

	if (copy_from_user(v, data, sizeof(*data)))
		return ERR_PTR(-EFAULT);

	/* Verification parameter meta data */
	p = ren_param_by_val(md_curr, v);
	if (IS_ERR_OR_NULL(p))
		return p;

	if (access && !(access & p->p.flag))
		return ERR_PTR(-EPERM);

	if (payload) {
		int payload_size = ren_param_payload_size(&p->p);

		if (copy_from_user(v->data, data->data, payload_size))
			return ERR_PTR(-EFAULT);
	}

	return p;
}

static int ren_ioctl_value_set(struct ren_file_ctx *fren,
	struct ren_value __user *data)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_blob_buffer tmpblob = fren->blob;
	struct ren_adapter_ops *adap = ren->adapter;
	struct ren_param *p;
	int rval = 0;

	rval = ren_adapter_check(adap);
	if (rval)
		return rval;

	p = ren_ioctl_value(fren, data, REN_ACCESS_WRITE, true);
	if (IS_ERR_OR_NULL(p))
		return PTR_ERR(p);

	tmpblob.pos_r = tmpblob.pos_w;
	rval = ren_blob_put(&tmpblob, fren->blob.value, true);
	if (rval < 0)
		return rval;

	rval = ren_blob_xfer(adap, &tmpblob, false);
	if (rval < 0)
		return rval;

	return 0;
}

static int ren_ioctl_value_get(struct ren_file_ctx *fren,
	struct ren_value __user *data)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_blob_buffer tmpblob = fren->blob;
	struct ren_adapter_ops *adap = ren->adapter;
	struct ren_param *p;
	int rval;

	rval = ren_adapter_check(adap);
	if (rval)
		return rval;

	p = ren_ioctl_value(fren, data, REN_ACCESS_READ, false);
	if (IS_ERR_OR_NULL(p))
		return PTR_ERR(p);

	tmpblob.pos_r = tmpblob.pos_w;
	rval = ren_blob_put(&tmpblob, fren->blob.value, false);
	if (rval < 0)
		return rval;

	rval = ren_blob_xfer(adap, &tmpblob, true);
	if (rval < 0)
		return rval;

	rval = ren_blob_peek(&tmpblob);
	if (rval < 0)
		return rval;

	rval = ren_blob_get(&tmpblob);
	if (rval < 0)
		return rval;

	rval += sizeof(struct ren_value) - sizeof(struct ren_blob_value);
	if (copy_to_user(data, tmpblob.value, rval))
		return -EFAULT;

	return rval < 0 ? rval : 0;
}

static int ren_ioctl_group_clean(struct ren_file_ctx *fren)
{
	struct ren_blob_buffer *blob = &fren->blob;

	return ren_blob_clean(blob);
}

static int ren_ioctl_group_state(struct ren_file_ctx *fren,
	struct ren_blob_state __user *data)
{
	struct ren_blob_buffer *blob = &fren->blob;
	struct ren_blob_state bstate;

	bstate.size = blob->size;
	bstate.count = blob->count;
	bstate.pos_r = blob->pos_r;
	bstate.pos_w = blob->pos_w;

	if (copy_to_user(data, &bstate, sizeof(*data)))
		return -EFAULT;

	return 0;
}

static int ren_ioctl_group_put(struct ren_file_ctx *fren,
	struct ren_value __user *data)
{
	struct ren_blob_buffer *blob = &fren->blob;
	struct ren_param *p;
	int psize, rval;

	p = ren_ioctl_value(fren, data, 0, true);
	if (IS_ERR_OR_NULL(p))
		return PTR_ERR(p);

	psize = ren_param_payload_size(&p->p);

	/* serialize parameter value */
	rval = ren_blob_put(blob, blob->value, true);
	if (rval < 0)
		return rval;

	return 0;
}

static int ren_ioctl_group_get(struct ren_file_ctx *fren,
	struct ren_value __user *data)
{
	struct ren_blob_buffer *blob = &fren->blob;
	struct ren_metadata *md_curr = fren->md;
	struct ren_param *p;
	int rval;

	rval = ren_blob_peek(blob);
	if (rval < 0)
		return -ENOENT;

	p = ren_param_by_val(md_curr, blob->value);
	if (IS_ERR(p))
		return PTR_ERR(p);

	/* HACK: Simulate one more entry in the blob if has correct parameter */
	if (!blob->count && p)
		blob->count++;

	rval = ren_blob_get(blob);
	if (rval < 0)
		return rval;

	rval += sizeof(struct ren_value) - sizeof(struct ren_blob_value);
	if (copy_to_user(data, blob->value, rval))
		return -EFAULT;

	return rval;
}

static int ren_ioctl_group_store(struct ren_file_ctx *fren)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_adapter_ops *adap = ren->adapter;
	struct ren_blob_buffer *blob = &fren->blob;

	return ren_blob_xfer(adap, blob, false);
}

static int ren_ioctl_group_load(struct ren_file_ctx *fren)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_adapter_ops *adap = ren->adapter;
	struct ren_blob_buffer *blob = &fren->blob;

	return ren_blob_xfer(adap, blob, true);
}

static int ren_ioctl_fw_new(struct ren_file_ctx *fren)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_metadata *md_fw = &ren->md_fw_new;

	/* Set FW metadata as current for this file descriptor */
	fren->md = md_fw;

	clear_bit(REN_FLG_FW_NEW_GROUP_ID, &ren->flags);
	ren->fw_new_group_id = 0;

	ren_blob_free(&fren->blob);

	return ren_metadata_free(md_fw);
}

static int ren_ioctl_fw_param(struct ren_file_ctx *fren,
	struct ren_parameter __user *data)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_metadata *md_fw = &ren->md_fw_new;

	return ren_ioctl_param_add_metadata(md_fw, data);
}

static int ren_ioctl_fw_value(struct ren_file_ctx *fren,
	struct ren_value __user *data)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_metadata *md_fw = &ren->md_fw_new;
	struct ren_param *p;
	struct ren_param_val *pv;
	struct ren_value value;

	if (copy_from_user(&value, data, sizeof(*data)))
		return -EPERM;

	p = ren_param_by_val(md_fw, &value);
	if (IS_ERR_OR_NULL(p))
		return PTR_ERR(p);

	pv = ren_param_val_add(md_fw, p, value.channel);
	if (IS_ERR_OR_NULL(pv))
		return PTR_ERR(pv);

	if (copy_from_user(&pv->v, data, pv->payload_size + sizeof(pv->v)))
		return -EFAULT;

	if (test_bit(REN_FLG_FW_NEW_GROUP_ID, &ren->flags))
		pv->group_id = ren->fw_new_group_id;

	return 0;
}

static int ren_ioctl_fw_prepare(struct ren_file_ctx *fren)
{
	struct ren_ctx *ren = fren->ren;
	int rval;

	if (test_bit(REN_FLG_FW_NEW_GROUP_ID, &ren->flags))
		rval = ren_fw_prepare_v1_2(fren);
	else
		rval = ren_fw_prepare_v1_1(fren);

	return rval;
}

static int ren_ioctl_fw_group_id(struct ren_file_ctx *fren,
	int __user *data)
{
	struct ren_ctx *ren = fren->ren;
	int group_id;

	if (!data)
		return -EINVAL;

	if (get_user(group_id, data))
		return -EFAULT;

	set_bit(REN_FLG_FW_NEW_GROUP_ID, &ren->flags);
	ren->fw_new_group_id = group_id;

	return 0;
}

static int ren_ioctl_fw_default(struct ren_file_ctx *fren,
	struct ren_list_id __user *data)
{
	struct ren_metadata *md = fren->md;
	struct ren_list_id fdef;
	struct ren_list_id *fw_def;
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

	ren_list_id_free(&md->def_grp);
	md->def_grp = ren_list_id_create(fw_def->count, fw_def->id);
	if (!md->def_grp)
		rval = -EINVAL;

	kfree(fw_def);

	return rval;
}

static int ren_ioctl_group_id_put(struct ren_file_ctx *fren,
	int __user *data)
{
	struct ren_ctx *ren = fren->ren;
	struct ren_blob_buffer *blob = &fren->blob;
	struct ren_metadata *md_fw = &ren->md_fw;
	int group_id;

	if (!data)
		return -EINVAL;

	if (get_user(group_id, data))
		return -EFAULT;

	return render_fw_put_by_group(blob, group_id, md_fw);
}

static long ren_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct ren_file_ctx *fren = file->private_data;
	struct ren_ctx *ren = fren->ren;
	struct device *dev = ren->mdev.this_device;
	void __user *data;
	int rval = -EINVAL;

	if (is_compat_task())
		data = compat_ptr(arg);
	else
		data = (void __user *)arg;

	mutex_lock(&ren->lock);

	switch (cmd) {
	case REN_IOCTL_VERSION:
		rval = ren_ioctl_get_version(fren, data);
		break;
	case REN_IOCTL_TYPE_BY_ID:
		rval = ren_ioctl_type_by_id(fren, data);
		break;
	case REN_IOCTL_TYPE_BY_NAME:
		rval = ren_ioctl_type_by_name(fren, data);
		break;
	case REN_IOCTL_TYPE_ENUM:
		rval = ren_ioctl_type_enum(fren, data);
		break;
	case REN_IOCTL_PARAM_COUNT:
		rval = ren_ioctl_param_cnt(fren, data);
		break;
	case REN_IOCTL_PARAM_BY_ID:
		rval = ren_ioctl_param_by_id(fren, data);
		break;
	case REN_IOCTL_PARAM_BY_NAME:
		rval = ren_ioctl_param_by_name(fren, data);
		break;
	case REN_IOCTL_PARAM_ENUM:
		rval = ren_ioctl_param_enum(fren, data);
		break;
	case REN_IOCTL_PARAM_ADD:
		rval = ren_ioctl_param_add(fren, data);
		break;
	case REN_IOCTL_PARAM_DEL_BY_ID:
		rval = ren_ioctl_param_del_by_id(fren, data);
		break;
	case REN_IOCTL_PARAM_DEL_ALL:
		rval = ren_ioctl_param_del_all(fren);
		break;
	case REN_IOCTL_VALUE_SET:
		rval = ren_ioctl_value_set(fren, data);
		break;
	case REN_IOCTL_VALUE_GET:
		rval = ren_ioctl_value_get(fren, data);
		break;
	case REN_IOCTL_GROUP_CLEAN:
		rval = ren_ioctl_group_clean(fren);
		break;
	case REN_IOCTL_GROUP_STATE:
		rval = ren_ioctl_group_state(fren, data);
		break;
	case REN_IOCTL_GROUP_STORE:
		rval = ren_ioctl_group_store(fren);
		break;
	case REN_IOCTL_GROUP_LOAD:
		rval = ren_ioctl_group_load(fren);
		break;
	case REN_IOCTL_GROUP_PUT:
		rval = ren_ioctl_group_put(fren, data);
		break;
	case REN_IOCTL_GROUP_GET:
		rval = ren_ioctl_group_get(fren, data);
		break;
	case REN_IOCTL_FW_NEW:
		rval = ren_ioctl_fw_new(fren);
		break;
	case REN_IOCTL_FW_PARAM:
		rval = ren_ioctl_fw_param(fren, data);
		break;
	case REN_IOCTL_FW_VALUE:
		rval = ren_ioctl_fw_value(fren, data);
		break;
	case REN_IOCTL_FW_PREPARE:
		rval = ren_ioctl_fw_prepare(fren);
		break;
	case REN_IOCTL_FW_GROUP_ID:
		rval = ren_ioctl_fw_group_id(fren, data);
		break;
	case REN_IOCTL_FW_DEFAULT:
		rval = ren_ioctl_fw_default(fren, data);
		break;
	case REN_IOCTL_GROUP_ID_PUT:
		rval = ren_ioctl_group_id_put(fren, data);
		break;
	default:
		dev_err(dev, "Invalid ioctl %08X\n", cmd);
		rval = -EINVAL;
	}

	mutex_unlock(&ren->lock);

	return rval;
}

static ssize_t ren_read(struct file *file, char __user *ubuf,
			size_t count, loff_t *ppos)
{
	struct ren_file_ctx *fren = file->private_data;
	struct ren_blob_buffer *blob = &fren->blob;
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

static ssize_t ren_write(struct file *file, const char __user *ubuf,
			size_t count, loff_t *ppos)
{
	struct ren_file_ctx *fren = file->private_data;
	struct ren_metadata *md_curr = fren->md;
	struct ren_blob_buffer cblob, *blob = &fren->blob;
	void *buf = blob->buf;
	size_t size = blob->size;
	ssize_t rval;

	if (!buf)
		return -EPERM;

	*ppos = blob->pos_w;

	rval = simple_write_to_buffer(buf, size, ppos, ubuf, count);
	if (rval <= 0)
		return rval;

	/* Compute and update blob counter */
	cblob = *blob;
	cblob.pos_r = cblob.pos_w;
	cblob.pos_w = *ppos;
	blob->count += ren_blob_count(md_curr, &cblob);
	rval = cblob.pos_r - blob->pos_w;
	blob->pos_w = *ppos = cblob.pos_r;

	return rval;
}

static loff_t ren_lseek(struct file *file, loff_t offset, int whence)
{
	struct ren_file_ctx *fren = file->private_data;
	struct ren_blob_buffer *blob = &fren->blob;
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

static int ren_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct ren_ctx *ren = dev_get_drvdata(mdev->this_device);
	struct ren_metadata *md_curr = &ren->md_algo;
	struct ren_file_ctx *fren;
	size_t blob_size, max_payload_size;
	struct ren_adapter_ops *adap = ren->adapter;
	int rval;

	fren = kcalloc(1, sizeof(*fren), GFP_KERNEL);
	if (!fren)
		return -ENOMEM;

	file->private_data = fren;

	fren->ren = ren;
	fren->md = md_curr;

	// TODO:
	ren->blob_size = adap->blob_size(ren->adapter);

	blob_size = ren->blob_size;
	//TODO: not sure about size of payload.
	max_payload_size = blob_size;//md_curr->max_payload_size;
	rval = ren_blob_alloc(&fren->blob, blob_size, max_payload_size);
	if (rval)
		goto no_blob;

	return 0;

no_blob:
	kfree(fren);

	return rval;
}

static int ren_release(struct inode *inode, struct file *file)
{
	struct ren_file_ctx *fren = file->private_data;

	ren_blob_free(&fren->blob);
	kfree(fren);

	return 0;
}

static const struct file_operations ren_fops = {
	.owner = THIS_MODULE,
	.open = ren_open,
	.release = ren_release,
	.unlocked_ioctl = ren_ioctl,
	.compat_ioctl = ren_ioctl,
	.read = ren_read,
	.write = ren_write,
	.llseek = ren_lseek,
};

int render_register(struct ren_adapter_ops *adap)
{
	struct ren_ctx *ren = &ren_core;
	int rval = -EINVAL;

	mutex_lock(&ren->lock);

	if (ren->adapter) {
		pr_err("ren: REN Adapter is already registered!\n");
		rval = -EBUSY;
		goto exit;
	}

	if (!adap->is_ready)
		pr_err("ren: Invalid REN Adapter callback is_ready\n");
	else if (!adap->is_active)
		pr_err("ren: Invalid REN Adapter callback is_active\n");
	else if (!adap->get_version)
		pr_err("ren: Invalid REN Adapter callback get_version\n");
	else if (!adap->blob_size)
		pr_err("ren: Invalid REN Adapter callback blob_size\n");
	else if (!adap->blob_xfer)
		pr_err("ren: Invalid REN Adapter callback blob_xfer\n");
	else {
		clear_bit(REN_FLG_ENABLED, &ren->flags);
		ren->adapter = adap;
		rval = 0;
	}

exit:
	mutex_unlock(&ren->lock);

	return rval;
}
EXPORT_SYMBOL(render_register);

int render_unregister(struct ren_adapter_ops *adap)
{
	struct ren_ctx *ren = &ren_core;
	int rval = 0;

	mutex_lock(&ren->lock);

	rval = ren->adapter == adap ? 0 : -EINVAL;
	if (rval)
		pr_err("ren: Can't unregister because REN handler is incorrect\n");
	else
		ren->adapter = NULL;

	mutex_unlock(&ren->lock);

	return 0;
}
EXPORT_SYMBOL(render_unregister);

int render_firmware(struct device *dev, struct ren_adapter_ops *renadap,
	const struct ren_fw_header *fw, size_t size)
{
	struct ren_ctx *ren = &ren_core;
	int rval;

	if (!fw || !size)
		return -EINVAL;

	if (fw->fourcc != REN_FW_FOURCC) {
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

	mutex_lock(&ren->lock);

	switch (fw->version) {
	case REN_FW_VERSION_1_1:
		rval = render_fw_reload_v1_1(dev, ren,
				(struct ren_fw_header_v1_1 *)fw, size);
		break;
	case REN_FW_VERSION_1_2:
		rval = render_fw_reload_v1_2(dev, ren,
				(struct ren_fw_header_v1_2 *)fw, size);
		break;
	default:
		dev_err(dev, "FW: Unsupported version 0x%04X\n", fw->version);
		rval = -EINVAL;
		break;
	}

	mutex_unlock(&ren->lock);

	return rval;
}

void render_notify_crash(struct work_struct *work)
{
	struct ren_ctx *ren = container_of(work, typeof(*ren), w_crash);

	mutex_lock(&ren->lock);
	mutex_unlock(&ren->lock);
}


void render_notify_enable(struct work_struct *work)
{
	struct ren_ctx *ren = container_of(work, typeof(*ren), w_enable);

	mutex_lock(&ren->lock);

	render_fw_populate(ren);

	mutex_unlock(&ren->lock);
}

void render_notify_disable(struct work_struct *work)
{
	struct ren_ctx *ren = container_of(work, typeof(*ren), w_disable);

	mutex_lock(&ren->lock);

	mutex_unlock(&ren->lock);
}

int render_notify(struct ren_adapter_ops *adap, 
		enum ren_adapter_notify notify)
{
	struct ren_ctx *ren = &ren_core;

	if (!ren->adapter || ren->adapter != adap)
		return -EINVAL;

	switch (notify) {
	case REN_NOTIFY_NOTHING:
		break;
	case REN_NOTIFY_CRASH:
		schedule_work(&ren->w_crash);
		break;
	case REN_NOTIFY_START:
		if (!test_and_set_bit(REN_FLG_ENABLED, &ren->flags))
			schedule_work(&ren->w_enable);
		break;
	case REN_NOTIFY_STOP:
		if (test_and_clear_bit(REN_FLG_ENABLED, &ren->flags))
			schedule_work(&ren->w_disable);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(render_notify);

static ssize_t ren_attr_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ren_ctx *ren = dev_get_drvdata(dev);
	struct ren_adapter_ops *adap = ren->adapter;
	uint32_t ver;
	int rval;

	rval = ren_adapter_check(adap);
	if (rval)
		return rval;

	rval = adap->get_version(adap, &ver);
	if (rval)
		return rval;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ver);
}
static DEVICE_ATTR(version, S_IRUSR, ren_attr_version_show, NULL);

static ssize_t ren_attr_param_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ren_ctx *ren = dev_get_drvdata(dev);
	struct ren_metadata *md_curr = &ren->md_algo;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ren_param_idx_cnt(md_curr));
}
static DEVICE_ATTR(param_count, S_IRUSR, ren_attr_param_count_show, NULL);

static ssize_t ren_attr_param_add_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "{%s}:{%s}:{%s}:{%s}\n",
		"index", "type", "count", "name");
}

static ssize_t ren_attr_param_add_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ren_ctx *ren = dev_get_drvdata(dev);
	struct ren_metadata *md_curr = &ren->md_algo;
	struct ren_data_type *data_type;
	struct ren_parameter p;
	int tmp, err;
	char *ch;

	memset(&p, 0x00, sizeof(p));

	/* Parse parameter index */
	ch = ren_strtok(buf, ':');
	if (!ch || sscanf(buf, "%d", &tmp) != 1)
		return -EINVAL;
	p.param = tmp;

	/* Parse parameter data type */
	buf = ch;
	ch = ren_strtok(buf, ':');
	data_type = ren_type_by_name(buf);
	if (!ch || !data_type) {
		if (!ch || sscanf(buf, "%d", &tmp) != 1)
			return -EINVAL;

		data_type = ren_type_by_id(tmp);
		if (!data_type)
			return -EINVAL;
	}
	p.type = data_type->type;

	/* Parse parameter elemt count */
	buf = ch;
	ch = ren_strtok(buf, ':');
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
	if (ren_param_by_id(md_curr, p.param))
		return -EBUSY;

	err = ren_param_add(md_curr, &p);

	return err < 0 ? err : count;
}
static DEVICE_ATTR(param_add, S_IRWXU,
	ren_attr_param_add_show, ren_attr_param_add_store);

static ssize_t ren_attr_param_del_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ren_ctx *ren = dev_get_drvdata(dev);
	struct ren_metadata *md_curr = &ren->md_algo;
	struct ren_param *param;
	int id, err;

	param = ren_param_by_name(md_curr, buf);
	if (!param) {
		if (sscanf(buf, "%d", &id) != 1)
			return -EINVAL;

		param = ren_param_by_id(md_curr, id);
		if (!param)
			return -ENOENT;
	}

	err = ren_param_del_by_id(md_curr, param->p.param);

	return err < 0 ? err : count;
}
static DEVICE_ATTR(param_delete, S_IWUSR, NULL, ren_attr_param_del_store);

static ssize_t ren_attr_param_del_all_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ren_ctx *ren = dev_get_drvdata(dev);
	struct ren_metadata *md_curr = &ren->md_algo;
	int err;

	err = ren_metadata_free(md_curr);

	return err < 0 ? err : count;
}
static DEVICE_ATTR(param_delete_all, S_IWUSR, NULL,
	ren_attr_param_del_all_store);

static ssize_t ren_attr_param_list_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ren_ctx *ren = dev_get_drvdata(dev);
	struct ren_metadata *md_curr = &ren->md_algo;
	struct ren_data_type *dt;
	struct ren_param *p;
	int size, len = 0;

	size = scnprintf(buf, PAGE_SIZE, "%-5s %-12s %-3s %s\n",
		"Index", "Type", "Cnt", "Name");
	len += size;
	buf += size;

	list_for_each_entry(p, &md_curr->params, param) {
		dt = ren_type_by_id(p->p.type);

		size = scnprintf(buf, PAGE_SIZE - len, "%5d %-12s %3d %s\n",
			p->p.param, dt->name, p->p.count, p->p.name);
		len += size;
		buf += size;
		if (len >= PAGE_SIZE)
			break;
	}

	return len;
}
static DEVICE_ATTR(param_list, S_IRUSR, ren_attr_param_list_show, NULL);

static ssize_t ren_attr_param_types_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i, size, len = 0;

	for (i = 0; i < ARRAY_SIZE(ren_data_types); i++) {
		struct ren_data_type *dt = &ren_data_types[i];

		size = scnprintf(buf, PAGE_SIZE - len, "%2d %s\n",
			dt->type, dt->name);
		len += size;
		buf += size;
		if (len >= PAGE_SIZE)
			break;
	}

	return len;
}
static DEVICE_ATTR(param_types, S_IRUSR, ren_attr_param_types_show, NULL);

static const struct attribute *ren_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_param_count.attr,
	&dev_attr_param_add.attr,
	&dev_attr_param_delete.attr,
	&dev_attr_param_delete_all.attr,
	&dev_attr_param_list.attr,
	&dev_attr_param_types.attr,
	NULL,
};

static struct ren_ctx ren_core = {
	.mdev = {
		.name = "render-ctrl",
		.minor = MISC_DYNAMIC_MINOR,
		.fops = &ren_fops,
	},
};

static int __init ren_core_init(void)
{
	struct ren_ctx *ren = &ren_core;
	struct device *dev;
	int rval;

	mutex_init(&ren->lock);
	ren->adapter = NULL;
	ren_metadata_init(&ren->md_algo);
	ren_metadata_init(&ren->md_fw);
	ren_metadata_init(&ren->md_fw_new);

	INIT_WORK(&ren->w_crash, render_notify_crash);
	INIT_WORK(&ren->w_enable, render_notify_enable);
	INIT_WORK(&ren->w_disable, render_notify_disable);

	rval = misc_register(&ren->mdev);
	if (rval) {
		pr_err("Can't create REN Acousting misc device!\n");
		return rval;
	}


	dev = ren->mdev.this_device;
	dev_set_drvdata(dev, ren);

	if (sysfs_create_files(&dev->kobj, ren_attrs))
		dev_err(dev, "Failed sysfs_create_files\n");

	return 0;
}

static void __exit ren_core_exit(void)
{
	struct ren_ctx *ren = &ren_core;

	ren_metadata_free(&ren->md_algo);
	ren_metadata_free(&ren->md_fw);
	ren_metadata_free(&ren->md_fw_new);

	misc_deregister(&ren->mdev);
}

module_init(ren_core_init);
module_exit(ren_core_exit);

MODULE_DESCRIPTION("Render driver");
MODULE_AUTHOR("Facebook");
MODULE_LICENSE("GPL v2");
