/*
 * soc-core.c  --  ALSA SoC Audio Layer
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
 * Copyright (C) 2010 Slimlogic Ltd.
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * Author: Liam Girdwood <lrg@slimlogic.co.uk>
 *         with code, comments and ideas from :-
 *         Richard Purdie <richard@openedhand.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  TODO:
 *   o Add hw rules to enforce rates, etc.
 *   o More testing with other codecs/machines.
 *   o Add more codecs and platforms to ensure good API coverage.
 *   o Support TDM on PCM and I2S
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <sound/soc.h>

#ifdef CONFIG_DEBUG_FS

#define NAME2STR(str)			(str ?: "-")

struct snd_soc_card_debugfs {
	struct dentry *root;
	struct dentry *rtd;
	struct dentry *widgets;
	struct dentry *routes;
	struct dentry *dapm;
	struct dentry *dapm_durty;
	struct dentry *graph_all;
	struct dentry *graph_used;
	struct dentry *kcontrols;
	struct dentry *attr;
	char *buf;
	unsigned int flg_p_conn:1;		/* path connect */
	unsigned int flg_p_disc:1;		/* path disconnect */
	unsigned int flg_w_conn:1;		/* widgets connected */
	unsigned int flg_w_disc:1;		/* widgets disconnected */
	unsigned int flg_w_active:1;		/* widgets active */
	unsigned int flg_w_inactive:1;		/* widgets inactive */
	unsigned int flg_d_fe_play:1;		/* direction FE playback */
	unsigned int flg_d_fe_cap:1;		/* direction FE capture */
};

static char *dapm_type_str[] = {
	[snd_soc_dapm_input] = "input",
	[snd_soc_dapm_output] = "output",
	[snd_soc_dapm_mux] = "mux",
	[snd_soc_dapm_demux] = "demux",
	[snd_soc_dapm_mixer] = "mixer",
	[snd_soc_dapm_mixer_named_ctl] = "mixer_named_ctl",
	[snd_soc_dapm_pga] = "pga",
	[snd_soc_dapm_out_drv] = "out_drv",
	[snd_soc_dapm_adc] = "adc",
	[snd_soc_dapm_dac] = "dac",
	[snd_soc_dapm_micbias] = "micbias",
	[snd_soc_dapm_mic] = "mic",
	[snd_soc_dapm_hp] = "hp",
	[snd_soc_dapm_spk] = "spk",
	[snd_soc_dapm_line] = "line",
	[snd_soc_dapm_switch] = "switch",
	[snd_soc_dapm_vmid] = "switch",
	[snd_soc_dapm_pre] = "pre",
	[snd_soc_dapm_post] = "post",
	[snd_soc_dapm_supply] = "supply",
	[snd_soc_dapm_regulator_supply] = "regulator_supply",
	[snd_soc_dapm_clock_supply] = "clock_supply",
	[snd_soc_dapm_aif_in] = "aif_in",
	[snd_soc_dapm_aif_out] = "aif_out",
	[snd_soc_dapm_siggen] = "siggen",
	[snd_soc_dapm_dai_in] = "dai_in",
	[snd_soc_dapm_dai_out] = "dai_out",
	[snd_soc_dapm_dai_link] = "dai_link",
	[snd_soc_dapm_kcontrol] = "kcontrol",
};

static char *soc_dbg_vals_conn[] = {
	"none", "connected", "disconneted", "all", NULL };

static char *soc_dbg_vals_act[] = {
	"none", "active", "inactive", "all", NULL };

static char *soc_dbg_vals_dir[] = {
	"none", "playback", "capture", "both", NULL };

static struct list_head *soc_dbg_list_item(struct list_head *head, int index)
{
	struct list_head *item;

	list_for_each(item, head)
		if (!index--)
			return item;

	return NULL;
}

static int soc_dbg_fix_name(char *buf)
{
	char *ch;

	for (ch = buf; *ch; ch++)
		if (strnchr(" :;,.-", 6, *ch))
			*ch = '_';

	return 0;
}

static struct snd_soc_dai *soc_dbg_dai(struct snd_soc_dai *dai)
{
	static struct snd_soc_dai dai_null = {.name = "-", .id = -1, };

	return dai ?: &dai_null;
}

static char *soc_dbg_widget_id(int id)
{
	if (id < 0 || id >= ARRAY_SIZE(dapm_type_str))
		return "invalid";

	return dapm_type_str[id];
}

static void *soc_dbg_rtd_start(struct seq_file *seq, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;

	if (!*pos) {
		seq_printf(seq, "%s;%s;%s;%s;%s;%s;%s;%s;%s\n",
			"device", "name", "stream", "platform", "cpu",
			"codec", "type", "runtime", "direction");
	}

	return *pos < card->num_rtd ? &card->rtd[*pos] : NULL;
}

static void soc_dbg_rtd_stop(struct seq_file *seq, void *v)
{
}

static void *soc_dbg_rtd_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;

	return ++*pos < card->num_rtd ? &card->rtd[*pos] : NULL;
}

static int soc_dbg_rtd_show(struct seq_file *seq, void *v)
{
	struct snd_soc_pcm_runtime *rtd = v;
	struct snd_soc_dai_link *dl = rtd->dai_link;
	struct snd_soc_dai *src_dai, *dst_dai;
	const char *platform = "-";
	char *dir = "both";

	if (rtd->platform && rtd->platform->component.name)
		platform = rtd->platform->component.name;

	src_dai = soc_dbg_dai(rtd->cpu_dai);
	dst_dai = soc_dbg_dai(rtd->codec_dai ?: rtd->codec_dais[0]);

	if (dl && dl->dpcm_playback && !dl->dpcm_capture)
		dir = "playback";
	else if (dl && !dl->dpcm_playback && dl->dpcm_capture)
		dir = "capture";

	seq_printf(seq, "%d;%s;%s;%s;%s/%d;%s/%d;%s;%s;%s\n",
		rtd->pcm ? rtd->pcm->device : -1,
		NAME2STR(dl->name),
		NAME2STR(dl->stream_name),
		platform,
		src_dai->name, src_dai->id,
		dst_dai->name, dst_dai->id,
		dl->no_pcm ? "BE" : "FE",
		dl->dynamic ? "dynamic" : "static",
		dir);

	return 0;
}

static const struct seq_operations soc_dbg_rtd_seq_ops = {
	.start	= soc_dbg_rtd_start,
	.stop	= soc_dbg_rtd_stop,
	.next	= soc_dbg_rtd_next,
	.show	= soc_dbg_rtd_show,
};

static int soc_dbg_rtd_open(struct inode *inode, struct file *file)
{
	struct seq_file *seq;
	int ret;

	ret = seq_open(file, &soc_dbg_rtd_seq_ops);
	if (ret)
		return ret;

	seq = file->private_data;
	seq->private = inode->i_private;

	return ret;
}

static const struct file_operations soc_dbg_rtd_fops = {
	.open = soc_dbg_rtd_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static void *soc_dbg_widgets_start(struct seq_file *seq, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;
	struct list_head *lh;

	if (!*pos) {
		seq_printf(seq, "%s;%s;%s;%s;%s;%s;%s;%s;%s\n",
			"type", "name", "sname", "component",
			"active", "connected", "is_ep", "ep[0]", "ep[1]");
	}

	lh = soc_dbg_list_item(&card->widgets, *pos);

	return lh ? list_entry(lh, struct snd_soc_dapm_widget, list) : NULL;
}

static void soc_dbg_widgets_stop(struct seq_file *seq, void *v)
{
}

static void *soc_dbg_widgets_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;
	struct snd_soc_dapm_widget *w = v;

	*pos = *pos + 1;

	if (list_is_last(&w->list, &card->widgets))
		return NULL;

	return list_next_entry(w, list);
}

static int soc_dbg_widgets_show(struct seq_file *seq, void *v)
{
	struct snd_soc_dapm_widget *w = v;
	struct snd_soc_component *comp = w->dapm ? w->dapm->component : NULL;


	seq_printf(seq, "%s;%s;%s;%s;%s;%s;%s;%d;%d\n",
		soc_dbg_widget_id(w->id),
		NAME2STR(w->name),
		NAME2STR(w->sname),
		NAME2STR(comp ? comp->name : NULL),
		NAME2STR(w->active ? "active" : NULL),
		NAME2STR(w->connected ? "connected" : NULL),
		NAME2STR(w->is_ep ? "ep" : NULL),
		w->endpoints[0], w->endpoints[1]);

	return 0;
}

static const struct seq_operations soc_dbg_widgets_seq_ops = {
	.start	= soc_dbg_widgets_start,
	.stop	= soc_dbg_widgets_stop,
	.next	= soc_dbg_widgets_next,
	.show	= soc_dbg_widgets_show,
};

static int soc_dbg_widgets_open(struct inode *inode, struct file *file)
{
	struct seq_file *seq;
	int ret;

	ret = seq_open(file, &soc_dbg_widgets_seq_ops);
	if (ret)
		return ret;

	seq = file->private_data;
	seq->private = inode->i_private;

	return ret;
}

static const struct file_operations soc_dbg_widgets_fops = {
	.open = soc_dbg_widgets_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static void *soc_dbg_routes_start(struct seq_file *seq, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;
	struct list_head *lh;

	if (!*pos) {
		seq_printf(seq, "%s;%s;%s;%s;%s;%s;%s;%s\n",
			"src-id", "src-name", "src-sname",
			"sink-id", "sink-name", "sink-sname",
			"connect", "supply");
	}

	lh = soc_dbg_list_item(&card->paths, *pos);

	return lh ? list_entry(lh, struct snd_soc_dapm_path, list) : NULL;
}

static void soc_dbg_routes_stop(struct seq_file *seq, void *v)
{
}

static void *soc_dbg_routes_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;
	struct snd_soc_dapm_path *p = v;

	*pos = *pos + 1;

	if (list_is_last(&p->list, &card->paths))
		return NULL;

	return list_next_entry(p, list);
}

static int soc_dbg_routes_show(struct seq_file *seq, void *v)
{
	struct snd_soc_dapm_path *p = v;
	struct snd_soc_dapm_widget *ws = p->source;
	struct snd_soc_dapm_widget *wd = p->sink;

	seq_printf(seq, "%s;%s;%s;%s;%s;%s;%s;%s\n",
		soc_dbg_widget_id(ws ? ws->id : -1),
		ws && ws->name ? ws->name : "-",
		ws && ws->sname ? ws->sname : "-",
		soc_dbg_widget_id(wd ? wd->id : -1),
		wd && wd->name ? wd->name : "-",
		wd && wd->sname ? wd->sname : "-",
		p->connect ? "connect" : "-",
		p->is_supply ? "is_supply" : "-");

	return 0;
}

static const struct seq_operations soc_dbg_routes_seq_ops = {
	.start	= soc_dbg_routes_start,
	.stop	= soc_dbg_routes_stop,
	.next	= soc_dbg_routes_next,
	.show	= soc_dbg_routes_show,
};

static int soc_dbg_routes_open(struct inode *inode, struct file *file)
{
	struct seq_file *seq;
	int ret;

	ret = seq_open(file, &soc_dbg_routes_seq_ops);
	if (ret)
		return ret;

	seq = file->private_data;
	seq->private = inode->i_private;

	return ret;
}

static const struct file_operations soc_dbg_routes_fops = {
	.open = soc_dbg_routes_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static void *soc_dbg_dapm_start(struct seq_file *seq, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;
	struct list_head *lh;

	if (!*pos) {
		mutex_lock(&card->dapm_power_mutex);
		seq_printf(seq, "%s;%s;%s;%s;%s\n",
			"name", "id", "notifier", "event", "bias_level");
	}

	lh = soc_dbg_list_item(&card->dapm_list, *pos);
	if (!lh) {
		mutex_unlock(&card->dapm_power_mutex);
		return NULL;
	}

	return list_entry(lh, struct snd_soc_dapm_context, list);
}

static void soc_dbg_dapm_stop(struct seq_file *seq, void *v)
{
}

static void *soc_dbg_dapm_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;
	struct snd_soc_dapm_context *d = v;

	*pos = *pos + 1;

	if (list_is_last(&d->list, &card->dapm_list))
		return NULL;

	return list_next_entry(d, list);
}

static int soc_dbg_dapm_show(struct seq_file *seq, void *v)
{
	struct snd_soc_dapm_context *d = v;
	struct snd_soc_component *c;

	if (!d || !d->component)
		return 0;

	c = d->component;
	seq_printf(seq, "%s;%d;%pf,%pf,%pf\n",
		NAME2STR(c ? c->name : NULL), c ? c->id : -1,
		d->seq_notifier, d->stream_event, d->set_bias_level);

	return 0;
}

static const struct seq_operations soc_dbg_dapm_seq_ops = {
	.start	= soc_dbg_dapm_start,
	.stop	= soc_dbg_dapm_stop,
	.next	= soc_dbg_dapm_next,
	.show	= soc_dbg_dapm_show,
};

static int soc_dbg_dapm_open(struct inode *inode, struct file *file)
{
	struct seq_file *seq;
	int ret;

	ret = seq_open(file, &soc_dbg_dapm_seq_ops);
	if (ret)
		return ret;

	seq = file->private_data;
	seq->private = inode->i_private;

	return ret;
}

static const struct file_operations soc_dbg_dapm_fops = {
	.open = soc_dbg_dapm_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static void *soc_dbg_dapm_durty_start(struct seq_file *seq, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;
	struct list_head *lh;

	if (!*pos) {
		mutex_lock(&card->dapm_power_mutex);
		seq_printf(seq, "%s;%s;%s;%s;%s\n",
			"name", "id", "notifier", "event", "bias_level");
	}

	lh = soc_dbg_list_item(&card->dapm_dirty, *pos);
	if (!lh) {
		mutex_unlock(&card->dapm_power_mutex);
		return NULL;
	}

	return list_entry(lh, struct snd_soc_dapm_context, list);
}

static void soc_dbg_dapm_durty_stop(struct seq_file *seq, void *v)
{
}

static void *soc_dbg_dapm_durty_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct snd_soc_card *card = seq->private;
	struct snd_soc_dapm_context *d = v;

	*pos = *pos + 1;

	if (list_is_last(&d->list, &card->dapm_dirty))
		return NULL;

	return list_next_entry(d, list);
}

static const struct seq_operations soc_dbg_dapm_durty_seq_ops = {
	.start	= soc_dbg_dapm_durty_start,
	.stop	= soc_dbg_dapm_durty_stop,
	.next	= soc_dbg_dapm_durty_next,
	.show	= soc_dbg_dapm_show,
};

static int soc_dbg_dapm_durty_open(struct inode *inode, struct file *file)
{
	struct seq_file *seq;
	int ret;

	ret = seq_open(file, &soc_dbg_dapm_durty_seq_ops);
	if (ret)
		return ret;

	seq = file->private_data;
	seq->private = inode->i_private;

	return ret;
}

static const struct file_operations soc_dbg_dapm_durty_fops = {
	.open = soc_dbg_dapm_durty_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

enum soc_dbg_graph_types {
	SOC_DGRP_TYPE_NONE = 0,
	SOC_DGRP_TYPE_PCM_DEV,
	SOC_DGRP_TYPE_WIDGET,
	SOC_DGRP_TYPE_RTD,
	SOC_DGRP_TYPE_PATH,
	SOC_DGRP_TYPE_PARENT,
};

struct soc_dbg_graph_item {
	struct list_head item;
	struct list_head child;
	enum soc_dbg_graph_types type;
	unsigned int flg_connect:1;
	unsigned int flg_active:1;
	unsigned int flg_usage:1;
	unsigned int flg_populate:1;
	unsigned int flg_consumed:1;
	unsigned int flg_do_show:1;
	unsigned int usage;
	char *name;
	union {
		void *data;
		struct snd_soc_dpcm_runtime *drtd; /* SOC_DGRP_TYPE_PCM_DEV */
		struct snd_soc_pcm_runtime *rtd;   /* SOC_DGRP_TYPE_RTD     */
		struct snd_soc_dapm_widget *w;	   /* SOC_DGRP_TYPE_WIDGET  */
		struct snd_soc_dapm_path *p;	   /* SOC_DGRP_TYPE_PATH    */
		struct device *dev;		   /* SOC_DGRP_TYPE_PARENT  */
	};
	struct soc_dbg_graph_item *parent;
};

struct soc_dbg_graph_ctx {
	struct seq_file *seq;
	struct snd_soc_card *card;
	struct list_head list;
	struct list_head parents;
	struct soc_dbg_graph_item *parent_prev;
	struct soc_dbg_graph_item *parent_next;
	struct soc_dbg_graph_item *pcm_dev;
	struct soc_dbg_graph_item *show;
	unsigned int flg_header_start:1;
	unsigned int flg_header_start_next:1;
	unsigned int flg_used_only:1;
	unsigned int flg_populate_parent:1;
	char level[16];
	char level_new[16];
	int base_pos;
	int index;
	int count_items;
};

static struct soc_dbg_graph_item *soc_dbg_graph_add(
	struct soc_dbg_graph_ctx *gctx, enum soc_dbg_graph_types type,
	void *data);

static struct soc_dbg_graph_item *soc_dbg_graph_find(
	struct soc_dbg_graph_ctx *gctx, enum soc_dbg_graph_types type,
	void *data)
{
	struct soc_dbg_graph_item *item;

	if (!data)
		return NULL;

	list_for_each_entry(item, &gctx->list, item) {
		if (type && type != item->type)
			continue;

		if (data && data != item->data)
			continue;

		return item;
	}

	return NULL;
}

static struct soc_dbg_graph_item *soc_dbg_graph_find_path(
	struct soc_dbg_graph_ctx *gctx,
	struct snd_soc_dapm_widget *source, struct snd_soc_dapm_widget *sink)
{
	struct soc_dbg_graph_item *item;

	if (!source || !sink)
		return NULL;

	list_for_each_entry(item, &gctx->list, item) {
		if (item->type != SOC_DGRP_TYPE_PATH)
			continue;

		if (item->p->source == source && item->p->sink == sink)
			return item;
	}

	return NULL;
}

static void soc_dbg_graph_set_usage(struct soc_dbg_graph_item *item)
{
	if (!item)
		return;

	item->flg_usage = 1;
	item->usage++;
}

static void soc_dbg_graph_set_show(struct soc_dbg_graph_item *item)
{
	if (!item)
		return;

	item->flg_do_show = 1;
}

static int soc_dbg_graph_set_parent(struct soc_dbg_graph_ctx *gctx,
				    struct soc_dbg_graph_item *item)
{
	struct device *dev = NULL;
	bool is_real;

	if (item->parent)
		return 0;

	if (item->type == SOC_DGRP_TYPE_WIDGET)
		dev = item->w->dapm->dev;
	else if (item->type == SOC_DGRP_TYPE_PCM_DEV)
		dev = gctx->pcm_dev->dev;
	else if (item->type == SOC_DGRP_TYPE_PARENT)
		dev = item->dev->parent;

	if (!dev || dev == gctx->card->dev)
		return -ENODEV;

	item->parent = soc_dbg_graph_find(gctx, SOC_DGRP_TYPE_PARENT, dev);
	if (item->parent) {
		gctx->flg_populate_parent = 1;
		return 0;
	}

	is_real = dev->parent && dev->bus == &platform_bus_type;
	if (item->type == SOC_DGRP_TYPE_PARENT && !is_real)
		return -ENODEV;

	item->parent = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_PARENT, dev);
	gctx->flg_populate_parent = 1;

	return 0;
}

static struct soc_dbg_graph_item *soc_dbg_graph_add(
	struct soc_dbg_graph_ctx *gctx, enum soc_dbg_graph_types type,
	void *data)
{
	struct soc_dbg_graph_item *item;

	if (!data)
		return NULL;

	item = soc_dbg_graph_find(gctx, type, data);
	if (item)
		return item;

	item = kcalloc(1, sizeof(*item), GFP_KERNEL);
	if (!item)
		return NULL;

	INIT_LIST_HEAD(&item->item);
	INIT_LIST_HEAD(&item->child);
	item->type = type;
	item->data = data;

	list_add_tail(&item->item, &gctx->list);
	gctx->count_items++;

	soc_dbg_graph_set_parent(gctx, item);

	return item;
}

static void soc_dbg_graph_add_rtd(struct soc_dbg_graph_ctx *gctx)
{
	struct snd_soc_card *card = gctx->card;
	struct soc_dbg_graph_item *item;
	char buf[64];
	int i, j;

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_pcm_runtime *rtd = &card->rtd[i];
		struct soc_dbg_graph_item *irtd;
		struct snd_soc_dai *dai;

		irtd = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_RTD, rtd);
		soc_dbg_graph_set_show(irtd);

		if (!rtd->dai_link->no_pcm) {
			struct soc_dbg_graph_item *pcm, *parent;

			parent = soc_dbg_graph_add(gctx,
				SOC_DGRP_TYPE_PARENT, rtd->dev);
			snprintf(buf, PAGE_SIZE, "PCM Device #%d", i);
			parent->name = kstrdup(buf, GFP_KERNEL);
			parent->parent = gctx->pcm_dev;

			pcm = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_PCM_DEV,
				&rtd->dpcm[SNDRV_PCM_STREAM_PLAYBACK]);
			snprintf(buf, PAGE_SIZE, "pcmC%dD%dp", 0, i);
			pcm->name = kstrdup(buf, GFP_KERNEL);
			pcm->parent = parent;
			soc_dbg_graph_set_show(pcm);

			pcm = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_PCM_DEV,
				&rtd->dpcm[SNDRV_PCM_STREAM_CAPTURE]);
			snprintf(buf, PAGE_SIZE, "pcmC%dD%dc", 0, i);
			pcm->name = kstrdup(buf, GFP_KERNEL);
			pcm->parent = parent;
			soc_dbg_graph_set_show(pcm);
		}

		dai = rtd->cpu_dai;
		item = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET,
			dai ? dai->playback_widget : NULL);
		soc_dbg_graph_set_show(item);

		item = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET,
			dai ? dai->capture_widget : NULL);
		soc_dbg_graph_set_show(item);

		dai = rtd->codec_dai;
		item = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET,
			dai ? dai->playback_widget : NULL);
		soc_dbg_graph_set_show(item);

		item = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET,
			dai ? dai->capture_widget : NULL);
		soc_dbg_graph_set_show(item);

		for (j = 0; j < rtd->num_codecs; j++) {
			dai = rtd->codec_dais[j];

			item = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET,
				dai ? dai->playback_widget : NULL);
			soc_dbg_graph_set_show(item);

			item = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET,
				dai ? dai->capture_widget : NULL);
			soc_dbg_graph_set_show(item);
		}
	}
}

static void soc_dbg_graph_add_widgets(struct soc_dbg_graph_ctx *gctx)
{
	struct snd_soc_card *card = gctx->card;
	struct snd_soc_dapm_widget *w;

	list_for_each_entry(w, &card->widgets, list) {
		struct soc_dbg_graph_item *item;

		item = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET, w);
		item->flg_connect = !!w->connected;
		item->flg_active = !!w->active;
		/* item->usage = !!(item->flg_connect || item->flg_active); */
	}
}

static void soc_dbg_graph_add_path(struct soc_dbg_graph_ctx *gctx)
{
	struct snd_soc_card *card = gctx->card;
	struct snd_soc_dapm_path *p;

	list_for_each_entry(p, &card->paths, list) {
		struct soc_dbg_graph_item *item, *src, *dst;

		if (!p->source || !p->sink)
			continue;

		item = soc_dbg_graph_find_path(gctx, p->source, p->sink);
		if (item)
			continue;

		src = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET, p->source);
		dst = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_WIDGET, p->sink);

		item = soc_dbg_graph_add(gctx, SOC_DGRP_TYPE_PATH, p);
		item->flg_connect = !!p->connect;
		item->usage = !!p->connect;

		if (src->parent && src->parent == dst->parent)
			item->parent = src->parent;
	}
}

static int soc_dbg_graph_remove_items(struct list_head *list)
{
	struct soc_dbg_graph_item *item, *n;

	list_for_each_entry_safe(item, n, list, item) {
		kfree(item->name);
		list_del(&item->item);
		kfree(item);
	}

	return 0;
}

static void soc_dbg_graph_populate_item(struct soc_dbg_graph_ctx *gctx,
	struct soc_dbg_graph_item *item);

static void soc_dbg_graph_use_widget(struct soc_dbg_graph_ctx *gctx,
	struct snd_soc_dapm_widget *w)
{
	struct soc_dbg_graph_item *item;

	if (!w)
		return;

	item = soc_dbg_graph_find(gctx, SOC_DGRP_TYPE_WIDGET, w);
	soc_dbg_graph_set_usage(item);
	soc_dbg_graph_populate_item(gctx, item);
}

static void soc_dbg_graph_populate_path(struct soc_dbg_graph_ctx *gctx,
	struct soc_dbg_graph_item *item)
{
	struct snd_soc_dapm_path *p = item->p;

	if (!p->connect)
		return;

	soc_dbg_graph_set_usage(item);
	soc_dbg_graph_use_widget(gctx, p->source);
	soc_dbg_graph_use_widget(gctx, p->sink);
}

static void soc_dbg_graph_populate_rtd(struct soc_dbg_graph_ctx *gctx,
	struct soc_dbg_graph_item *item)
{
	struct snd_soc_pcm_runtime *rtd = item->rtd;
	struct snd_soc_dai *dai;
	int i;

	soc_dbg_graph_set_usage(item);

	dai = rtd->cpu_dai;
	soc_dbg_graph_use_widget(gctx, dai ? dai->playback_widget : NULL);
	soc_dbg_graph_use_widget(gctx, dai ? dai->capture_widget : NULL);

	dai = rtd->codec_dai;
	soc_dbg_graph_use_widget(gctx, dai ? dai->playback_widget : NULL);
	soc_dbg_graph_use_widget(gctx, dai ? dai->capture_widget : NULL);

	for (i = 0; i < rtd->num_codecs; i++) {
		dai = rtd->codec_dais[i];

		soc_dbg_graph_use_widget(gctx,
			dai ? dai->playback_widget : NULL);
		soc_dbg_graph_use_widget(gctx,
			dai ? dai->capture_widget : NULL);
	}
}

static void soc_dbg_graph_populate_pcm_dev(struct soc_dbg_graph_ctx *gctx,
	struct soc_dbg_graph_item *item)
{
	soc_dbg_graph_set_usage(item);
}

static void soc_dbg_graph_populate_widget(struct soc_dbg_graph_ctx *gctx,
	struct soc_dbg_graph_item *item)
{
}

static void soc_dbg_graph_populate_item(struct soc_dbg_graph_ctx *gctx,
	struct soc_dbg_graph_item *item)
{
	if (!item || item->flg_populate)
		return;

	switch (item->type) {
	case SOC_DGRP_TYPE_PCM_DEV:
		soc_dbg_graph_populate_pcm_dev(gctx, item);
		break;

	case SOC_DGRP_TYPE_PATH:
		soc_dbg_graph_populate_path(gctx, item);
		break;

	case SOC_DGRP_TYPE_RTD:
		soc_dbg_graph_populate_rtd(gctx, item);
		break;

	case SOC_DGRP_TYPE_WIDGET:
		soc_dbg_graph_populate_widget(gctx, item);
		break;
	default:
		break;
	}

	item->flg_populate = 1;
}

static int soc_dbg_graph_populate_usage(struct soc_dbg_graph_ctx *gctx)
{
	struct soc_dbg_graph_item *item;

	list_for_each_entry(item, &gctx->list, item)
		soc_dbg_graph_populate_item(gctx, item);

	return 0;
}

static int soc_dbg_graph_populate_parents(struct soc_dbg_graph_ctx *gctx)
{
	struct soc_dbg_graph_item *item, *n;

	do {
		gctx->flg_populate_parent = 0;
		list_for_each_entry_safe(item, n, &gctx->list, item) {
			soc_dbg_graph_set_parent(gctx, item);
		}
	} while (gctx->flg_populate_parent);

	return 0;
}

static int soc_dbg_graph_parent_names(struct soc_dbg_graph_ctx *gctx)
{
	struct soc_dbg_graph_item *item;
	const char *name;

	list_for_each_entry(item, &gctx->list, item) {
		int len;

		if (item->name)
			continue;

		if (item->type != SOC_DGRP_TYPE_PARENT)
			continue;

		name = dev_name(item->dev);
		len = strlen(name);
		while (strnchr(name, len, ':'))
			name = strnchr(name, len, ':') + 1;
		item->name = kstrdup(name, GFP_KERNEL);
	}

	return 0;
}

static void soc_dbg_graph_walk_child(struct soc_dbg_graph_ctx *gctx,
				     struct soc_dbg_graph_item *item,
				     struct list_head *list)
{
	struct soc_dbg_graph_item *i, *n;

	list_del_init(&item->item);

	if (!list_empty(&item->child)) {
		list_add_tail(&item->item, &gctx->parents);
		list_for_each_entry_safe(i, n, &item->child, item)
			soc_dbg_graph_walk_child(gctx, i, list);
	} else
		list_add_tail(&item->item, list);
}

static int soc_dbg_graph_sort(struct soc_dbg_graph_ctx *gctx)
{
	struct soc_dbg_graph_item *item, *n;
	struct list_head list = LIST_HEAD_INIT(list);

	list_for_each_entry_safe(item, n, &gctx->list, item) {
		if (item->parent) {
			list_del_init(&item->item);
			list_add_tail(&item->item, &item->parent->child);
		}
	}

	list_for_each_entry_safe(item, n, &gctx->list, item)
		if (!list_empty(&item->child))
			soc_dbg_graph_walk_child(gctx, item, &list);

	list_for_each_entry_safe(item, n, &gctx->list, item)
		soc_dbg_graph_walk_child(gctx, item, &list);

	list_add(&gctx->list, &list);
	list_del(&list);

	return 0;
}

static void soc_dbg_graph_start_header(struct soc_dbg_graph_ctx *gctx)
{
	struct seq_file *seq = gctx->seq;
	char *buf = gctx->card->debugfs->buf;

	if (gctx->flg_header_start)
		return;

	strlcpy(buf, gctx->card->name, PAGE_SIZE);
	soc_dbg_fix_name(buf);

	seq_printf(seq, "digraph %s {\n", buf);
	strlcpy(gctx->level, "\t", strlen(gctx->level));
	seq_printf(seq, "%srankdir=LR;\n", gctx->level);
	seq_printf(seq, "%s//splines=ortho;\n", gctx->level);
	seq_puts(seq, "\n");

	gctx->flg_header_start_next = 1;
}

static void soc_dbg_graph_start_footer(struct soc_dbg_graph_ctx *gctx)
{
	struct seq_file *seq = gctx->seq;

	seq_puts(seq, "}\n");

	gctx->flg_header_start_next = 0;
}

static void soc_dbg_graph_parent_header(struct soc_dbg_graph_ctx *gctx,
				       struct soc_dbg_graph_item *item)
{
	struct seq_file *seq = gctx->seq;

	if (!item)
		return;

	if (item->parent)
		soc_dbg_graph_parent_header(gctx, item->parent);

	seq_printf(seq, "%ssubgraph cluster_%p {\n", gctx->level, item);
	strlcat(gctx->level, "\t", sizeof(gctx->level));

	seq_printf(seq, "%snode [style=filled,color=white];\n", gctx->level);
	seq_printf(seq, "%sstyle=filled;\n", gctx->level);
	seq_printf(seq, "%scolor=lightgrey;\n", gctx->level);
	seq_printf(seq, "%slabel = \"%s\";\n", gctx->level, item->name);
	seq_puts(seq, "\n");
}

static void soc_dbg_graph_parent_footer(struct soc_dbg_graph_ctx *gctx,
					struct soc_dbg_graph_item *item,
					bool last)
{
	struct seq_file *seq = gctx->seq;

	if (!item)
		return;

	if (item->parent)
		soc_dbg_graph_parent_footer(gctx, item->parent, false);

	gctx->level[strlen(gctx->level) - 1] = 0;
	seq_printf(seq, "%s}\n%s", gctx->level, last ? "\n" : "");
}

static void *soc_dbg_graph_get_val(struct soc_dbg_graph_ctx *gctx)
{
	struct list_head *lh;

	lh = soc_dbg_list_item(&gctx->list, gctx->index);
	if (!lh)
		return NULL;

	gctx->show = list_entry(lh, struct soc_dbg_graph_item, item);

	return gctx->show;
}

static void *soc_dbg_graph_start(struct seq_file *seq, loff_t *pos)
{
	struct soc_dbg_graph_ctx *gctx = seq->private;

	return soc_dbg_graph_get_val(gctx);
}

static void soc_dbg_graph_stop(struct seq_file *seq, void *v)
{
}

static void *soc_dbg_graph_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct soc_dbg_graph_ctx *gctx = seq->private;

	if (gctx->show) {
		gctx->show->flg_consumed = 1;
		gctx->show = NULL;
	}

	strlcpy(gctx->level, gctx->level_new, sizeof(gctx->level));
	gctx->flg_header_start = gctx->flg_header_start_next;
	gctx->parent_prev = gctx->parent_next;

	gctx->index++;
	(*pos)++;

	return soc_dbg_graph_get_val(gctx);
}

static int soc_dbg_graph_show_pcm_dev(struct soc_dbg_graph_ctx *gctx,
				      struct soc_dbg_graph_item *item)
{
	struct seq_file *seq = gctx->seq;

	seq_printf(seq, "%s%p [%s,%s,%s=\"%s\"];\n",
		gctx->level, item, "shape=box", "color=lightblue",
		"label", item->name);

	return 0;
}

static int soc_dbg_graph_show_widgets(struct soc_dbg_graph_ctx *gctx,
				      struct soc_dbg_graph_item *item)
{
	struct seq_file *seq = gctx->seq;
	struct snd_soc_dapm_widget *w = item->w;
	char *buf = gctx->card->debugfs->buf;
	char *perip = NULL;
	char *color = NULL;
	char *shape = NULL;
	char *sides = NULL;
	char *style = NULL;

	if (gctx->flg_used_only && !item->usage)
		return 0;

	buf += snprintf(buf, PAGE_SIZE, "%p [label=\"t:%s\\nn:%s\\ns:%s",
		item, soc_dbg_widget_id(w->id),
		w->name ?: "", w->sname ?: "");
	if (w->is_ep)
		buf += snprintf(buf, PAGE_SIZE, "\\nep:%s,%s",
			w->is_ep & SND_SOC_DAPM_EP_SOURCE ? "source" : "-",
			w->is_ep & SND_SOC_DAPM_EP_SINK ? "sink" : "-");
	if (w->connected)
		buf += snprintf(buf, PAGE_SIZE, "\\nf:connected");
	if (w->active)
		buf += snprintf(buf, PAGE_SIZE, "\\nf:active");
	buf += snprintf(buf, PAGE_SIZE, "\"");

	switch (w->id) {
	case snd_soc_dapm_input:
		shape = "diamond";
		color = "green";
		perip = "2";
		break;
	case snd_soc_dapm_dai_in:
		color = "green";
		perip = "3";
		break;
	case snd_soc_dapm_aif_in:
		color = "green";
		perip = "2";
		break;
	case snd_soc_dapm_output:
		shape = "diamond";
		color = "red";
		perip = "2";
		break;
	case snd_soc_dapm_dai_out:
		color = "red";
		perip = "3";
		break;
	case snd_soc_dapm_aif_out:
		color = "red";
		perip = "2";
		break;
	case snd_soc_dapm_mux:
		color = "blue";
		shape = "polygon";
		sides = "6";
		break;
	case snd_soc_dapm_demux:
		color = "blue";
		shape = "polygon";
		sides = "8";
		break;
	case snd_soc_dapm_switch:
		color = "black";
		shape = "tab";
		break;
	case snd_soc_dapm_mixer:
		color = "black";
		shape = "polygon";
		sides = "9";
		break;
	default:
		color = "blue";
		shape = "circle";
		break;
	}

	style = w->connected ? "solid" : "dashed";

	if (color)
		buf += snprintf(buf, PAGE_SIZE, ",color=%s", color);

	if (shape)
		buf += snprintf(buf, PAGE_SIZE, ",shape=%s", shape);

	if (sides)
		buf += snprintf(buf, PAGE_SIZE, ",sides=%s", sides);

	if (style)
		buf += snprintf(buf, PAGE_SIZE, ",style=%s", style);

	if (perip)
		buf += snprintf(buf, PAGE_SIZE, ",peripheries=%s", perip);

	buf += snprintf(buf, PAGE_SIZE, "];");

	seq_printf(seq, "%s%s\n", gctx->level, gctx->card->debugfs->buf);

	return 0;
}

static int soc_dbg_graph_show_path(struct soc_dbg_graph_ctx *gctx,
				   struct soc_dbg_graph_item *item)
{
	struct seq_file *seq = gctx->seq;
	struct snd_soc_dapm_path *p = item->p;
	struct soc_dbg_graph_item *source, *sink;
	char *style, *color;

	if (gctx->flg_used_only && !item->usage)
		return 0;

	source = soc_dbg_graph_find(gctx, SOC_DGRP_TYPE_WIDGET, p->source);
	sink = soc_dbg_graph_find(gctx, SOC_DGRP_TYPE_WIDGET, p->sink);

	if (!source || !sink)
		return 0;

	style = item->usage ? "solid" : "dashed";
	color = "brown";

	seq_printf(seq, "%s%p -> %p [style=%s,color=%s];\n",
		gctx->level, source, sink, style, color);

	return 0;
}

static void soc_dbg_graph_links_add(struct soc_dbg_graph_ctx *gctx,
	struct snd_soc_dapm_widget *widget,
	struct soc_dbg_graph_item *links[], int *num)
{
	struct soc_dbg_graph_item *item;

	if (!widget || *num >= 16)
		return;

	item = soc_dbg_graph_find(gctx, SOC_DGRP_TYPE_WIDGET, (widget));
	if (!item)
		return;

	links[*num] = item;
	(*num)++;
}

static void soc_dbg_graph_links_dup(struct soc_dbg_graph_item *links[],
	int num)
{
	int i, j;

	for (i = 0; i < num; i++)
		for (j = 1; j < num; j++)
			if (i != j && links[i] == links[j])
				links[j] = NULL;
}

static int soc_dbg_graph_show_rtd(struct soc_dbg_graph_ctx *gctx,
				  struct soc_dbg_graph_item *item)
{
	struct seq_file *seq = gctx->seq;
	struct soc_dbg_graph_item *witem, *links[16];
	struct snd_soc_pcm_runtime *rtd = item->rtd;
	struct snd_soc_dai_link *dl = rtd->dai_link;
	struct snd_soc_dai *dai;
	char *style, *color, *ops = gctx->card->debugfs->buf;
	int i, num_links;

	if (gctx->flg_used_only && !item->usage)
		return 0;

	style = "bold";
	color = "black";

	num_links = 0;

	witem = soc_dbg_graph_find(gctx, SOC_DGRP_TYPE_PCM_DEV,
			&rtd->dpcm[SNDRV_PCM_STREAM_PLAYBACK]);
	if (witem)
		links[num_links++] = witem;

	dai = rtd->cpu_dai;
	if (dai && dai->playback_widget)
		soc_dbg_graph_links_add(gctx, dai->playback_widget,
			links, &num_links);

	dai = rtd->codec_dai;
	if (dai && dai->playback_widget)
		soc_dbg_graph_links_add(gctx, dai->playback_widget,
			links, &num_links);

	for (i = 0; i < rtd->num_codecs; i++) {
		dai = rtd->codec_dais[i];
		if (dai->playback_widget)
			soc_dbg_graph_links_add(gctx, dai->playback_widget,
				links, &num_links);
	}

	soc_dbg_graph_links_dup(links, num_links);
	if (dl->dpcm_playback || dl->playback_only) {
		snprintf(ops, PAGE_SIZE,
			"style=%s,color=%s,label=\"n:%s\\ns:%s\\n%s / %s\"",
			style, color, dl->name, dl->stream_name,
			dl->no_pcm ? "BE" : "FE", "Playback");

		for (i = 1; i < num_links; i++) {
			if (!links[i])
				continue;

			seq_printf(seq, "%s%p -> %p [%s];\n",
				gctx->level, links[0], links[i], ops);
		}
	}

	num_links = 0;

	witem = soc_dbg_graph_find(gctx, SOC_DGRP_TYPE_PCM_DEV,
			&rtd->dpcm[SNDRV_PCM_STREAM_CAPTURE]);
	if (witem)
		links[num_links++] = witem;

	dai = rtd->cpu_dai;
	if (dai && dai->capture_widget)
		soc_dbg_graph_links_add(gctx, dai->capture_widget,
			links, &num_links);

	dai = rtd->codec_dai;
	if (dai && dai->capture_widget)
		soc_dbg_graph_links_add(gctx, dai->capture_widget,
			links, &num_links);

	for (i = 0; i < rtd->num_codecs; i++) {
		dai = rtd->codec_dais[i];
		if (dai->capture_widget)
			soc_dbg_graph_links_add(gctx, dai->capture_widget,
				links, &num_links);
	}

	soc_dbg_graph_links_dup(links, num_links);
	if (dl->dpcm_capture || dl->capture_only) {
		snprintf(ops, PAGE_SIZE,
			"style=%s,color=%s,label=\"n:%s\\ns:%s\\n%s / %s\"",
			style, color, dl->name, dl->stream_name,
			dl->no_pcm ? "BE" : "FE", "Capture");

		for (i = 1; i < num_links; i++) {
			if (!links[i])
				continue;

			seq_printf(seq, "%s%p -> %p [%s];\n",
				gctx->level, links[i], links[0], ops);
		}
	}

	return 0;
}

static int soc_dbg_graph_show(struct seq_file *seq, void *v)
{
	struct soc_dbg_graph_ctx *gctx = seq->private;
	struct soc_dbg_graph_item *item = v;
	bool do_show = true;
	bool restore = false;
	char level[16] = {0};

	soc_dbg_graph_start_header(gctx);

	if (gctx->parent_prev != item->parent) {
		restore = true;
		strlcpy(level, gctx->level, sizeof(level));
		soc_dbg_graph_parent_footer(gctx, gctx->parent_prev, true);
		soc_dbg_graph_parent_header(gctx, item->parent);
		strlcpy(gctx->level_new, gctx->level, sizeof(gctx->level_new));
		gctx->parent_next = item->parent;
	}

	if (gctx->flg_used_only)
		do_show = item->flg_usage || item->flg_do_show;

	if (do_show) {
		switch (item->type) {
		case SOC_DGRP_TYPE_PCM_DEV:
			soc_dbg_graph_show_pcm_dev(gctx, item);
			break;

		case SOC_DGRP_TYPE_WIDGET:
			soc_dbg_graph_show_widgets(gctx, item);
			break;

		case SOC_DGRP_TYPE_PATH:
			soc_dbg_graph_show_path(gctx, item);
			break;

		case SOC_DGRP_TYPE_RTD:
			soc_dbg_graph_show_rtd(gctx, item);
			break;

		default:
			break;
		}
	}

	if (restore)
		strlcpy(gctx->level, level, sizeof(gctx->level));

	if (list_is_last(&item->item, &gctx->list))
		soc_dbg_graph_start_footer(gctx);

	return 0;
}

static const struct seq_operations soc_dbg_graph_seq_ops = {
	.start	= soc_dbg_graph_start,
	.stop	= soc_dbg_graph_stop,
	.next	= soc_dbg_graph_next,
	.show	= soc_dbg_graph_show,
};

static int soc_dbg_graph_all_open(struct inode *inode, struct file *file)
{
	struct soc_dbg_graph_ctx *gctx;
	struct seq_file *seq;
	int ret;

	ret = seq_open(file, &soc_dbg_graph_seq_ops);
	if (ret)
		return ret;

	gctx = kcalloc(1, sizeof(*gctx), GFP_KERNEL);
	if (!gctx)
		return -ENOMEM;

	seq = file->private_data;
	seq->private = gctx;

	gctx->card = inode->i_private;
	gctx->seq = file->private_data;
	INIT_LIST_HEAD(&gctx->list);
	INIT_LIST_HEAD(&gctx->parents);

	gctx->pcm_dev = soc_dbg_graph_add(gctx,
			SOC_DGRP_TYPE_PARENT, gctx->card);
	gctx->pcm_dev->name = kstrdup("PCM Devices", GFP_KERNEL);

	soc_dbg_graph_add_rtd(gctx);
	soc_dbg_graph_add_widgets(gctx);
	soc_dbg_graph_add_path(gctx);
	soc_dbg_graph_populate_parents(gctx);
	soc_dbg_graph_populate_usage(gctx);
	soc_dbg_graph_parent_names(gctx);
	soc_dbg_graph_sort(gctx);

	return ret;
}

static int soc_dbg_graph_used_open(struct inode *inode, struct file *file)
{
	struct soc_dbg_graph_ctx *gctx;
	struct seq_file *seq;
	int ret;

	ret = soc_dbg_graph_all_open(inode, file);
	if (ret)
		return ret;

	seq = file->private_data;
	gctx = seq->private;

	gctx->flg_used_only = 1;

	return ret;
}

static int soc_dbg_graph_release(struct inode *inode, struct file *file)
{
	struct seq_file *seq = file->private_data;
	struct soc_dbg_graph_ctx *gctx = seq->private;

	soc_dbg_graph_remove_items(&gctx->list);
	soc_dbg_graph_remove_items(&gctx->parents);
	kfree(gctx);

	return seq_release(inode, file);
}

static const struct file_operations soc_dbg_graph_all_fops = {
	.open = soc_dbg_graph_all_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = soc_dbg_graph_release,
};

static const struct file_operations soc_dbg_graph_used_fops = {
	.open = soc_dbg_graph_used_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = soc_dbg_graph_release,
};

static int soc_dbg_attr_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t soc_dbg_attr_read_val(char *b, int len,
	const char *name, char **values, int bit0, int bit1, int bit2)
{
	int index = 0;

	index += (!!bit0 << 0);
	index += (!!bit1 << 1);
	index += (!!bit2 << 2);

	return snprintf(b, len, "%s: %s\n", name, values[index]);
}

ssize_t soc_dbg_attr_read(struct file *f, char __user *ubuf,
			  size_t count, loff_t *pos)
{
	struct snd_soc_card *card = f->private_data;
	struct snd_soc_card_debugfs *dbg = card->debugfs;
	ssize_t len, ret = 0;
	char *buf, *b;

	buf = b = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += soc_dbg_attr_read_val(b, PAGE_SIZE, "rtd.directions",
		soc_dbg_vals_dir, dbg->flg_d_fe_play, dbg->flg_d_fe_cap, 0);
	b = buf + len;

	len += soc_dbg_attr_read_val(b, PAGE_SIZE, "widgets.connected",
		soc_dbg_vals_conn, dbg->flg_w_conn, dbg->flg_w_disc, 0);
	b = buf + len;

	len += soc_dbg_attr_read_val(b, PAGE_SIZE, "widgets.active",
		soc_dbg_vals_act, dbg->flg_w_active, dbg->flg_w_inactive, 0);
	b = buf + len;

	len += soc_dbg_attr_read_val(b, PAGE_SIZE, "path.connected",
		soc_dbg_vals_conn, dbg->flg_p_conn, dbg->flg_p_disc, 0);
	b = buf + len;

	ret = simple_read_from_buffer(ubuf, count, pos, buf, len);

	kfree(buf);

	return ret;
}

int soc_dbg_attr_find(const char __user *buf, const char *name, char **vals)
{
	int index = 0;
	char *var;

	var = strnstr(buf, name, strlen(buf));
	if (!var)
		return -ENOENT;

	var += strlen(name);
	for (; *var; var++) {
		if (!strnchr(": ", 2, *var))
			break;
	}

	if (!*var)
		return -ENODATA;

	for (index = 0; *vals; vals++, index++) {
		if (strnstr(var, *vals, strlen(var)))
			return index;
	}

	return -EINVAL;
}

ssize_t soc_dbg_attr_write(struct file *f, const char __user *buf,
			   size_t size, loff_t *pos)
{
	struct snd_soc_card *card = f->private_data;
	struct snd_soc_card_debugfs *dbg = card->debugfs;
	char *b = dbg->buf;
	int res;

	res = copy_from_user(b, buf, min(PAGE_SIZE, size));
	if (res)
		return res;

	res = soc_dbg_attr_find(b, "rtd.directions", soc_dbg_vals_dir);
	if (res != -ENOENT) {
		if (res < 0)
			return res;
		dbg->flg_d_fe_play = (res >> 0) & 0x1;
		dbg->flg_d_fe_cap = (res >> 1) & 0x1;
		return size;
	}

	res = soc_dbg_attr_find(b, "widgets.connected", soc_dbg_vals_conn);
	if (res != -ENOENT) {
		if (res < 0)
			return res;
		dbg->flg_w_conn = (res >> 0) & 0x1;
		dbg->flg_w_disc = (res >> 1) & 0x1;
		return size;
	}

	res = soc_dbg_attr_find(b, "widgets.active", soc_dbg_vals_act);
	if (res != -ENOENT) {
		if (res < 0)
			return res;
		dbg->flg_w_active = (res >> 0) & 0x1;
		dbg->flg_w_inactive = (res >> 1) & 0x1;
		return size;
	}

	res = soc_dbg_attr_find(b, "path.connected", soc_dbg_vals_conn);
	if (res != -ENOENT) {
		if (res < 0)
			return res;
		dbg->flg_p_conn = (res >> 0) & 0x1;
		dbg->flg_p_disc = (res >> 1) & 0x1;
		return size;
	}

	return -EINVAL;
}

static const struct file_operations soc_dbg_attr_fops = {
	.open = soc_dbg_attr_open,
	.read = soc_dbg_attr_read,
	.write = soc_dbg_attr_write,
};

struct soc_dbg_kctrl_w_ctx {
	struct seq_file *seq;
	struct snd_soc_card *card;
	struct snd_soc_dapm_widget *widget;
	struct snd_kcontrol *kctrl;
	unsigned int flg_header:1;		/* print graph header */
	unsigned int flg_footer:1;		/* print graph footer */
	unsigned int flg_widget_header:1;	/* print widget header */
	unsigned int flg_widget_footer:1;	/* print widget footer */
	unsigned int flg_kctrl:1;		/* print kcontrol */
	int kctrl_idx;
};

static int soc_dbg_kctrl_widget_next_item(struct soc_dbg_kctrl_w_ctx *kctx)
{
	struct snd_soc_card *card = kctx->card;
	struct snd_soc_dapm_widget *w = kctx->widget;
	struct list_head *whead = &card->widgets;

	if (kctx->flg_footer) {
		kctx->widget = NULL;
		return -ENOENT;
	}

	if (w && w->kcontrols && w->num_kcontrols > 0) {
		if (++kctx->kctrl_idx < w->num_kcontrols) {
			int idx = kctx->kctrl_idx;

			kctx->kctrl = w->kcontrols[idx];
			kctx->flg_kctrl = !!kctx->kctrl;
			kctx->flg_widget_footer = idx == w->num_kcontrols - 1;
			return 0;
		}
	}

	kctx->kctrl = NULL;
	kctx->widget = NULL;
	kctx->kctrl_idx = -1;
	kctx->flg_widget_header = 0;
	kctx->flg_widget_footer = 0;
	kctx->flg_kctrl = 0;

	if (!w)
		w = list_entry(whead, struct snd_soc_dapm_widget, list);

	list_for_each_entry_continue(w, whead, list) {
		if (w->num_kcontrols <= 0)
			continue;

		kctx->widget = w;
		kctx->flg_widget_header = 1;
		break;
	}

	if (!kctx->widget && !kctx->flg_header && !kctx->flg_footer) {
		kctx->widget = (void *)kctx;
		kctx->flg_footer = 1;
	}

	return kctx->widget ? 0 : -ENOENT;
}


static void *soc_dbg_kctrl_widget_start(struct seq_file *seq, loff_t *pos)
{
	struct soc_dbg_kctrl_w_ctx *kctx = seq->private;

	if (!*pos)
		soc_dbg_kctrl_widget_next_item(kctx);

	return kctx->widget;
}

static void soc_dbg_kctrl_widget_stop(struct seq_file *seq, void *v)
{
}

static void *soc_dbg_kctrl_widget_next(struct seq_file *seq, void *v,
	loff_t *pos)
{
	struct soc_dbg_kctrl_w_ctx *kctx = seq->private;

	kctx->flg_header = 0;
	kctx->flg_widget_header = 0;
	kctx->flg_widget_footer = 0;
	kctx->flg_kctrl = 0;

	if (!soc_dbg_kctrl_widget_next_item(kctx))
		(*pos)++;

	return kctx->widget;
}

static void soc_dbg_kctrl_widget_start_header(struct soc_dbg_kctrl_w_ctx *kctx)
{
	struct seq_file *seq = kctx->seq;
	char *buf = kctx->card->debugfs->buf;

	strlcpy(buf, kctx->card->name, PAGE_SIZE);
	soc_dbg_fix_name(buf);

	seq_printf(seq, "digraph %s {\n", buf);
	seq_puts(seq, "\tlayout=fdp;\n");
	seq_puts(seq, "\n");
}

static void soc_dbg_kctrl_widget_show_widget(struct soc_dbg_kctrl_w_ctx *kctx)
{
	struct seq_file *seq = kctx->seq;
	const struct snd_soc_dapm_widget *w = kctx->widget;

	seq_printf(seq, "\tsubgraph cluster_%p {\n", w);

	seq_printf(seq, "\t\t%p [color=%s,label=\"t:%s\\nn:%s\\ns:%s",
		w, "red", soc_dbg_widget_id(w->id),
		w->name ?: "", w->sname ?: "");
	if (w->is_ep)
		seq_printf(seq, "\\nep:%s,%s",
			w->is_ep & SND_SOC_DAPM_EP_SOURCE ? "source" : "-",
			w->is_ep & SND_SOC_DAPM_EP_SINK ? "sink" : "-");
	if (w->connected)
		seq_puts(seq, "\\nf:connected");
	if (w->active)
		seq_puts(seq, "\\nf:active");

	seq_puts(seq, "\"];\n");
}

static void soc_dbg_kctrl_widget_show_kctrl(struct soc_dbg_kctrl_w_ctx *kctx)
{
	struct seq_file *seq = kctx->seq;
	const struct snd_soc_dapm_widget *w = kctx->widget;
	const struct snd_kcontrol *k = kctx->kctrl;
	const char *kname, *kprefix = "";

	if (!w || !k)
		return;

	kname = k->id.name;
	if (strnstr(kname, w->name, strlen(kname)) == kname) {
		kprefix = "...";
		kname += strlen(w->name);
	}

	if (!strlen(kname))
		kname = k->id.name;

	seq_printf(seq, "\t\t%p_%p [color=%s,label=\"%s%s\"];\n",
		k, w, "blue", kprefix, kname);
	seq_printf(seq, "\t\t%p_%p -> %p [color=%s];\n",
		k, w, w, "green");
}

static int soc_dbg_kctrl_widget_show(struct seq_file *seq, void *v)
{
	struct soc_dbg_kctrl_w_ctx *kctx = seq->private;

	if (kctx->flg_header)
		soc_dbg_kctrl_widget_start_header(kctx);

	if (kctx->flg_widget_header)
		soc_dbg_kctrl_widget_show_widget(kctx);

	if (kctx->kctrl)
		soc_dbg_kctrl_widget_show_kctrl(kctx);

	if (kctx->flg_widget_footer)
		seq_puts(seq, "\t}\n\n");

	if (kctx->flg_footer)
		seq_puts(seq, "}\n");

	return 0;
}

static const struct seq_operations soc_dbg_kctrl_widget_seq_ops = {
	.start	= soc_dbg_kctrl_widget_start,
	.stop	= soc_dbg_kctrl_widget_stop,
	.next	= soc_dbg_kctrl_widget_next,
	.show	= soc_dbg_kctrl_widget_show,
};

static int soc_dbg_kctrl_widget_open(struct inode *inode, struct file *file)
{
	struct soc_dbg_kctrl_w_ctx *kctx;
	struct seq_file *seq;
	int ret;

	ret = seq_open(file, &soc_dbg_kctrl_widget_seq_ops);
	if (ret)
		return ret;

	kctx = kcalloc(1, sizeof(*kctx), GFP_KERNEL);
	if (!kctx)
		return -ENOMEM;

	seq = file->private_data;
	seq->private = kctx;

	kctx->card = inode->i_private;
	kctx->seq = file->private_data;
	kctx->flg_header = 1;

	return ret;
}

static int soc_dbg_kctrl_widget_release(struct inode *inode, struct file *file)
{
	struct seq_file *seq = file->private_data;
	struct soc_dbg_kctrl_w_ctx *kctx = seq->private;

	kfree(kctx);

	return seq_release(inode, file);
}

static const struct file_operations soc_dbg_kctrl_widget_fops = {
	.open = soc_dbg_kctrl_widget_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = soc_dbg_kctrl_widget_release,
};

int snd_soc_card_debugfs_init(struct snd_soc_card *card)
{
	struct snd_soc_card_debugfs *dbg;

	dbg = devm_kcalloc(card->dev, 1, sizeof(*dbg), GFP_KERNEL);
	if (!dbg)
		return -ENOMEM;

	dbg->buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!dbg->buf) {
		kfree(dbg);
		return -ENOMEM;
	}

	card->debugfs = dbg;
	dbg->flg_p_conn = 1;
	dbg->flg_p_disc = 1;
	dbg->flg_w_conn = 1;
	dbg->flg_w_disc = 1;
	dbg->flg_w_active = 1;
	dbg->flg_w_inactive = 1;
	dbg->flg_d_fe_play = 1;
	dbg->flg_d_fe_cap = 1;

	dbg->root = debugfs_create_dir("asoc-dump", card->debugfs_card_root);

	dbg->attr = debugfs_create_file("attributes", 0644,
		dbg->root, card, &soc_dbg_attr_fops);

	dbg->rtd = debugfs_create_file("rtd", 0644,
		dbg->root, card, &soc_dbg_rtd_fops);

	dbg->widgets = debugfs_create_file("widgets", 0644,
		dbg->root, card, &soc_dbg_widgets_fops);

	dbg->routes = debugfs_create_file("routes", 0644,
		dbg->root, card, &soc_dbg_routes_fops);

	dbg->dapm = debugfs_create_file("dapm", 0644,
		dbg->root, card, &soc_dbg_dapm_fops);

	dbg->dapm_durty = debugfs_create_file("dapm_durty", 0644,
		dbg->root, card, &soc_dbg_dapm_durty_fops);

	dbg->graph_all = debugfs_create_file("graph_all.dot", 0644,
		dbg->root, card, &soc_dbg_graph_all_fops);

	dbg->graph_used = debugfs_create_file("graph_used.dot", 0644,
		dbg->root, card, &soc_dbg_graph_used_fops);

	dbg->kcontrols = debugfs_create_file("kcontrols.dot", 0644,
		dbg->root, card, &soc_dbg_kctrl_widget_fops);

	return 0;
}

void snd_soc_card_debugfs_deinit(struct snd_soc_card *card)
{
	struct snd_soc_card_debugfs *dbg = card->debugfs;

	debugfs_remove(dbg->kcontrols);
	debugfs_remove(dbg->graph_used);
	debugfs_remove(dbg->graph_all);
	debugfs_remove(dbg->dapm_durty);
	debugfs_remove(dbg->dapm);
	debugfs_remove(dbg->routes);
	debugfs_remove(dbg->widgets);
	debugfs_remove(dbg->rtd);
	debugfs_remove(dbg->attr);
	debugfs_remove(dbg->root);
	devm_kfree(card->dev, dbg);
}

#else
int snd_soc_sysfs_init(struct snd_soc_card *card)
{
	return 0;
}

void snd_soc_card_debugfs_deinit(struct snd_soc_card *card)
{
}
#endif
