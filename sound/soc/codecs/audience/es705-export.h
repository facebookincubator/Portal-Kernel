/*
 * es705-export.h  --
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ES705_EXPORT_H
#define _ES705_EXPORT_H

#include <sound/soc.h>
#include <sound/soc-dai.h>

#if defined(CONFIG_SND_SOC_ES705)

int es705_slim_set_channel_map(struct snd_soc_dai *dai,
			       unsigned int tx_num, unsigned int *tx_slot,
			       unsigned int rx_num, unsigned int *rx_slot);
int es705_slim_get_channel_map(struct snd_soc_dai *dai,
			       unsigned int *tx_num, unsigned int *tx_slot,
			       unsigned int *rx_num, unsigned int *rx_slot);
int es705_slim_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai);
int es705_remote_cfg_slim_rx(int dai_id);
int es705_remote_cfg_slim_tx(int dai_id);
int es705_remote_close_slim_rx(int dai_id);
int es705_remote_close_slim_tx(int dai_id);
int es705_remote_add_codec_controls(struct snd_soc_codec *codec);

int es705_remote_route_enable(struct snd_soc_dai *dai);

#elif defined(CONFIG_SND_SOC_ES705_ESCORE) || \
	defined(CONFIG_SND_SOC_ES704_ESCORE) || \
	defined(CONFIG_SND_SOC_ES804_ESCORE)
int escore_slim_set_channel_map(struct snd_soc_dai *dai,
			       unsigned int tx_num, unsigned int *tx_slot,
			       unsigned int rx_num, unsigned int *rx_slot);
int escore_slim_get_channel_map(struct snd_soc_dai *dai,
			       unsigned int *tx_num, unsigned int *tx_slot,
			       unsigned int *rx_num, unsigned int *rx_slot);
int escore_slim_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai);
int escore_remote_cfg_slim_rx(int dai_id);
int escore_remote_cfg_slim_tx(int dai_id);
int escore_remote_close_slim_rx(int dai_id);
int escore_remote_close_slim_tx(int dai_id);
int es705_remote_add_codec_controls(struct snd_soc_codec *codec);

int es705_remote_route_enable(struct snd_soc_dai *dai);

int es705_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai);
void es705_i2s_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai);
#endif


#endif /* _ES705_EXPORT_H */
