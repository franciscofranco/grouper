/*
 * tegra_p1852.c - Tegra machine ASoC driver for P1852 Boards.
 *
 * Author: Nitin Pai <npai@nvidia.com>
 * Copyright (C) 2010-2012 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 * Stephen Warren <swarren@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/tegra_p1852_pdata.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#define DRV_NAME "tegra-snd-p1852"

struct tegra_p1852 {
	struct tegra_asoc_utils_data util_data;
	struct tegra_p1852_platform_data *pdata;
};

static int tegra_p1852_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_p1852 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk;
	int i2s_daifmt = 0;
	int err;
	struct tegra_p1852_platform_data *pdata;
	int codec_id = codec_dai->id;

	pdata = machine->pdata;

	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	if (pdata->codec_info[codec_id].master)
		i2s_daifmt |= SND_SOC_DAIFMT_CBM_CFM;
	else
		i2s_daifmt |= SND_SOC_DAIFMT_CBS_CFS;

	switch (pdata->codec_info[codec_id].i2s_format) {
	case format_tdm:
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
		break;
	case format_i2s:
		i2s_daifmt |= SND_SOC_DAIFMT_I2S;
		break;
	case format_rjm:
		i2s_daifmt |= SND_SOC_DAIFMT_RIGHT_J;
		break;
	case format_ljm:
		i2s_daifmt |= SND_SOC_DAIFMT_LEFT_J;
		break;
	default:
		break;
	}

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0)
		dev_info(card->dev, "codec_dai fmt not set\n");

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0)
		dev_info(card->dev, "codec_dai clock not set\n");

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_p1852 *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

static struct snd_soc_ops tegra_p1852_ops = {
	.hw_params = tegra_p1852_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_dai_link tegra_p1852_dai_link[] = {
	{
		.name = "I2S-TDM-1",
		.stream_name = "TEGRA PCM",
		.platform_name = "tegra-pcm-audio",
		.ops = &tegra_p1852_ops,
	},
	{
		.name = "I2S-TDM-2",
		.stream_name = "TEGRA PCM",
		.platform_name = "tegra-pcm-audio",
		.ops = &tegra_p1852_ops,
	}
};

static struct snd_soc_card snd_soc_tegra_p1852 = {
	.name = "tegra-p1852",
	.dai_link = tegra_p1852_dai_link,
	.num_links = ARRAY_SIZE(tegra_p1852_dai_link),
};

static __devinit int tegra_p1852_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_p1852;
	struct tegra_p1852 *machine;
	struct tegra_p1852_platform_data *pdata;
	int ret;
	int i;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_p1852), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_p1852 struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;

	/* The codec driver and codec dai have to come from the system
	 * level board configuration file
	 * */
	for (i = 0; i < ARRAY_SIZE(tegra_p1852_dai_link); i++) {
		tegra_p1852_dai_link[i].codec_name =
				pdata->codec_info[i].codec_name;
		tegra_p1852_dai_link[i].cpu_dai_name =
				pdata->codec_info[i].cpu_dai_name;
		tegra_p1852_dai_link[i].codec_dai_name =
				pdata->codec_info[i].codec_dai_name;
		tegra_p1852_dai_link[i].name =
				pdata->codec_info[i].name;
	}

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev);
	if (ret)
		goto err_free_machine;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
	}

	if (!card->instantiated) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_card;
	}

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_p1852_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_p1852 *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);
	tegra_asoc_utils_fini(&machine->util_data);
	kfree(machine);

	return 0;
}

static struct platform_driver tegra_p1852_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_p1852_driver_probe,
	.remove = __devexit_p(tegra_p1852_driver_remove),
};

static int __init tegra_p1852_modinit(void)
{
	return platform_driver_register(&tegra_p1852_driver);
}
module_init(tegra_p1852_modinit);

static void __exit tegra_p1852_modexit(void)
{
	platform_driver_unregister(&tegra_p1852_driver);
}
module_exit(tegra_p1852_modexit);

MODULE_AUTHOR("Nitin Pai <npai@nvidia.com>");
MODULE_DESCRIPTION("Tegra+P1852 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
