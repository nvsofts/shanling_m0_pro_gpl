/*
 * u_audio.c -- ALSA audio utilities for Gadget stack
 *
 * Copyright (C) 2008 Bryan Wu <cooloney@kernel.org>
 * Copyright (C) 2008 Analog Devices, Inc
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <asm/atomic.h>

#include "u_audio.h"

/*
 * This component encapsulates the ALSA devices for USB audio gadget
 */

//#define USB_V1

#ifdef CONFIG_ANDROID
#define FILE_PCM_PLAYBACK	"/dev/snd/pcmC0D0p"
#define FILE_DSD_PLAYBACK	"/dev/snd/pcmC0D1p"

#define FILE_PCM_CAPTURE	"/dev/snd/pcmC0D2c"
#define FILE_CONTROL		"/dev/snd/controlC0"
#else
#define FILE_PCM_PLAYBACK	"/dev/pcmC0D0p"
#define FILE_PCM_CAPTURE	"/dev/pcmC0D0c"
#define FILE_CONTROL		"/dev/controlC0"
#endif

static char *fn_play = FILE_PCM_PLAYBACK;
module_param(fn_play, charp, S_IRUGO);
MODULE_PARM_DESC(fn_play, "Playback PCM device file name");

static char *fn_cap = FILE_PCM_CAPTURE;
module_param(fn_cap, charp, S_IRUGO);
MODULE_PARM_DESC(fn_cap, "Capture PCM device file name");

static char *fn_cntl = FILE_CONTROL;
module_param(fn_cntl, charp, S_IRUGO);
MODULE_PARM_DESC(fn_cntl, "Control device file name");


extern int actual_rate;
extern int usbdac_dsdmode;
extern bool usbdac_muteFlag;
bool uac_rate_flag = true;
extern void usbdac_setdsdmode(int setflag);
extern void usbdac_changemute(int mute);


static int playback_reinit_dev(struct gaudio *card);

/*-------------------------------------------------------------------------*/

/**
 * Some ALSA internal helper functions
 */
static int snd_interval_refine_set(struct snd_interval *i, unsigned int val)
{
	struct snd_interval t;
	t.empty = 0;
	t.min = t.max = val;
	t.openmin = t.openmax = 0;
	t.integer = 1;
	return snd_interval_refine(i, &t);
}

static int _snd_pcm_hw_param_set(struct snd_pcm_hw_params *params,
				 snd_pcm_hw_param_t var, unsigned int val,
				 int dir)
{
	int changed;
	if (hw_is_mask(var)) {
		struct snd_mask *m = hw_param_mask(params, var);
		if (val == 0 && dir < 0) {
			changed = -EINVAL;
			snd_mask_none(m);
		} else {
			if (dir > 0)
				val++;
			else if (dir < 0)
				val--;
			changed = snd_mask_refine_set(
					hw_param_mask(params, var), val);
		}
	} else if (hw_is_interval(var)) {
		struct snd_interval *i = hw_param_interval(params, var);
		if (val == 0 && dir < 0) {
			changed = -EINVAL;
			snd_interval_none(i);
		} else if (dir == 0)
			changed = snd_interval_refine_set(i, val);
		else {
			struct snd_interval t;
			t.openmin = 1;
			t.openmax = 1;
			t.empty = 0;
			t.integer = 0;
			if (dir < 0) {
				t.min = val - 1;
				t.max = val;
			} else {
				t.min = val;
				t.max = val+1;
			}
			changed = snd_interval_refine(i, &t);
		}
	} else
		return -EINVAL;
	if (changed) {
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
	}
	return changed;
}
/*-------------------------------------------------------------------------*/

static int default_hw_params(struct gaudio_snd_dev *snd)
{
	struct snd_pcm_substream *substream = snd->substream;
	struct snd_pcm_hw_params *params;
	snd_pcm_sframes_t result;

	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	_snd_pcm_hw_params_any(params);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_ACCESS,
			snd->access, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_FORMAT,
			snd->format, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_CHANNELS,
			snd->channels, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_RATE,
			snd->rate, 0);

	result = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_DROP, NULL);
	if (result < 0)  printk("===SNDRV_PCM_IOCTL_DROP===result[%d]====\n",(int)result);
	result = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_HW_PARAMS, params);
	if (result < 0)  printk("=SNDRV_PCM_IOCTL_HW_PARAMS==SNDRV_PCM_IOCTL_DROP===result[%d]====\n",(int)result);

	result = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
	if (result < 0) {
		printk("Preparing sound card failed: %d\n", (int)result);
		kfree(params);
		return result;
	}

	/* Store the hardware parameters */
	snd->access = params_access(params);
	snd->format = params_format(params);
	snd->channels = params_channels(params);
	snd->rate = params_rate(params);

	kfree(params);

	INFO(snd->card,
		"%s Hardware params: access %x, format %x, channels %d, rate %d\n",
		substream->name, snd->access, snd->format, snd->channels, snd->rate);
	return 0;
}

static int playback_prepare_params(struct gaudio_snd_dev *snd)
{

       /*
	* SNDRV_PCM_ACCESS_RW_INTERLEAVED,
	* SNDRV_PCM_FORMAT_S16_LE
	* CHANNELS: 2
	* RATE: 48000
	*/
	snd->access = SNDRV_PCM_ACCESS_RW_INTERLEAVED;
	snd->format = SNDRV_PCM_FORMAT_S16_LE;
	snd->channels = UAC_DEFAULT_CH;
	snd->rate = actual_rate;

	return default_hw_params(snd);
}


/**
 * Playback audio buffer data by ALSA PCM device
 */
static size_t u_audio_playback(struct gaudio *card, void *buf, size_t count,int tmpdsdmode)
{
	struct gaudio_snd_dev	*snd = &card->playback;
	struct snd_pcm_substream *substream = snd->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	mm_segment_t old_fs;
	ssize_t result;
	snd_pcm_sframes_t frames;

#ifndef USB_V1
	if (runtime->rate != actual_rate || tmpdsdmode != usbdac_dsdmode)		uac_rate_flag = true;

	if(uac_rate_flag)
	{
		uac_rate_flag = false;
		//printk("===========actual_rate[%d]=====\n",actual_rate);
	    runtime->rate = actual_rate;
	
		if(tmpdsdmode == 1 || tmpdsdmode == 2)
			usbdac_setdsdmode(1);
		else
			usbdac_setdsdmode(0);
		
		if(!usbdac_muteFlag)
		{
			usbdac_muteFlag = true;
			usbdac_changemute(usbdac_muteFlag);
		}
	
		if(usbdac_dsdmode == 0 && usbdac_dsdmode == tmpdsdmode) //pcm«–ªª
		{
			result = playback_prepare_params(snd);
			if (result) 
			{
				pr_err("Failed to init audio streams");
				return 0;
			}
		}
		else
		{
			usbdac_dsdmode = tmpdsdmode;
			result = playback_reinit_dev(card);
			if (result < 0) 
			{
				pr_err("Failed to reinit audio streams");
				return 0;
			}
			snd = &card->playback;
			substream = snd->substream;
			runtime = substream->runtime;		
		}
		return 0;
	}
#endif


if(usbdac_muteFlag)
{
	usbdac_muteFlag = false;
	usbdac_changemute(usbdac_muteFlag);
}
try_again:
	if (runtime->status->state == SNDRV_PCM_STATE_XRUN ||
		runtime->status->state == SNDRV_PCM_STATE_SUSPENDED) {
		printk("========xrun=========\n");
		result = snd_pcm_kernel_ioctl(substream,
				SNDRV_PCM_IOCTL_PREPARE, NULL);
		if (result < 0) {
			printk("Preparing sound card failed: %d\n",
					(int)result);
			return result;
		}
	}
	frames = bytes_to_frames(runtime, count);


	old_fs = get_fs();
	set_fs(KERNEL_DS);
	result = snd_pcm_lib_write(snd->substream, buf, frames);
	if (result != frames) {
		ERROR(card, "Playback error: %d\n", (int)result);
		set_fs(old_fs);
		goto try_again;
	}
	set_fs(old_fs);

	return 0;
}

#ifdef USB_V1

static int capture_default_hw_params(struct gaudio_snd_dev *snd)
{
	snd->access = SNDRV_PCM_ACCESS_RW_INTERLEAVED;
	snd->format = SNDRV_PCM_FORMAT_S16_LE;
	snd->channels = MIC_CH;
	snd->rate = MIC_SATE;

	return default_hw_params(snd);
}

static size_t u_audio_capture(struct gaudio *card, void *buf, size_t count)
{
	struct gaudio_snd_dev	*snd = &card->capture;
	struct snd_pcm_substream *substream = snd->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	ssize_t result;
	mm_segment_t old_fs;
	snd_pcm_sframes_t frames;

try_again:
	if (runtime->status->state == SNDRV_PCM_STATE_XRUN ||
			runtime->status->state == SNDRV_PCM_STATE_SUSPENDED) {
		result = snd_pcm_kernel_ioctl(substream,
				SNDRV_PCM_IOCTL_PREPARE, NULL);
		if (result < 0) {
			ERROR(card, "Preparing sound card failed: %d\n",
					(int)result);
			return result;
		}
	}

	frames = bytes_to_frames(runtime, count);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	result = snd_pcm_lib_read(snd->substream, buf, frames);
	if (result != frames) {
		ERROR(card, "Capture error: %d\n", (int)result);
		set_fs(old_fs);
		goto try_again;
	}
	set_fs(old_fs);

	return result;
}

static int u_audio_get_playback_channels(struct gaudio *card)
{
	return card->playback.channels;
}

static int u_audio_get_playback_rate(struct gaudio *card)
{
	return card->playback.rate;
}
static int u_audio_get_capture_channels(struct gaudio *card)
{
	return card->capture.channels;
}

static int u_audio_get_capture_rate(struct gaudio *card)
{
	return card->capture.rate;
}
#endif
/**
 * Open ALSA PCM and control device files
 * Initial the PCM or control device
 */
static int gaudio_open_snd_dev(struct gaudio *card)
{
	struct snd_pcm_file *pcm_file;
	struct gaudio_snd_dev *snd;
	int ret = 0;

	if (!card)
		return -ENODEV;

	/* Open control device */
	snd = &card->control;
	snd->filp = filp_open(fn_cntl, O_RDWR, 0);
	if (IS_ERR(snd->filp)) {
		ret = PTR_ERR(snd->filp);
		ERROR(card, "unable to open sound control device file: %s\n",
				fn_cntl);
		snd->filp = NULL;
		goto control_failed;
	}
	snd->card = card;

	/* Open PCM playback device and setup substream */
	snd = &card->playback;
	if(usbdac_dsdmode == 1 || usbdac_dsdmode == 2)
		snd->filp = filp_open(FILE_DSD_PLAYBACK, O_WRONLY, 0);
	else
		snd->filp = filp_open(fn_play, O_WRONLY, 0);
	if (IS_ERR(snd->filp)) {
		ret = PTR_ERR(snd->filp);
		ERROR(card, "No such PCM playback device: %s\n", fn_play);
		snd->filp = NULL;
		filp_close(card->control.filp, current->files);
		goto playback_failed;
	}
	pcm_file = snd->filp->private_data;
	snd->substream = pcm_file->substream;
	snd->card = card;
	ret = playback_prepare_params(snd);
	if(ret < 0)		return ret;
#ifdef USB_V1
	/* Open PCM capture device and setup substream */
	snd = &card->capture;
	snd->filp = filp_open(fn_cap, O_RDONLY, 0);
	if (IS_ERR(snd->filp)) {
		ERROR(card, "No such PCM capture device: %s\n", fn_cap);
		ret = PTR_ERR(snd->filp);
		snd->substream = NULL;
		snd->card = NULL;
		snd->filp = NULL;
		filp_close(card->playback.filp, current->files);
		filp_close(card->control.filp, current->files);
		goto capture_failed;
	}
	pcm_file = snd->filp->private_data;
	snd->substream = pcm_file->substream;
	snd->card = card;
	capture_default_hw_params(snd);
#endif
	return 0;
#ifdef USB_V1
capture_failed:
	card->playback.filp = NULL;
	card->playback.substream = NULL;
	card->playback.card = NULL;
	card->playback.channels = 0;
	card->playback.access = 0;
	card->playback.rate = 0;
	card->playback.format = 0;
#endif
playback_failed:
	card->control.card = NULL;
control_failed:
	return ret;
}

/**
 * Close ALSA PCM and control device files
 */
static int gaudio_close_snd_dev(struct gaudio *gau)
{
	struct gaudio_snd_dev	*snd;

	/* Close control device */
	snd = &gau->control;
	if (snd->filp)
		filp_close(snd->filp, current->files);

	/* Close PCM playback device and setup substream */
	snd = &gau->playback;
	if (snd->filp)
		filp_close(snd->filp, current->files);

	/* Close PCM capture device and setup substream */
	snd = &gau->capture;
	if (snd->filp)
		filp_close(snd->filp, current->files);


	return 0;
}

static struct gaudio *the_card;
static struct f_audio *audio_card;
/**
 * gaudio_setup - setup ALSA interface and preparing for USB transfer
 *
 * This sets up PCM, mixer or MIDI ALSA devices fore USB gadget using.
 *
 * Returns negative errno, or zero on success
 */
int __init gaudio_setup(struct f_audio * audio)
{
	int	ret;

	audio_card = audio;
	ret = gaudio_open_snd_dev(&audio->card);
	if (ret)
	{
		ERROR(&audio->card, "we need at least one control device\n");
		the_card = NULL;
	}
	else if (!the_card)
		the_card = &audio->card;

	return ret;

}

/**
 * gaudio_cleanup - remove ALSA device interface
 *
 * This is called to free all resources allocated by @gaudio_setup().
 */
void gaudio_cleanup(void)
{

	if (the_card) {
		gaudio_close_snd_dev(the_card);
		the_card = NULL;
	}

	kfree(audio_card);

}


static int playback_reinit_dev(struct gaudio *card)
{
	int res = 0;
	gaudio_close_snd_dev(card);
	res = gaudio_open_snd_dev(card);
	return res;
}


