/* nec-terratec-cinergy-xs.h - Keytable for nec_terratec_cinergy_xs Remote Controller
 *
 * keymap imported from ir-keymaps.c
 *
 * Copyright (c) 2010 by Mauro Carvalho Chehab
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/rc-map.h>
#include <linux/module.h>

/* Terratec Cinergy Hybrid T USB XS FM
   Mauro Carvalho Chehab
 */

static struct rc_map_table nec_cd80[] = {

	/* Terratec Grey IR, with most keys in orange */
	{ 0x812C, KEY_POWER2},            //开机(长按CD)
	{ 0x811A, KEY_EJECTCLOSECD},          //出仓
	{ 0x8112, KEY_PLAYPAUSE},        //播放暂停
	{ 0x811E, KEY_F1},               //单曲重复播放
	{ 0x8101, KEY_REWIND},           //快退
	{ 0x8102, KEY_FORWARD},          //快进
	{ 0x811C, KEY_F2},               //随机播放
	{ 0x8108, KEY_F4},               //INPUT(-)
	{ 0x811B, KEY_F5},               //INPUT(+)
	{ 0x810A, KEY_MUTE},             //MUTE
	{ 0x8107, KEY_BRIGHTNESS_CYCLE}, //dimmer
	{ 0x8109, KEY_VOLUMEUP},         //vol+
	{ 0x811F, KEY_VOLUMEDOWN},       //vol-
};

static struct rc_map_list nec_cd80_map = {
	.map = {
		.scan    = nec_cd80,
		.size    = ARRAY_SIZE(nec_cd80),
		.rc_type = RC_TYPE_NEC,
		.name    = RC_MAP_NEC_CD80,
	}
};

static int __init init_rc_map_nec_cd80(void)
{
	return rc_map_register(&nec_cd80_map);
}

static void __exit exit_rc_map_nec_cd80(void)
{
	rc_map_unregister(&nec_cd80_map);
}

module_init(init_rc_map_nec_cd80)
module_exit(exit_rc_map_nec_cd80)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mauro Carvalho Chehab");


