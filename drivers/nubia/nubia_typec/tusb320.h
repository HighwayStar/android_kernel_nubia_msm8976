/*

 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __TUSB320__
#define __TUSB320__

typedef unsigned char BYTE;
#define CC_TYPE_MASK 0x0E

enum tusb320_type_status{
	TUSB320_TYPE_STATUS_DEFAULT = 0x00,
	TUSB320_TYPE_STATUS_DFP = 0x40,
	TUSB320_TYPE_STATUS_UFP = 0x80,
	TUSB320_TYPE_STATUS_ACCESSORY = 0xc0,
};

enum tusb320_type{
	TUSB320_TYPE_DEFAULT = 0,
	TUSB320_TYPE_UFP = 1,
	TUSB320_TYPE_DFP = 2,
	TUSB320_TYPE_DRP = 3,
};

enum tusb_cc_type{
	CC_DEFAULT = 0x00,
	CC_AUDIO = 0x08,
	CC_AUDIO_CHARGED_ACCESSORY = 0x0A,
	CC_HOST = 0x0C,//CC_DFP_DEBUG_ACCESSORY
	CC_DEVICE = 0x0E,//CC_UFP_DEBUG_ACCESSORY
};

struct tusb320_platform_data{
	void (*audio_cb)(bool attach);
	void (*debug_cb)(bool attach);
	void (*power_acc_cb)(bool attach);
	void (*source_cb)(bool attach, int bc_lvl);
	void (*sink_cb)(bool attach);
};


#endif

