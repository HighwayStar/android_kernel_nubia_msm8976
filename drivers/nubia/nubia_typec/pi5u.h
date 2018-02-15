/*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __PI5U_H__
#define __PI5U_H__

typedef unsigned char BYTE;

enum pi5u_type{
	PI5U_TYPE_UFP = 0,
	PI5U_TYPE_DFP = 1,
	PI5U_TYPE_DRP = 2,
};

enum pi5u_port_status{
	PI5U_PORT_STATUS_STANDBY = 0,
	PI5U_PORT_STATUS_DEVICE = 1,
	PI5U_PORT_STATUS_HOST = 2,
	PI5U_PORT_STATUS_AUDIO_ACCESSORY = 3,
	PI5U_PORT_STATUS_DEBUG_ACCESSORY = 4,
	PI5U_PORT_STATUS_DEVICE_WITH_ACTIVE_TABLE = 5,
};

struct pi5u_platform_data{
	void (*audio_cb)(bool attach);
	void (*debug_cb)(bool attach);
	void (*power_acc_cb)(bool attach);
	void (*source_cb)(bool attach, int bc_lvl);
	void (*sink_cb)(bool attach);
};


#endif

