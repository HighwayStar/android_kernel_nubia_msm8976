
/*
 *  pa22_ps.h - Linux kernel modules for proximity sensor
 *
 *  Copyright (c) 2013, All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __PA224_PS_H__
#define __PA224_PS_H__


/*pa224 als/ps Default*/
#define PA24_PS_POLL_DELAY		100
#define PA24_PS_ENABLE_DELAY	30

#define PA24_PS_TH_HIGH		40
#define PA24_PS_TH_LOW		25
#define PA24_PS_TH_MIN		0	// Minimun value
#define PA24_PS_TH_MAX		255     // 8 bit MAX

#define PA24_PS_NEAR_DISTANCE	3       //Near distance 0 cm
#define PA24_PS_FAR_DISTANCE	10       //Far distance 1 cm
#define PA24_PS_UNKOWN_DISTANCE	0       //Unkown distance 1 cm

#define PA24_PS_UNCOVER_MIN_SAFE 6 	// to decide if sensor need naked calibration
#define PA24_PS_UNCOVER_MAX_SAFE 100   // provided to decide if sensor is qualified
#define PA24_PS_OFFSET_DEFAULT	 5 	// for X-talk cannceling
#define PA24_PS_OFFSET_MAX		150

#define PA24_MIN_NEAR_CNT	15  // min 3cm count
#define PA24_MAX_NEAR_CNT	75  // max 3cm count
#define PA24_MANUAL_OFFSET	2

#define PA24_LED_CURR		6 	/*4:15mA | 5:12mA | 6:10mA | 7:7mA*/
#define PA24_PS_PRST		1	// 0:1point 1:2points 2:4points 3:8points (for INT)
#define PA24_PS_SET				1	// 0:No interrupt 1:PS interrupt only
#define PA24_PS_MODE		0	// 0:OFFSET 1:NORMAL
#define PA24_INT_TYPE		0 	// 0:Window type 1:Hysteresis type for Auto Clear flag , if ALS use interrupt mode,should use windows mode
#define PA24_PS_PERIOD		0	/* 0:6.25 ms | 1:12.5 ms | 2:25 ms | 3:50 ms | 4:100 ms | 5:200 ms | 6:400 ms | 7:800 ms */

#define PS_INT			0 	//gpio_to_irq(xxxx) GPIO Define
#define PS_POLLING		0	// 0:INT Mode 1:Polling Mode

#define saturation_delay    100
#define sequence_dealy      15
#define OIL_EFFECT          35
#define ps_ary_size         4

/*pa22 ps sensor register map*/
#define REG_CFG0 			0X00  	// PS_ON(D1)
#define REG_CFG1 			0X01  	// LED_CURR(D6-4),PS_PRST(D3-2)
#define REG_CFG2 			0X02  	// PS_MODE(D6),CLEAR(D4),INT_SET(D3-2),PS_INT(D1)
#define REG_CFG3			0X03  	// INT_TYPE(D6),PS_PERIOD(D5-3)
#define REG_PS_TL			0X08  	// PS Threshold Low
#define REG_PS_TH			0X0A  	// PS Threshold High
#define REG_PS_DATA			0X0E  	// PS DATA
#define REG_PS_OFFSET		0X10  	// TBD
#define REG_PS_SET			0X11  	// 0x82
#define REG_CFG4			0X12    //0x0C
#define PS_ACTIVE			0x02
#define PS_INT_ACTIVE		0x02
#define forward_step 	15
#define backward_step 	5

#define INPUT_NAME_PS       "proximity"
#define PS_CAL_FILE_PATH    "/persist/sensors/xtalk_cal"
#define PS_INT_ACTIVE	 0x02
/* Oil */
#define TXC_ABS(x) (x) >= 0 ? (x):(x)*(-1)
#define TXC_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))

#define TXC_SUM(arr, sum) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	for (i=0; i<size; i++) \
		sum += arr[i]; \
} while (0)

#define TXC_ABS_SUM(arr, sum) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	for (i = 0; i < size; i++) \
		sum += TXC_ABS(arr[i]); \
} while (0)

#define IS_CLOSE(arr, close) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	close = 1; \
	while (i < size && close == 1) { \
		if (arr[i] >= 0) \
			i++; \
		else \
			close = 0; \
	} \
}while (0)

#define IS_AWAY(arr, away) \
do { \
	int i = 0; \
	int size = TXC_SIZE(arr); \
	away = 1; \
	while (i < size && away == 1) { \
		if ( arr[i] <= 0) \
			i++; \
		else \
			away = 0; \
	}  \
}while (0)


static void pa22_report_event(struct pa22_ps_data *data);
static void pa22_irq_enable(struct pa22_ps_data *data, bool enable, bool flag_sync);
static int pa22_init_client(struct pa22_ps_data *data);
static int pa22_ps_unregister(struct pa22_platform_device *pdev_node);

#endif
