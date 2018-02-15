/*
 * nubia_disp_preference.c - nubia lcd display color enhancement and temperature setting
 *	      Linux kernel modules for mdss
 *
 * Copyright (c) 2015 nubia <nubia@nubia.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Supports NUBIA lcd display color enhancement and color temperature setting
 */

/*------------------------------ header file --------------------------------*/
#include "nubia_disp_preference.h"
#include <linux/delay.h>

/*------------------------------- variables ---------------------------------*/
static struct kobject *enhance_kobj = NULL;
static struct mdss_dsi_ctrl_pdata *nubia_mdss_dsi_ctrl = NULL;
static int boot_flag = 0;

static unsigned int old_cabc = 25;
static uint32_t brightness = 1;
extern int cabc_panel_state;

static struct nubia_disp_type nubia_disp_val = {
	.en_cabc = 1,
	.cabc = CABC_OFF,
	.en_saturation = 1,
#if defined( CONFIG_NUBIA_LCD_ALPM_MODE )
	.en_alpm = 0,
#endif
	.debug_type = 0,//
	.saturation = SATURATION_SOFT,
	.en_colortmp = 1,
	.colortmp =  COLORTMP_NORMAL,
};

/* debug type support, be carefull to check lcd_debug_get_cmds() */
char *debug_type_list[] =
{
	"on",
	"off",
#if defined( CONFIG_NUBIA_LCD_ALPM_MODE )
	"alpm_on",
	"alpm_off",
#endif
};

extern int nubia_mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds);

struct mdp_pcc_cfg_data pcc_cfg_warm = {
	.block = 0x10,
	.ops = 0x5,
	{
		.c = 0,
		.r = 0x8000,
		.g = 0,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0x8000,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0,
		.b = 0x8000,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
};
struct mdp_pcc_cfg_data pcc_cfg_off = {
        .block = 0x10,
        .ops = 0x5,
        {
                .c = 0,
                .r = 0x8000,
                .g = 0,
                .b = 0,
                .rr = 0,
                .gg = 0,
                .bb = 0,
                .rg = 0,
                .gb = 0,
                .rb = 0,
                .rgb_0 = 0,
                .rgb_1 = 0
        },
        {
                .c = 0,
                .r = 0,
                .g = 0x8000,
                .b = 0,
                .rr = 0,
                .gg = 0,
                .bb = 0,
                .rg = 0,
                .gb = 0,
                .rb = 0,
                .rgb_0 = 0,
                .rgb_1 = 0
        },
        {
                .c = 0,
                .r = 0,
                .g = 0,
                .b = 0x8000,
                .rr = 0,
                .gg = 0,
                .bb = 0,
                .rg = 0,
                .gb = 0,
                .rb = 0,
                .rgb_0 = 0,
                .rgb_1 = 0
        },
};
struct mdp_pcc_cfg_data pcc_cfg_natural = {
	.block = 0x10,
	.ops = 0x5,
	{
		.c = 0,
		.r = 0x8000,
		.g = 0,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0x8000,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0,
		.b = 0x8000,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
};

struct mdp_pcc_cfg_data pcc_cfg_cool = {
	.block = 0x10,
	.ops = 0x5,
	{
		.c = 0,
		.r = 0x8000,
		.g = 0,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0x8000,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0,
		.b = 0x8000,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
};

#ifdef CONFIG_NUBIA_LCD_COLORTMP_CONTINUOUS
struct mdp_pcc_cfg_data nubia_mdp_pcc_cfg_adjustable = {
	.block = 0x10,
	.ops = 0x5,
	{
		.c = 0,
		.r = 0x8000,
		.g = 0,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0x8000,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0,
		.b = 0x8000,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
};
#endif

#if NUBIA_DISP_COLORTMP_DEBUG
static struct mdp_pcc_cfg_data nubia_colortmp_debug = {
	.block = 0x10,
	.ops = 0x5,
	{
	  .c = 0,
	  .r = 0x8000,
	  .g = 0,
	  .b = 0,
	  .rr = 0,
	  .gg = 0,
	  .bb = 0,
	  .rg = 0,
	  .gb = 0,
	  .rb = 0,
	  .rgb_0 = 0,
	  .rgb_1 = 0
	},
	{
	  .c = 0,
	  .r = 0,
	  .g = 0x8000,
	  .b = 0,
	  .rr = 0,
	  .gg = 0,
	  .bb = 0,
	  .rg = 0,
	  .gb = 0,
	  .rb = 0,
	  .rgb_0 = 0,
	  .rgb_1 = 0
	},
	{
	  .c = 0,
	  .r = 0,
	  .g = 0,
	  .b = 0x8000,
	  .rr = 0,
	  .gg = 0,
	  .bb = 0,
	  .rg = 0,
	  .gb = 0,
	  .rb = 0,
	  .rgb_0 = 0,
	  .rgb_1 = 0
	},
};
#endif

static int nubia_set_ce_cabc(int sat_val, int cabc_val)
{
	int ret=0;

	switch(sat_val){
		case SATURATION_OFF:
                        switch(cabc_val){
                                case CABC_OFF:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds0.cmd_cnt)
                                                ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds0);
                                        break;
                                case CABC_LEVEL1:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds1.cmd_cnt)
                                                ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds1);
                                        break;
                                case CABC_LEVEL2:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds2.cmd_cnt)
                                                ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds2);
                                        break;
                                case CABC_LEVEL3:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds3.cmd_cnt)
                                                ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds3);
                                        break;
                        }
                        break;
		case SATURATION_SOFT:
			switch(cabc_val){
                                case CABC_OFF:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds4.cmd_cnt)
                                        	ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds4);
                                        break;
                                case CABC_LEVEL1:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds5.cmd_cnt)
	                                   ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds5);
                                        break;
                                case CABC_LEVEL2:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds6.cmd_cnt)
                                        	ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds6);
                                        break;
                                case CABC_LEVEL3:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds7.cmd_cnt)
                                        	ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds7);
                                        break;
			}
                     break;
		case SATURATION_STD:
			switch(cabc_val){
                                case CABC_OFF:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds8.cmd_cnt)
                                        	ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds8);
                                        break;
                                case CABC_LEVEL1:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds9.cmd_cnt)
                                        	ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds9);
                                        break;
                                case CABC_LEVEL2:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds10.cmd_cnt)
                                        	ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds10);
                                        break;
                                case CABC_LEVEL3:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds11.cmd_cnt)
                                        	ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds11);
                                        break;
			}
                     break;
		case SATURATION_GLOW:
                        switch(cabc_val){
                                case CABC_OFF:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds12.cmd_cnt)
                                                ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds12);
                                        break;
                                case CABC_LEVEL1:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds13.cmd_cnt)
                                                ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds13);
                                        break;
                                case CABC_LEVEL2:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds14.cmd_cnt)
                                                ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds14);
                                        break;
                                case CABC_LEVEL3:
                                        if (nubia_mdss_dsi_ctrl->ce_cabc_cmds15.cmd_cnt)
                                                ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cabc_cmds15);
                                        break;
                        }
                        break;

	}
	if(ret == 0)
		NUBIA_DISP_DEBUG("success to set sat = %d ,cabc as = %d\n", sat_val, cabc_val);
	else
		NUBIA_DISP_INFO("failed to set sat cabc.\n");

	return ret;
}

static int nubia_set_cabc(int cabc_val)
{
        int ret = 0;
	if (!nubia_disp_val.en_cabc) {
                ret = -1;
                NUBIA_DISP_ERROR("no saturation\n");
                return ret;
        }

	if (!nubia_mdss_dsi_ctrl->panel_data.panel_info.panel_name) {
                ret = -1;
                NUBIA_DISP_ERROR("invalid nubia_mdss_dsi_ctrl\n");
                return ret;
        } else {
                NUBIA_DISP_DEBUG("lcd is %s\n", nubia_mdss_dsi_ctrl->panel_data.panel_info.panel_name);
        }

        switch (cabc_val) {
		case CABC_OFF:
			if (nubia_mdss_dsi_ctrl->cabc_cmds_off.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->cabc_cmds_off);
			break;
		case CABC_LEVEL1:
			if (nubia_mdss_dsi_ctrl->cabc_cmds_level1.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->cabc_cmds_level1);
			break;
		case CABC_LEVEL2:
			if (nubia_mdss_dsi_ctrl->cabc_cmds_level2.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->cabc_cmds_level2);
			break;
		case CABC_LEVEL3:
			if (nubia_mdss_dsi_ctrl->cabc_cmds_level3.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->cabc_cmds_level3);
			break;
        	default:
			if (nubia_mdss_dsi_ctrl->cabc_cmds_off.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->cabc_cmds_off);
			break;
	}
	return ret;
}

static ssize_t lcd_power_on_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
	uint32_t val = 0;
	int ret = 0;
	nubia_disp_val.en_cabc = 1;

	val = CABC_LEVEL1;
	if ((val != CABC_OFF) && (val != CABC_LEVEL1) &&
		(val != CABC_LEVEL2) && (val != CABC_LEVEL3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return snprintf(buf, PAGE_SIZE, "invalid cabc val = %d\n", val);
	}

	NUBIA_DISP_INFO("cabc value = %d\n", val);

	if(!boot_flag){
		return snprintf(buf, PAGE_SIZE, "boot flag\n");
	}

	ret = nubia_set_cabc(val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}
	nubia_set_ce_cabc(val,nubia_disp_val.cabc);
	return snprintf(buf, PAGE_SIZE, "nubia_disp_val.cabc = %d, nubia_disp_val.en_cabc = %d\n", nubia_disp_val.cabc, nubia_disp_val.en_cabc);
}

static ssize_t lcd_power_on_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;
	nubia_disp_val.en_cabc = 1;

	sscanf(buf, "%d", &val);

	if ((val != CABC_OFF) && (val != CABC_LEVEL1) &&
		(val != CABC_LEVEL2) && (val != CABC_LEVEL3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return size;
	}

	NUBIA_DISP_INFO("cabc value = %d\n", val);

	if(!boot_flag){
		return size;
	}

	ret = nubia_set_cabc(val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}
	nubia_set_ce_cabc(val,nubia_disp_val.cabc);
	return size;
}


static ssize_t lcd_power_off_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
	uint32_t val = 0;
	int ret = 0;
	nubia_disp_val.en_cabc = 1;
	if(!nubia_disp_val.en_cabc) {
		NUBIA_DISP_ERROR("no cabc\n");
		return snprintf(buf, PAGE_SIZE, "no cabc\n");
	}

	val = CABC_OFF;

	if ((val != CABC_OFF) && (val != CABC_LEVEL1) &&
		(val != CABC_LEVEL2) && (val != CABC_LEVEL3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return snprintf(buf, PAGE_SIZE, "invalid cabc val = %d\n", val);
        }

	NUBIA_DISP_INFO("cabc value = %d\n", val);

	if(!boot_flag){
		return snprintf(buf, PAGE_SIZE, "boot flag\n");
	}

	ret = nubia_set_cabc(val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}
	nubia_set_ce_cabc(val,nubia_disp_val.cabc);
	nubia_disp_val.en_cabc = 0;
	return snprintf(buf, PAGE_SIZE, "nubia_disp_val.cabc = %d, nubia_disp_val.en_cabc = %d\n", nubia_disp_val.cabc, nubia_disp_val.en_cabc);
}

static ssize_t lcd_power_off_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;
	nubia_disp_val.en_cabc = 1;
	if(!nubia_disp_val.en_cabc) {
		NUBIA_DISP_ERROR("no cabc\n");
		return size;
	}
	val = CABC_OFF;

	if ((val != CABC_OFF) && (val != CABC_LEVEL1) &&
		(val != CABC_LEVEL2) && (val != CABC_LEVEL3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return size;
	}

	NUBIA_DISP_INFO("cabc value = %d\n", val);

	if(!boot_flag){
		return size;
	}

	ret = nubia_set_cabc(val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_INFO("success to set cabc as = %d\n", val);
	}
	nubia_set_ce_cabc(val,nubia_disp_val.cabc);
	nubia_disp_val.en_cabc = 0;
	return size;
}

static ssize_t cabc_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
       if (nubia_disp_val.en_cabc)
                return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.cabc);
        else
                return snprintf(buf, PAGE_SIZE, "NULL\n");

}

int  brightness_cabc_set(unsigned int bright)
{
        int ret = 0;
	//get brightness
	if(bright == 0){
		return 0;
	}
	brightness = bright;

	if(!nubia_disp_val.en_cabc) {
                NUBIA_DISP_ERROR("no cabc\n");
                return 0;
        }

	//cabc setting
	if(brightness > nubia_mdss_dsi_ctrl->nubia_mdss_dsi_cabc_low_bl){
		//open cabc
		if(old_cabc == nubia_disp_val.cabc){
                        return 0;
		}
		ret = nubia_set_cabc(old_cabc);
		ret += nubia_set_ce_cabc(nubia_disp_val.saturation,old_cabc);
		if (ret == 0) {
	                nubia_disp_val.cabc = old_cabc;
	                NUBIA_DISP_DEBUG("success to set cabc as = %d\n", old_cabc);
	        }
	}
	else{
		//close cabc
		if(CABC_OFF == nubia_disp_val.cabc){
                        return 0;
		}
		ret = nubia_set_cabc(CABC_OFF);
		ret += nubia_set_ce_cabc(nubia_disp_val.saturation,CABC_OFF);
		if (ret == 0) {
	                nubia_disp_val.cabc = CABC_OFF;
	                NUBIA_DISP_DEBUG("success to set cabc as = %d\n", CABC_OFF);
	        }
	}

	return ret;
}

static ssize_t cabc_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;
	if(!nubia_disp_val.en_cabc) {
		NUBIA_DISP_ERROR("no cabc\n");
		return size;
	}

	if (cabc_panel_state == 0) {
		NUBIA_DISP_ERROR("cabc panel not ready !\n");
		return size;
	}

	sscanf(buf, "%d", &val);

	if ((val != CABC_OFF) && (val != CABC_LEVEL1) &&
		(val != CABC_LEVEL2) && (val != CABC_LEVEL3)) {
		NUBIA_DISP_ERROR("invalid cabc val = %d\n", val);
		return size;
	}

	NUBIA_DISP_DEBUG("cabc value = %d\n", val);

	//save previous cabc
	old_cabc = val ;
	NUBIA_DISP_DEBUG("[CABC]:%d,cabc_store:%d\n", __LINE__,old_cabc);

	if(!boot_flag){
		return size;
	}

	if(brightness <= nubia_mdss_dsi_ctrl->nubia_mdss_dsi_cabc_low_bl){
		val = CABC_OFF;
	}

	if(val == nubia_disp_val.cabc){
		return size;
	}
	ret = nubia_set_cabc(val);
	ret += nubia_set_ce_cabc(nubia_disp_val.saturation,val);
	if (ret == 0) {
		nubia_disp_val.cabc = val;
		NUBIA_DISP_ERROR("success to set cabc as = %d\n", val);
	}

	return size;
}

#ifndef CONFIG_NUBIA_LCD_COLORTMP_QDCM
static int nubia_set_saturation(int sat_val)
{
	int ret = 0;

	if (!nubia_disp_val.en_saturation) {
		ret = -1;
		NUBIA_DISP_ERROR("no saturation\n");
		return ret;
	}

	if (!nubia_mdss_dsi_ctrl->panel_data.panel_info.panel_name) {
		ret = -1;
		NUBIA_DISP_ERROR("invalid nubia_mdss_dsi_ctrl\n");
		return ret;
	} else {
		NUBIA_DISP_DEBUG("lcd is %s\n", nubia_mdss_dsi_ctrl->panel_data.panel_info.panel_name);
	}

	switch (sat_val) {
		NUBIA_DISP_DEBUG("sat val is %d\n", sat_val);
		case SATURATION_OFF:
			break;
 		case SATURATION_SOFT:
			if (nubia_mdss_dsi_ctrl->ce_cmds_soft.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cmds_soft);
			break;
		case SATURATION_STD:
			if (nubia_mdss_dsi_ctrl->ce_cmds_std.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cmds_std);
			break;
		case SATURATION_GLOW:
			if (nubia_mdss_dsi_ctrl->ce_cmds_glow.cmd_cnt)
				ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->ce_cmds_glow);
			break;
		default:
 			break;
 		}
	return ret;
}

static ssize_t saturation_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	if (nubia_disp_val.en_saturation)
		return snprintf(buf, PAGE_SIZE, "%d\n",	nubia_disp_val.saturation);
	else
		return snprintf(buf, PAGE_SIZE, "NULL\n");
}

static ssize_t saturation_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;
	int ret = 0;

	if(!nubia_disp_val.en_saturation) {
		NUBIA_DISP_ERROR("no saturation\n");
		return size;
	}

	sscanf(buf, "%d", &val);

	if ((val != SATURATION_OFF) && (val != SATURATION_SOFT) && (val != SATURATION_STD) &&
		(val != SATURATION_GLOW)) {
		NUBIA_DISP_ERROR("invalid saturation val = %d\n", val);
		return size;
	}

	NUBIA_DISP_DEBUG("saturation value = %d\n", val);

	if(!boot_flag){
		return size;
	}

	ret = nubia_set_saturation(val);
	ret += nubia_set_ce_cabc(val,nubia_disp_val.cabc);
	if (ret == 0) {
		nubia_disp_val.saturation = val;
		NUBIA_DISP_ERROR("success to set saturation as = %d\n", val);
	}
	return size;
}

#ifdef CONFIG_NUBIA_LCD_COLORTMP_CONTINUOUS
static int nubia_set_colortmp(void)
{
	int ret = 0;

	if ((!nubia_disp_val.en_colortmp) || (NULL == nubia_mdss_dsi_ctrl)) {
		ret = -1;
		NUBIA_DISP_ERROR("invalid param\n");
		return ret;
	}

	if (!nubia_mdss_dsi_ctrl->panel_data.panel_info.panel_name) {
		ret = -1;
		NUBIA_DISP_DEBUG("invalid nubia_mdss_dsi_ctrl\n");
		return ret;
	} else {
		NUBIA_DISP_DEBUG("lcd is %s\n", nubia_mdss_dsi_ctrl->panel_data.panel_info.panel_name);
	}

	nubia_mdss_pcc_config(&nubia_mdp_pcc_cfg_adjustable);

	return ret;
}
#else
static int nubia_set_colortmp(int ctmp_val)
{
	int ret = 0;

	if ((!nubia_disp_val.en_colortmp) || (NULL == nubia_mdss_dsi_ctrl)) {
		ret = -1;
		NUBIA_DISP_ERROR("invalid param\n");
		return ret;
	}

	if (!nubia_mdss_dsi_ctrl->panel_data.panel_info.panel_name) {
		ret = -1;
		NUBIA_DISP_DEBUG("invalid nubia_mdss_dsi_ctrl\n");
		return ret;
	} else {
		NUBIA_DISP_DEBUG("lcd is %s\n", nubia_mdss_dsi_ctrl->panel_data.panel_info.panel_name);
	}

	NUBIA_DISP_DEBUG("pcc_cfg_warm.r.r = %x",pcc_cfg_warm.r.r);
	NUBIA_DISP_DEBUG("pcc_cfg_warm.g.g = %x",pcc_cfg_warm.g.g);
	NUBIA_DISP_DEBUG("pcc_cfg_warm.b.b = %x",pcc_cfg_warm.b.b);

	NUBIA_DISP_DEBUG("pcc_cfg_natural.r.r = %x",pcc_cfg_natural.r.r);
	NUBIA_DISP_DEBUG("pcc_cfg_natural.g.g = %x",pcc_cfg_natural.g.g);
	NUBIA_DISP_DEBUG("pcc_cfg_natural.b.b = %x",pcc_cfg_natural.b.b);

	NUBIA_DISP_DEBUG("pcc_cfg_cool.r.r = %x",pcc_cfg_cool.r.r);
	NUBIA_DISP_DEBUG("pcc_cfg_cool.g.g = %x",pcc_cfg_cool.g.g);
	NUBIA_DISP_DEBUG("pcc_cfg_cool.b.b = %x",pcc_cfg_cool.b.b);

	switch (ctmp_val) {
		case COLORTMP_WARM:
			ret = nubia_mdss_pcc_config(&pcc_cfg_warm);
	       		break;
       		case COLORTMP_NORMAL:
			ret = nubia_mdss_pcc_config(&pcc_cfg_natural);
       			break;
	       	case COLORTMP_COOL:
			ret = nubia_mdss_pcc_config(&pcc_cfg_cool);
       			break;
	       	default:
			ret = nubia_mdss_pcc_config(&pcc_cfg_natural);
			break;
		}

	return ret;
}
#endif

static ssize_t colortmp_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	if (nubia_disp_val.en_colortmp)
		return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.colortmp);
	else
		return snprintf(buf, PAGE_SIZE, "NULL\n");
}

#ifdef CONFIG_NUBIA_LCD_COLORTMP_CONTINUOUS
static ssize_t colortmp_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int val = 0, ret = 0;
	u32 blue = 0x10;
	u32 red = 0x10;

	if(!nubia_disp_val.en_colortmp) {
		NUBIA_DISP_ERROR("no colortmp\n");
		return size;
	}

	if(!nubia_mdss_dsi_ctrl->nubia_colortmp_adjustable_blue)
		blue = nubia_mdss_dsi_ctrl->nubia_colortmp_adjustable_blue;

	if(!nubia_mdss_dsi_ctrl->nubia_colortmp_adjustable_red)
		red = nubia_mdss_dsi_ctrl->nubia_colortmp_adjustable_red;

	sscanf(buf, "%d", &val);

	NUBIA_DISP_DEBUG("colortmp val = %d\n", val);

	//std
	if(val == 127) {
		nubia_mdp_pcc_cfg_adjustable.r.r = 0x8000;
		nubia_mdp_pcc_cfg_adjustable.g.g = 0x8000;
		nubia_mdp_pcc_cfg_adjustable.b.b = 0x8000;
	}
	//warm
	else if(val > 127) {
		nubia_mdp_pcc_cfg_adjustable.r.r = 0x8000;
		nubia_mdp_pcc_cfg_adjustable.g.g = 0x8000;
		//0x8000 - blue * (val - 127) / 127
		nubia_mdp_pcc_cfg_adjustable.b.b = 0x8000 -blue * (val - 127);
	}
	//cool
	else if(val < 127) {
		//0x8000 - red * (127 - val) / 127;
		nubia_mdp_pcc_cfg_adjustable.r.r = 0x8000 - red * (127 - val);
		nubia_mdp_pcc_cfg_adjustable.g.g = 0x8000;
		nubia_mdp_pcc_cfg_adjustable.b.b = 0x8000;
	}

	ret = nubia_set_colortmp();
	if (ret == 0) {
		nubia_disp_val.colortmp = val;
		NUBIA_DISP_DEBUG("success to set colortmp as = %d\n", val);
	}

	return size;
}
#else
static ssize_t colortmp_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int val = 0, ret = 0;

	if(!nubia_disp_val.en_colortmp) {
		NUBIA_DISP_ERROR("no colortmp\n");
		return size;
	}

	sscanf(buf, "%d", &val);

	if ((val != COLORTMP_WARM) && (val != COLORTMP_NORMAL)
		&& (val != COLORTMP_COOL)) {
		NUBIA_DISP_ERROR("invalid colortmp val\n");
		return size;
	}

	NUBIA_DISP_DEBUG("colortmp val = %d\n", val);

	ret = nubia_set_colortmp(val);
	if (ret == 0) {
		nubia_disp_val.colortmp = val;
		NUBIA_DISP_DEBUG("success to set colortmp as = %d\n", val);
	}

	return size;
}
#endif
#endif

#if NUBIA_DISP_COLORTMP_DEBUG
static ssize_t colortmp_r_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%4x\n", nubia_colortmp_debug.r.r);
}

static ssize_t colortmp_r_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;

	sscanf(buf, "%x", &val);

	nubia_colortmp_debug.r.r = val;

	return size;
}

static ssize_t colortmp_g_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%4x\n", nubia_colortmp_debug.g.g);
}

static ssize_t colortmp_g_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;

	sscanf(buf, "%x", &val);

	nubia_colortmp_debug.g.g = val;

	return size;
}

static ssize_t colortmp_b_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%4x\n", nubia_colortmp_debug.b.b);
}

static ssize_t colortmp_b_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	uint32_t val = 0;

	sscanf(buf, "%x", &val);

	nubia_colortmp_debug.b.b = val;

	return size;
}

static ssize_t colortmp_debug_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "r = 0x%4x, g = 0x%4x, b = 0x%4x\n",
		nubia_colortmp_debug.r.r, nubia_colortmp_debug.g.g, nubia_colortmp_debug.b.b);
}

static ssize_t colortmp_debug_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int val = 0;

	if(!nubia_disp_val.en_colortmp) {
		NUBIA_DISP_ERROR("no colortmp\n");
	     return size;
	}

	sscanf(buf, "%d", &val);

	NUBIA_DISP_INFO("colortmp debug val = %d\n", val);

	if (val == 1)
		nubia_mdss_pcc_config(&nubia_colortmp_debug);

	return size;
}
#endif

static struct dsi_panel_cmds *lcd_debug_get_cmds( void )
{
       struct dsi_panel_cmds *cur_cmds = NULL;
	switch( nubia_disp_val.debug_type )
	{
		case 0:
			cur_cmds= &nubia_mdss_dsi_ctrl->on_cmds;
		break;
		case 1:
			cur_cmds= &nubia_mdss_dsi_ctrl->off_cmds;
		break;
#if defined( CONFIG_NUBIA_LCD_ALPM_MODE )
		case 2:
			cur_cmds= &nubia_mdss_dsi_ctrl->alpm_cmds_on;
		break;
		case 3:
			cur_cmds= &nubia_mdss_dsi_ctrl->alpm_cmds_off;
		break;
#endif
		default:
		break;
	}
	return cur_cmds;
}

#define COUNT_CH 2000
static ssize_t lcd_debug_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	size_t ret=0;
	int i=0,j=0,k=0;
	char str[3],sntr[COUNT_CH];
	struct dsi_panel_cmds *on_cmds=NULL;
	struct dsi_cmd_desc *dsi_cmd=NULL;
	char *ch=NULL;

	on_cmds = lcd_debug_get_cmds();
	if(on_cmds==NULL){
		pr_err("on_cmds==NULL");
		return ret;
	}

	dsi_cmd=on_cmds->cmds;
	if(dsi_cmd==NULL){
		pr_err("dsi_cmd==NULL\n");
		return ret;
	}

	for(i=0;i<on_cmds->cmd_cnt;i++){
		pr_err("debug dsi_cmd->dchdr.dlen = %x\n",dsi_cmd->dchdr.dlen);

		sprintf(str,"%02x",i);
		if(k<(COUNT_CH-1))
			sntr[k++]=str[0];
		if(k<(COUNT_CH-1))
			sntr[k++]=str[1];
		if(k<(COUNT_CH-1))
			sntr[k++]=' ';
		sprintf(str,"%02x",dsi_cmd->dchdr.dlen);
		if(k<(COUNT_CH-1))
			sntr[k++]=str[0];
		if(k<(COUNT_CH-1))
			sntr[k++]=str[1];
		if(k<(COUNT_CH-1))
			sntr[k++]=' ';

		ch=dsi_cmd->payload;
		for(j=0;j<dsi_cmd->dchdr.dlen;j++){
			if(ch==NULL){
				pr_err("ch==NULL i = %x ,j = %x\n",i,j);
				if(k<COUNT_CH)
					sntr[k]='\0';
				pr_err("%s",sntr);
				ret = snprintf(buf, PAGE_SIZE, sntr);
				return ret;
			}
			sprintf(str,"%02x",*(ch));
			if(k<(COUNT_CH-1))
				sntr[k++]=str[0];
			if(k<(COUNT_CH-1))
				sntr[k++]=str[1];
			if(k<(COUNT_CH-1))
				sntr[k++]=' ';
			ch++;
		}
		if(k<(COUNT_CH-1))
			sntr[k++]='\n';
		dsi_cmd++;
	}
	pr_err("k = %x\n",k);
	if(k<COUNT_CH)
		sntr[k]='\0';
	pr_err("%s",sntr);
	ret = snprintf(buf, PAGE_SIZE, sntr);
	//no. len cmd ...
	return ret;
}

static ssize_t lcd_debug_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int val[256];
	int len,i;
	struct dsi_panel_cmds *on_cmds=NULL;
	struct dsi_cmd_desc *dsi_cmd=NULL;
	char *ch=NULL;

	on_cmds = lcd_debug_get_cmds();
	if(on_cmds==NULL){
		pr_err("on_cmds==NULL\n");
		return size;
	}
	//no. len cmd ...
	len = sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",&val[0], &val[1], &val[2], &val[3], &val[4], &val[5], &val[6], &val[7], &val[8], &val[9],&val[10], &val[11], &val[12], &val[13], &val[14], &val[15], &val[16], &val[17], &val[18], &val[19],&val[20], &val[21], &val[22], &val[23], &val[24], &val[25], &val[26], &val[27], &val[28], &val[29],&val[30], &val[31], &val[32], &val[33], &val[34], &val[35], &val[36], &val[37], &val[38], &val[39],&val[40], &val[41], &val[42], &val[43], &val[44], &val[45], &val[46], &val[47], &val[48], &val[49]);

	pr_err("debug len %x\n",len);
	for(i=0;i<len;i++)
		pr_err("debug %x\n",val[i]);

	if(val[0]>=on_cmds->cmd_cnt){
		pr_err("val[0]>=on_cmds->cmd_cnt\n");
		return size;
	}

	dsi_cmd=on_cmds->cmds;
	if(dsi_cmd==NULL){
		pr_err("dsi_cmd==NULL\n");
		return size;
	}

	for(i=0;i<val[0];i++)
		dsi_cmd++;
	if(dsi_cmd==NULL){
		pr_err("dsi_cmd==NULL\n");
		return size;
	}

	if(val[1]>dsi_cmd->dchdr.dlen){
		pr_err("val[1]>dsi_cmd->dchdr.dlen");
		return size;
	}

	ch=dsi_cmd->payload;
	for(i=0;i<val[1] && i<len-2;i++){
		if(ch==NULL){
			pr_err("ch==NULL,i = %x\n",i);
			return size;
		}
		*ch=val[i+2];
		ch++;
	}

	return size;
}

static ssize_t lcd_debug_type_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", debug_type_list[nubia_disp_val.debug_type]);
}

static ssize_t lcd_debug_type_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int i;
       char bbuf[32];
       strncpy(bbuf, buf, 32);
       i = strlen(bbuf);
       if( bbuf[i-1] == '\n' ) bbuf[i-1] = 0;
	for( i = 0; i< sizeof(debug_type_list)/sizeof(debug_type_list[0]); i++ )
       {
		if( !strcmp(bbuf, debug_type_list[i]) )
			break;
       }
	if( i !=  sizeof(debug_type_list)/sizeof(debug_type_list[0]) )
		nubia_disp_val.debug_type = i;
       pr_err("%s after store debug_type=%d\n", __func__, nubia_disp_val.debug_type);
       return size;
}

#if defined( CONFIG_NUBIA_LCD_ALPM_MODE )
static ssize_t lcd_alpm_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", nubia_disp_val.en_alpm);
}

static ssize_t lcd_alpm_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t size)
{
	int val = -1;
	int ret = -2;
       if( ( 1 == sscanf(buf, "%d", &val) )  && ( (val = !!val) !=  nubia_disp_val.en_alpm) )
       {
		if ( val  && nubia_mdss_dsi_ctrl->alpm_cmds_on.cmd_cnt )
		{
			ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->alpm_cmds_on);
			if ( !ret )
			    nubia_disp_val.en_alpm = val;
		}
              else if( !val && nubia_mdss_dsi_ctrl->alpm_cmds_off.cmd_cnt )
		{
			ret = nubia_mdss_dsi_panel_cmds_send(nubia_mdss_dsi_ctrl, &nubia_mdss_dsi_ctrl->alpm_cmds_off);
			if ( !ret )
			    nubia_disp_val.en_alpm = val;
		}
        }
        pr_err("%s ret=%d, store val=%d, after store state=%d\n", __func__, ret, val, nubia_disp_val.en_alpm);
        return size;
}
#endif

static struct kobj_attribute lcd_disp_attrs[] = {
#ifndef CONFIG_NUBIA_LCD_COLORTMP_QDCM
	__ATTR(saturation,      0664, saturation_show,     saturation_store),
	__ATTR(colortmp,        0664, colortmp_show,       colortmp_store),
#endif
//donot use ACL node, just return NULL if no cabc support
#if defined ( CONFIG_NUBIA_LCD_USE_ACL )
	__ATTR(acl,        0664, cabc_show,       cabc_store),
#else
	__ATTR(cabc,        0664, cabc_show,       cabc_store),
#endif
#if NUBIA_DISP_COLORTMP_DEBUG
	__ATTR(colortmp_r,      0664, colortmp_r_show,     colortmp_r_store),
	__ATTR(colortmp_g,      0664, colortmp_g_show,     colortmp_g_store),
	__ATTR(colortmp_b,      0664, colortmp_b_show,     colortmp_b_store),
	__ATTR(colortmp_debug,  0664, colortmp_debug_show, colortmp_debug_store),
#endif
#if defined( CONFIG_NUBIA_LCD_ALPM_MODE )
	__ATTR(alpm,  0664, lcd_alpm_show, lcd_alpm_store),
#endif
	__ATTR(lcd_debug,  0664, lcd_debug_show, lcd_debug_store),
	__ATTR(lcd_debug_type,  0664, lcd_debug_type_show, lcd_debug_type_store),
	__ATTR(lcd_power_on,        0664, lcd_power_on_show,       lcd_power_on_store),
	__ATTR(lcd_power_off,        0664, lcd_power_off_show,       lcd_power_off_store),
};

void nubia_feature_check( void )
{

/* FIX ME: too many things to check, we only check off cmd */
    if ( \
        nubia_mdss_dsi_ctrl->cabc_cmds_off.cmd_cnt  || \
        nubia_mdss_dsi_ctrl->ce_cabc_cmds0.cmd_cnt
    )
        nubia_disp_val.en_cabc = 1;
    else
        nubia_disp_val.en_cabc = 0;

    if ( \
        nubia_mdss_dsi_ctrl->ce_cmds_soft.cmd_cnt ||  \
        nubia_mdss_dsi_ctrl->ce_cabc_cmds0.cmd_cnt
    )
        nubia_disp_val.en_saturation = 1;
    else
        nubia_disp_val.en_saturation = 0;

    nubia_disp_val.en_colortmp = 1;
}

void nubia_set_dsi_ctrl(struct mdss_dsi_ctrl_pdata * ctrl)
{
	nubia_mdss_dsi_ctrl = ctrl;
	nubia_feature_check();
}
EXPORT_SYMBOL(nubia_set_dsi_ctrl);

void nubia_disp_preference(void)
{
	int ret = 0;
	NUBIA_DISP_INFO("nubia_disp_preference start\n");

#ifndef CONFIG_NUBIA_LCD_COLORTMP_QDCM
	ret = nubia_set_saturation(nubia_disp_val.saturation);
#endif
	ret = nubia_set_cabc(nubia_disp_val.cabc);
	ret = nubia_set_ce_cabc(nubia_disp_val.saturation, nubia_disp_val.cabc);
	if (ret == 0)
		NUBIA_DISP_INFO("success to set sat = %d ,cabc as = %d\n",
						nubia_disp_val.saturation, nubia_disp_val.cabc);

	if (!boot_flag){
		boot_flag = 1;
		if(nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_warm.defult == 0){
			pcc_cfg_warm.r.r = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_warm.red;
			pcc_cfg_warm.g.g = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_warm.green;
			pcc_cfg_warm.b.b = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_warm.blue;
		}
		if(nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_natural.defult == 0){
			pcc_cfg_natural.r.r = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_natural.red;
			pcc_cfg_natural.g.g = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_natural.green;
			pcc_cfg_natural.b.b = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_natural.blue;
		}
		if(nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_cool.defult == 0){
			pcc_cfg_cool.r.r = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_cool.red;
			pcc_cfg_cool.g.g = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_cool.green;
			pcc_cfg_cool.b.b = nubia_mdss_dsi_ctrl->nubia_mdp_colortmp_cool.blue;
		}
	}
#ifndef CONFIG_NUBIA_LCD_COLORTMP_QDCM
#ifdef CONFIG_NUBIA_LCD_COLORTMP_CONTINUOUS
	ret = nubia_set_colortmp();
#else
	ret = nubia_set_colortmp(nubia_disp_val.colortmp);
#endif
#endif
	if (ret == 0)
                NUBIA_DISP_INFO("success to set colortmp as = %d\n", nubia_disp_val.colortmp);
}
EXPORT_SYMBOL(nubia_disp_preference);

static int __init nubia_disp_preference_init(void)
{
	int retval = 0;
	int attr_count = 0;

	enhance_kobj = kobject_create_and_add("lcd_enhance", kernel_kobj);

	if (!enhance_kobj) {
		NUBIA_DISP_ERROR("failed to create and add kobject\n");
		return -ENOMEM;
	}

	/* Create attribute files associated with this kobject */
	for (attr_count = 0; attr_count < ARRAY_SIZE(lcd_disp_attrs); attr_count++) {
		retval = sysfs_create_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);
		if (retval < 0) {
			NUBIA_DISP_ERROR("failed to create sysfs attributes\n");
			goto err_sys_creat;
		}
	}

	return retval;

err_sys_creat:
	for (--attr_count; attr_count >= 0; attr_count--)
		sysfs_remove_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);

	kobject_put(enhance_kobj);
	return retval;
}

static void __exit nubia_disp_preference_exit(void)
{
	int attr_count = 0;

	for (attr_count = 0; attr_count < ARRAY_SIZE(lcd_disp_attrs); attr_count++)
		sysfs_remove_file(enhance_kobj, &lcd_disp_attrs[attr_count].attr);

	kobject_put(enhance_kobj);
	nubia_mdss_dsi_ctrl = NULL;
}

MODULE_AUTHOR("NUBIA LCD Driver Team Software");
MODULE_DESCRIPTION("NUBIA LCD DISPLAY Color Saturation and Temperature Setting");
MODULE_LICENSE("GPL");
module_init(nubia_disp_preference_init);
module_exit(nubia_disp_preference_exit);
