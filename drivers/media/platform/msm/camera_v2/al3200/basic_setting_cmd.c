/*
 * File: basic_setting_cmd.c
 * Description: Mini ISP sample codes
 *
 * (C)Copyright altek Corporation 2013
 *
 *  2013/10/14; Aaron Chuang; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */

/******Include File******/

#include <linux/string.h>

#include "include/isp_camera_cmd.h"
#include "include/ispctrl_if_master.h"
#include "include/error/ispctrl_if_master_err.h"
#include "include/miniisp.h"
#include "include/ispctrl_if_master_local.h"

/******Private Constant Definition******/

/******Private Type Declaration******/

/******Private Function Prototype******/

/******Private Global Variable******/
static struct isp_cmd_ae_zone_info ae_zone_cfg[AEZONECFG_MAXGROUP];
static struct isp_cmd_af_zone_info af_zone_cfg[AFZONECFG_MAXGROUP];


/******Public Global Variable******/

/******Public Function******/

/*
 *\brief Camera profile parameters init
 *\return None
 */
void mast_basic_setting_para_init(void)
{
	/*Reset Camera profile parameters */
	memset(&ae_zone_cfg, 0x0,
		sizeof(struct isp_cmd_ae_zone_info)*AEZONECFG_MAXGROUP);
	memset(&af_zone_cfg, 0x0,
		sizeof(struct isp_cmd_af_zone_info)*AFZONECFG_MAXGROUP);
}

/*
 *\brief Set Depth 3A Info
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_depth_3a_info(struct misp_data *devdata,
							u16 opcode, u8 *param)
{
	/*Error Code */
	errcode err = ERR_SUCCESS;
	u32 para_size = 38;

	/*Send command to slave */
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
					param, para_size);
	return err;
}

/*
 *\brief Set depth input WOI
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_depth_input_woi(struct misp_data *devdata,
							u16 opcode, u8 *param)
{
	/*Error Code */
	errcode err = ERR_SUCCESS;
	u32 para_size = 8;

	/*Send command to slave */
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
					param, para_size);
	return err;
}


/******End Of File******/
