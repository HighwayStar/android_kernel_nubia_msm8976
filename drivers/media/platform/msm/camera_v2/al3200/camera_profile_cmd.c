/*
 * File: camera_profile_cmd.c
 * Description: Mini ISP sample codes
 *
 * (C)Copyright altek Corporation 2013
 *
 *  2013/10/14; Bruce Chung; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */

/******Include File******/

#include <linux/string.h>

#include "include/mtype.h"
#include "include/error.h"
#include "include/miniisp.h"
#include "include/isp_camera_cmd.h"
#include "include/ispctrl_if_master.h"
#include "include/error/ispctrl_if_master_err.h"
#include "include/ispctrl_if_master_local.h"

/******Private Constant Definition******/

/******Private Type Declaration******/

/******Private Function Prototype******/

/******Private Global Variable******/


static struct isp_cmd_basic_para mast_basic_para;
static struct isp_cmd_system_info mast_system_info;
static struct isp_cmd_sensor_info mast_sensors_info[SENSOR_TYPEMAX];
static struct isp_cmd_tx_info mast_tx_stream_on_off[SENSOR_TYPEMAX];

/******Public Function******/

/*
 *\brief Camera profile parameters init
 *\return None
 */
void isp_mast_camera_profile_para_init(void)
{
	/*Reset Camera profile parameters*/
	memset(&mast_basic_para, 0x0, sizeof(struct isp_cmd_basic_para));
	memset(&mast_system_info, 0x0, sizeof(struct isp_cmd_system_info));
	memset(&mast_sensors_info, 0x0,
		sizeof(struct isp_cmd_sensor_info)*SENSOR_TYPEMAX);
	memset(&mast_tx_stream_on_off, 0x0,
		sizeof(struct isp_cmd_tx_info)*SENSOR_TYPEMAX);
}

/*
 *\brief Get System Info
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_system_info(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 system_info_size;

	/* Set System Info Size*/
	system_info_size = sizeof(struct isp_cmd_system_info);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, NULL, 0);
	if (ERR_SUCCESS != err)
		goto mast_camera_profile_cmd_get_system_info_end;


	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata,
			(u8 *)&mast_system_info, system_info_size, true);
	if (ERR_SUCCESS != err)
		goto mast_camera_profile_cmd_get_system_info_end;

	/* copy to usr defined target addr*/
	memcpy(param, &mast_system_info, system_info_size);

mast_camera_profile_cmd_get_system_info_end:
	return err;
}


/*
 *\brief Set Basic Parameters
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_basic_parameters(
						struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u32 basic_para_size;

	/*Set Basic Parameter Size*/
	basic_para_size = sizeof(struct isp_cmd_basic_para);

	/*copy to Basic Parameter's structure addr*/
	memcpy(&mast_basic_para, param, basic_para_size);

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			(u8 *)&mast_basic_para, basic_para_size);

	return err;
}

/*
 *\brief Get Basic Parameters
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_basic_parameters(
						struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 basic_para_size;

	/* Set Basic Parameter Size*/
	basic_para_size = sizeof(struct isp_cmd_basic_para);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, NULL, 0);
	if (ERR_SUCCESS != err)
		goto mast_camera_profile_cmd_get_basic_parameters_end;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata,
			(u8 *)&mast_basic_para, basic_para_size, true);
	if (ERR_SUCCESS != err)
		goto mast_camera_profile_cmd_get_basic_parameters_end;

	/* copy to usr defined target addr*/
	memcpy(param, &mast_basic_para, basic_para_size);

mast_camera_profile_cmd_get_basic_parameters_end:
	return err;
}

/*
 *\brief Set Sensor Mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_sensor_mode(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	para_size = 7;

	/*copy to Basic Parameter's structure addr*/
	/*memcpy(&mast_sensors_info, param, para_size);*/

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
				param, para_size);

	return err;
}

/*
 *\brief Get Sensor Mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_sensor_mode(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	para_size = sizeof(struct isp_cmd_sensor_info)*SENSOR_TYPEMAX;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, NULL, 0);
	if (ERR_SUCCESS != err)
		goto mast_camera_profile_cmd_get_sensor_mode_end;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata,
			(u8 *)&mast_sensors_info, para_size, true);
	if (ERR_SUCCESS != err)
		goto mast_camera_profile_cmd_get_sensor_mode_end;

	/* copy to usr defined target addr*/
	memcpy(param, &mast_sensors_info, para_size);

mast_camera_profile_cmd_get_sensor_mode_end:
	return err;
}

/*
 *\brief Set Output Format
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_output_format(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	para_size = 3;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, param,
						para_size);
	return err;
}



/*
 *\brief Preview stream on
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_preview_stream_on_off(
						struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	para_size = sizeof(struct isp_cmd_tx_info)*SENSOR_TYPEMAX;

	/* copy to Basic Parameter's structure addr*/
	memcpy(mast_tx_stream_on_off, param, para_size);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			(u8 *)mast_tx_stream_on_off, para_size);
	return err;
}


/************************** End Of File *******************************/
