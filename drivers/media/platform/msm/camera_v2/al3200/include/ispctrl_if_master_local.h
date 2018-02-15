/*
 * File: ispctrl_if_master_local.h
 * Description: The structure and API definition ISP Ctrl IF Master Local
 *It,s a header file that define structure and API for ISP Ctrl IF Master Local
 * (C)Copyright altek Corporation 2013
 *
 *  2013/10/14; Aaron Chuang; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */


/**
 * \file	 ispctrl_if_master_local.h
 * \brief	ISP Ctrl IF Master Local and API define
 * \version  0.01
 * \author   Aaron Chuang
 * \date	 2013/09/23
 * \see	  ispctrl_if_master_local.h
 */

#ifndef _ISPCTRLIF_MASTER_LOCAL_H_
#define _ISPCTRLIF_MASTER_LOCAL_H_

/**
 *@addtogroup ispctrl_if_master_local
 *@{
 */


/******Include File******/

#include "mtype.h"
#include "miniisp.h"


/******Public Constant Definition******/
#define ISP_REGISTER_PARA_SIZE  8
#define ISP_REGISTER_VALUE  4


/******Public Function Prototype******/

/*********************** System manage command ***********************/
/**
 *\brief Change mode
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_change_mode(struct misp_data *devdata,
						u16 opcode, u8 *param);

/**
 *\brief Get status of Mode Change
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_status_of_mode_change(
						struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get status of last executed command
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_status_of_last_exec_command(
						struct misp_data *devdata,
						u16 opcode, u8 *param);

/**
 *\brief Get error code CMD
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_error_code_command(struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Set ISP register
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_set_isp_register(struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get ISP register
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_isp_register(struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Set common log level
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param

 *\return Error code
 */
errcode mast_sys_manage_cmd_set_comomn_log_level(struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get chip test report
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_chip_test_report(struct misp_data *devdata,
								u16 opcode,
								u8 *param);


/**
 *\brief Master Camera profile parameters init
 *\return None
 */


/**
 *\brief Get IRQ status
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_irq_status(struct misp_data *devdata,
								u16 opcode,
								u8 *param);


/**
 *\brief Get Polling Command Status
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_polling_command_status(
						struct misp_data *devdata,
								u16 opcode,
								u8 *param);

void isp_mast_camera_profile_para_init(void);

/*********************** Camera profile command ***********************/
/**
 *\brief Get System Info
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_system_info(struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Set Basic Parameters
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_basic_parameters(
						struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get Basic Parameters
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_basic_parameters(
						struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Set Sensor Mode
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_sensor_mode(struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get Sensor Mode
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_sensor_mode(struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Set Output Format
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_output_format(struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Preview stream on
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_preview_stream_on_off(
						struct misp_data *devdata,
								u16 opcode,
								u8 *param);

/* Bulk data command*/

/**
 *\brief Write Boot Code
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], boot file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_boot_code(struct misp_data *devdata,
								u8 *param,
							struct file *filp);


/**
 *\brief Write Basic Code
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], basic file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_basic_code(struct misp_data *devdata,
								u8 *param,
							struct file *filp);

/**
 *\brief Write Advanced Code
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], advance file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_advanced_code(struct misp_data *devdata,
								u8 *param,
							struct file *filp);

/**
 *\brief Write Calibration Data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], sc table file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_calibration_data(struct misp_data *devdata,
								u8 *param,
							struct file *filp);

/**
 *\brief Read Calibration Data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_read_memory_data(struct misp_data *devdata,
							u8 *param);

/**
 *\brief Read common log data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\return Error code
 */
errcode bulk_data_cmd_read_common_log(struct misp_data *devdata,
							u8 *param);

/**
 *\brief Camera profile parameters init
 *\return None
 */
void mast_basic_setting_para_init(void);

/*********************** Basic setting command ************************/
/**
 *\brief Set Depth 3A Info
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_depth_3a_info(struct misp_data *devdata,
						u16 opcode,
						u8 *param);

/**
 *\brief Set depth input WOI
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_depth_input_woi(struct misp_data *devdata,
						u16 opcode,
						u8 *param);

/*********************** operation command ***********************/
/**
 *\brief Mini ISP open
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_operation_cmd_miniisp_open(struct misp_data *devdata,
						u16 opcode, u8 *param);


/****************************** End of File *********************************/

/**
 *@}
 */

#endif /* _ISPCTRLIF_MASTER_LOCAL_H_*/
