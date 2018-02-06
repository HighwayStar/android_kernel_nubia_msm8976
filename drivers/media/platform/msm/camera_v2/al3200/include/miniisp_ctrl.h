/*
 * File: miniisp_ctrl.h
 * Description: The structure and API definition mini ISP Ctrl
 * It is a header file that define structure and API for mini ISP Ctrl
 * (C)Copyright altek Corporation 2013
 *
 *  2013/10/14; Aaron Chuang; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */


#ifndef _MINIISP_CTRL_H_
#define _MINIISP_CTRL_H_

/**
 *@addtogroup MiniISPCtrl
 *@{
 */

/******Include File******/

#include "mtype.h"
#include "isp_camera_cmd.h"
#include "error.h"
#include "moduleid_pj.h"


/******Public Function Prototype******/


/**
 *\brief Mini ISP open
 *\param boot_code_file_name		[In], Boot code filename
 *\param basic_code_file_name	   [In], Basic code filename
 *\param advanced_code_file_name	[In], Advanced code filename
 *\param scenario_table_file_name	 [In], Sc table filename
 *\return Error code
 *\image html BootSeq.jpg
 */
extern errcode mini_isp_drv_open(char *boot_code_file_name,
				char *basic_code_file_name,
				char *advanced_code_file_name,
				char *scenario_table_file_name);

/**
 *\brief Mini ISP write boot code
 *\param n/a
 *\return Error code
 */
extern errcode mini_isp_drv_write_boot_code(void);

/**
 *\brief Mini ISP write basic code
 *\param n/a
 *\return Error code
 */
extern errcode mini_isp_drv_write_basic_code(void);

/**
 *\brief Mini ISP Write Calibration Data 0x210B
 *\param info_id [In],		0   :  otp data
 *				1   :  packet data
 *				2   :  scenario table
 *\param buf_addr [In], otp/packet data buffer start address
 *\param buf_len [In], otp/packet data buffer len
 *\return Error code
 */
extern errcode mini_isp_drv_write_calibration_data(u8 info_id,
					u8 *buf_addr , u32 buf_len);



/**
 *\brief Set ISP basic parameter
 *\param isp_basic_para [In], ISP basic parameter
 *\return Error code
 */
extern errcode mini_isp_drv_set_basic_param(
	struct isp_cmd_basic_para *isp_basic_para);

/**
 *\brief Set Sensor Mode	0x300A
 *\param sensor_on_off [In],sensor on/off
 *\param scenario_id[In], Scenario ID
 *\param scenario_table_flip_mirror [In], oppsite to set SCID
 *\param otp_data_flip_mirror[In],  opsite to set OTP data
 *\return Error code
 */
extern errcode mini_isp_drv_set_sensor_mode(u8 sensor_on_off,
					u8 scenario_id,
					u8 scenario_table_flip_mirror,
					u8 otp_data_flip_mirror);

/**
 *\brief Preview stream on/off		0x3010
 *\param primary_sensor [In], Primary rear sensor on/off
 *\param second_sensor [In], Second rear sensor on/off
 *\param depth_on_off [In], Depth Stream on/off
 *\return Error code
 */
extern errcode mini_isp_drv_preview_stream_on_off(u8 primary_sensor,
						u8 second_sensor,
						u8 depth_on_off);


extern errcode mini_isp_drv_change_mode(u8 mode_value);

extern errcode mini_isp_drv_get_sensor_mode(void);

extern errcode mini_isp_drv_get_basic_param(void);

extern errcode mini_isp_drv_get_system_info(void);

extern errcode mini_isp_drv_get_last_exec_cmd(void);

extern errcode mini_isp_drv_get_err_code_cmd(void);

extern errcode mini_isp_drv_get_change_mode_status(void);

extern u16 mini_isp_drv_read_spi_status(void);

/** \brief  Master boot miniISP
 *\param  None
 *\return None
 */
extern int mini_isp_drv_boot_mini_isp(void);

/**
 *\brief Set ISP register	0x0100
 *\param reg_start_addr [In], Reg start addr
 *\param reg_value [In], Reg value
 *\return Error code
 */
extern errcode mini_isp_drv_set_isp_register(u32 reg_start_addr,
					u32 reg_value);


/**
 *\brief Get ISP register
 *\param reg_start_addr [In], Reg start addr
 *\param reg_count  [In], Reg count
 *\return Error code
 */
extern errcode mini_isp_drv_get_isp_register(u32 reg_start_addr,
					u32 reg_count);

/**
 *\brief Set Depth 3A Information	0x10B9
 *\param depth_3a_info [In], Depth 3A parameter
 *\return Error code
 */
extern errcode mini_isp_drv_set_depth_3a_info(
	struct isp_cmd_depth_3a_info *depth_3a_info);

/**
 *\brief Set Depth input WOI	0x10BB
 *\param x_position [In], X position
 *\param y_position [In], Y position
 *\param woi_width [In], WOI width
 *\param woi_height [In], WOI height
 *\return Error code
 */
extern errcode mini_isp_drv_set_depth_input_woi(u16 x_position,
						u16 y_position,
						u16 woi_width,
						u16 woi_height);

/**
 *\brief Reading Common Log
 *\param stop [In], Stop to log flag
 *\return Error code
 */
extern errcode mini_isp_drv_read_com_log(bool stop);

/**
 *\brief Read memory
 *\param start_addr [In]starting address
 *\param read_size [In]TotalReadSize
 *\return Error code
 */
extern errcode mini_isp_drv_read_memory(u32 start_addr, u32 read_size);


/**
 *\brief  Master boot miniISP
 *\param  e [In], MINI_ISP_EVENT
 *\return Errorcode
 */
extern int mini_isp_drv_wait_for_event(u16 e);
extern errcode mini_isp_drv_read_reg_e_mode(void);
extern errcode mini_isp_drv_read_reg_e_mode_for_bypass_use(void);
extern errcode mini_isp_drv_setting(u16 mini_isp_mode);
extern void mini_isp_reset(void);

/******End of File******/

/**
 *@}
 */

#endif /* _MINIISP_CTRL_H_*/
