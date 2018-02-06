/*
 * File: miniisp_ctrl.c
 * Description: Mini ISP Ctrl sample codes
 *
 * (C)Copyright altek Corporation 2013
 *
 *  2013/10/14; Aaron Chuang; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */


/******Include File******/

#include <linux/types.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "include/miniisp.h"
#include "include/miniisp_customer_define.h"
#include "include/ispctrl_if_master.h"
#include "include/miniisp_ctrl.h"
#include "include/ispctrl_if_master_local.h"
#include "include/error/ispctrl_if_master_err.h"


/******Private Constant Definition******/

#define AEZONECFG_NUM 1 /*must smaller than AEZONECFG_MAXGROUP*/
#define AFZONECFG_NUM 1 /*must smaller than AFZONECFG_MAXGROUP*/
#define MINI_ISP_LOG_TAG	"[miniisp_ctrl]"

#define MINI_PARAMETERS 0


/******Private Function Prototype******/

static int load_code_task(void *data);
static u16 calibration_check_sum(u8 *input_buffer_addr, u16 input_buffer_size);


/******Private Type Declaration******/


/******Private Global Variable******/

static struct memmory_dump_hdr_info mem_dum_hdr_cfg = {0};
static struct common_log_hdr_info  com_log_hdr_cfg = {0};
static bool stop_to_log;

/*Command parameter buffer*/
static u8 cmd_param_buf[T_SPI_CMD_LENGTH];
static u8 rcv_cmd_param_buf[T_SPI_CMD_LENGTH];

static bool load_code_ready;


/******Public Global Variable******/


/******Public Function******/

/*extern void mini_isp_reset(void);*/

/**
 *\brief Mini ISP open 0x4000
 *\param boot_code_file_name [In], Boot code filename
 *\param basic_code_file_name [In], Basic code filename
 *\param advanced_code_file_name [In], Advanced code filename
 *\param scenario_table_file_name [In], SC table filename
 *\return Error code
 */
errcode mini_isp_drv_open(char *boot_code_file_name,
				char *basic_code_file_name,
				char *advanced_code_file_name,
				char *scenario_table_file_name)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_MINIISPOPEN; /*0x4000*/
	u64 bootPath, basicPath, advPath, scenarioPath;

	bootPath  = (u64)boot_code_file_name;
	basicPath = (u64)basic_code_file_name;
	advPath   = (u64)advanced_code_file_name;
	scenarioPath  = (u64)scenario_table_file_name;


	/* Command parameter buffer*/
	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);
	misp_info("%s - test start", __func__);
	/* Parameter 0 boot code filename*/
	memcpy(&cmd_param_buf[0], &bootPath, 8);
	/* Parameter 1 basic code filename*/
	memcpy(&cmd_param_buf[8], &basicPath, 8);
	/* Parameter 2 advanced code filename*/
	memcpy(&cmd_param_buf[16], &advPath, 8);
	/* Parameter 3 calibration filename(sc atable)*/
	memcpy(&cmd_param_buf[24], &scenarioPath, 8);


	/* mini ISP open*/
	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	if (err != ERR_SUCCESS)
		misp_err("%s open file failed. err: 0x%x", __func__, err);

	return err;

}
EXPORT_SYMBOL(mini_isp_drv_open);

/**
 *\brief Mini ISP write boot code 0x2008
 *\return Error code
 */
errcode mini_isp_drv_write_boot_code(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u16 opcode = ISPCMD_BULK_WRITE_BOOTCODE; /*0x2008*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);
	misp_info("%s write boot code state: %d", __func__, err);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_write_boot_code);

/**
 *\brief Mini ISP write basic code 0x2002
 *\return Error code
 */
errcode mini_isp_drv_write_basic_code(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u16 opcode = ISPCMD_BULK_WRITE_BASICCODE; /*0x2002*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	misp_info("%s write basic code state: %d", __func__, err);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_write_basic_code);


/**
 *\brief MiniISP Write Calibration Data   0x210B
 *\param info_id [In],		0   :  otp data
 *				1   :  packet data
 *				2   :  scenario table
 *\param buf_addr [In], otp/packet data buffer start address
 *\param buf_len [In], otp/packet data buffer len
 *\return Error code
 */
errcode mini_isp_drv_write_calibration_data(u8 info_id, u8 *buf_addr,
					u32 buf_len)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u16 opcode = ISPCMD_BULK_WRITE_CALIBRATION_DATA; /*0x210B*/
	u16 chk_sum;
	u32 block_size = 384*1024;
	u8 *allocated_memmory = 0;

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/*
	 *misp_info("%s info_id %d  buf_addr %p buf_len %d",
	 *	__func__, info_id, buf_addr, buf_len);
	 */
	/* Parameter 0 Info ID*/
	cmd_param_buf[8] = info_id;
	if (info_id == 2) {
		err = ispctrl_if_mast_execute_cmd(opcode,
						cmd_param_buf);
	} else {
		/*  Request memory*/
		allocated_memmory = kzalloc(buf_len+T_SPI_CMD_LENGTH,
					GFP_KERNEL);
		if (!allocated_memmory) {
			err = ~ERR_SUCCESS;
			goto allocate_memory_fail;
		}
		memcpy(allocated_memmory + T_SPI_CMD_LENGTH, buf_addr,
			buf_len);
		memcpy(allocated_memmory, &buf_len, sizeof(u32));
		memcpy(allocated_memmory + 4, &block_size, sizeof(u32));
		memcpy(allocated_memmory + 8, &info_id, sizeof(u8));
		chk_sum = calibration_check_sum(
			allocated_memmory + T_SPI_CMD_LENGTH,
			buf_len);
		memcpy(allocated_memmory+9, &chk_sum, sizeof(u16));
		/*
		 *misp_info("%s Cal_param[0][1][2][3]:%02x %02x %02x %02x",
		 *		__func__, allocated_memmory[0],
		 *		allocated_memmory[1],
		 *		allocated_memmory[2],
		 *		allocated_memmory[3]);
		 *misp_info("%s Cal_param[4][5][6][7]:%02x %02x %02x %02x",
		 *		__func__, allocated_memmory[4],
		 *		allocated_memmory[5],
		 *		allocated_memmory[6],
		 *		allocated_memmory[7]);
		 *misp_info("%s Cal_param[8][9][10]:%02x %02x %02x",
		 *		__func__, allocated_memmory[8],
		 *		allocated_memmory[9],
		 *		allocated_memmory[10]);
		 */
		err = ispctrl_if_mast_execute_cmd(opcode,
					allocated_memmory);
		kfree(allocated_memmory);
	}
	misp_info("%s write calibration data state: %d", __func__, err);


	goto miniisp_drv_write_calibration_data_end;
allocate_memory_fail:
	misp_err("%s Allocate memory failed.", __func__);
	kfree(allocated_memmory);
miniisp_drv_write_calibration_data_end:

	return err;

}
EXPORT_SYMBOL(mini_isp_drv_write_calibration_data);


/**
 *\brief Set ISP basic parameter	0x3002
 *\param isp_basic_para [In], ISP basic parameter
 *\return Error code
 */
errcode mini_isp_drv_set_basic_param(struct isp_cmd_basic_para *isp_basic_para)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_CAMERA_SET_BASICPARAMETERS; /*0x3002*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/* Copy ISP basic parameters*/
	memcpy(cmd_param_buf, isp_basic_para,
		sizeof(struct isp_cmd_basic_para));

	/* Set ISP basic parameter*/
	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;

}
EXPORT_SYMBOL(mini_isp_drv_set_basic_param);

/*0x3003*/
errcode mini_isp_drv_get_basic_param(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	int i = 0;
	/* Op code*/
	u16 opcode = ISPCMD_CAMERA_GET_BASICPARAMETERS; /*0x3003*/

	err = ispctrl_if_mast_execute_cmd(opcode, rcv_cmd_param_buf);
	for (i = 0; i < T_SPI_CMD_LENGTH; i = i + 4)
		if (i + 4 < T_SPI_CMD_LENGTH)
			misp_info("%02x %02x %02x %02x", rcv_cmd_param_buf[i],
			rcv_cmd_param_buf[i+1], rcv_cmd_param_buf[i+2],
			rcv_cmd_param_buf[i+3]);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_get_basic_param);

/**
 *\brief Set Sensor Mode	0x300A
 *\param sensor_on_off [In],sensor on/off
 *\param scenario_id[In], Scenario ID
 *\param scenario_table_flip_mirror [In], oppsite to set SCID
 *\param otp_data_flip_mirror[In],  opsite to set OTP data
 *\return Error code
 */
errcode mini_isp_drv_set_sensor_mode(u8 sensor_on_off, u8 scenario_id,
		u8 scenario_table_flip_mirror, u8 otp_data_flip_mirror)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_CAMERA_SET_SENSORMODE;  /*0x300A*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/* Parameter 0 sensor on/off*/
	cmd_param_buf[0] = sensor_on_off;
	/* Parameter 1 Scenario ID*/
	cmd_param_buf[1] = scenario_id;
	/* Parameter 2 oppsite to set SCID on/off*/
	cmd_param_buf[2] = scenario_table_flip_mirror;
	/* Parameter 3 opsite to set OTP data on/off*/
	cmd_param_buf[3] = otp_data_flip_mirror;
	/* Parameter 4 reserve*/
	cmd_param_buf[4] = 0;
	/* Parameter 5 reserve*/
	cmd_param_buf[5] = 0;
	/* Parameter 6 reserve*/
	cmd_param_buf[6] = 0;


	/* Set Sensor Mode*/
	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;

}
EXPORT_SYMBOL(mini_isp_drv_set_sensor_mode);

/**
 *\brief Set ISP register	0x0100
 *\param a_udStartAddr [In], Reg start addr
 *\param reg_value [In], Reg value
 *\return Error code
 v*/
errcode mini_isp_drv_set_isp_register(u32 reg_start_addr, u32 reg_value)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_SYSTEM_GET_ISPREGISTER; /*0x0101*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/* Reg start addr*/
	memcpy(&cmd_param_buf[0], &reg_start_addr, 4);
	/* Reg count*/
	memcpy(&cmd_param_buf[4], &reg_value, 4);

	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_set_isp_register);


/**
 *\brief Set ISP register	0x0101
 *\param a_udStartAddr [In], Reg start addr
 *\param reg_count [In], Reg count
 *\return Error code
 */
errcode mini_isp_drv_get_isp_register(u32 reg_start_addr, u32 reg_count)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_SYSTEM_GET_ISPREGISTER; /*0x0101*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/* Reg start addr*/
	memcpy(&cmd_param_buf[0], &reg_start_addr, 4);
	/* Reg count*/
	memcpy(&cmd_param_buf[4], &reg_count, 4);

	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_get_isp_register);



/**
 *\brief Preview stream on/off		0x3010
 *\param primary_sensor [In], Primary rear sensor on/off
 *\param second_sensor [In], Second rear sensor on/off
 *\param depth_on_off [In], Depth Stream on/off
 *\return Error code
 */
errcode mini_isp_drv_preview_stream_on_off(u8 primary_sensor,
				u8 second_sensor, u8 depth_on_off)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_CAMERA_PREVIEWSTREAMONOFF; /*0x3010*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/* Parameter 0 primary rear sensor on/off*/
	cmd_param_buf[0] = primary_sensor;
	/* Parameter 1 second rear sensor on/off*/
	cmd_param_buf[1] = second_sensor;
	/* Parameter 2 Front sensor on/off*/
	cmd_param_buf[2] = depth_on_off;

	/* Preview stream on/off*/
	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;


}
EXPORT_SYMBOL(mini_isp_drv_preview_stream_on_off);


errcode mini_isp_drv_set_com_log_level(u32 log_level)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_SYSTEM_SET_COMLOGLEVEL;

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	memcpy(cmd_param_buf, &log_level, sizeof(u32));


	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_set_com_log_level);

/*0x0010*/
errcode mini_isp_drv_change_mode(u8 mode_value)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_SYSTEM_CHANGEMODE; /*0x0010*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	cmd_param_buf[0] = mode_value;

	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_change_mode);

/*0x300B*/
errcode mini_isp_drv_get_sensor_mode(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_CAMERA_GET_SENSORMODE; /*0x300B*/

	err = ispctrl_if_mast_execute_cmd(opcode, rcv_cmd_param_buf);

	return err;

}
EXPORT_SYMBOL(mini_isp_drv_get_sensor_mode);


/**
 *\brief Set Depth 3A Information	0x10B9
 *\param depth_3a_info [In], ISP Depth 3A parameter
 *\return Error code
 */
errcode mini_isp_drv_set_depth_3a_info(
			struct isp_cmd_depth_3a_info *depth_3a_info)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_BASIC_SET_DEPTH_3A_INFO; /*0x10B9*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/* Copy ISP Depth 3A Info*/
	memcpy(cmd_param_buf, depth_3a_info,
		sizeof(struct isp_cmd_depth_3a_info));


	/* Set Depth 3A Info*/
	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_set_depth_3a_info);

/**
 *\brief Set Depth input WOI	0x10BB
 *\param x_position [In], X position
 *\param y_position [In], Y position
 *\param woi_width [In], WOI width
 *\param woi_height [In], WOI height
 *\return Error code
 */
errcode mini_isp_drv_set_depth_input_woi(u16 x_position, u16 y_position,
			u16 woi_width, u16 woi_height)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_BASIC_SET_DEPTH_INPUT_WOL; /*0x10B9*/

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/* Copy X position*/
	memcpy(&cmd_param_buf[0], &x_position, sizeof(u16));
	/* Copy Y position*/
	memcpy(&cmd_param_buf[2], &y_position, sizeof(u16));
	/* Copy WOI width*/
	memcpy(&cmd_param_buf[4], &woi_width, sizeof(u16));
	/* Copy WOI height*/
	memcpy(&cmd_param_buf[6], &woi_height, sizeof(u16));


	/* Set Depth Input WOI*/
	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_set_depth_input_woi);


/*0x3001*/
errcode mini_isp_drv_get_system_info(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_CAMERA_GET_SYSTEMINFORMATION; /*0x3001*/

	err = ispctrl_if_mast_execute_cmd(opcode, rcv_cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_get_system_info);


/*0x0015*/
errcode mini_isp_drv_get_last_exec_cmd(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_SYSTEM_GET_STATUSOFLASTEXECUTEDCOMMAND; /*0x0015*/

	err = ispctrl_if_mast_execute_cmd(opcode, rcv_cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_get_last_exec_cmd);

/*0x0016*/
errcode mini_isp_drv_get_err_code_cmd(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_SYSTEM_GET_ERRORCODE; /*0x0016*/

	err = ispctrl_if_mast_execute_cmd(opcode, rcv_cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_get_err_code_cmd);


/*0x0011*/
errcode mini_isp_drv_get_change_mode_status(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_SYSTEM_GET_STATUSOFMODECHANGE; /*0x0011*/

	err = ispctrl_if_mast_execute_cmd(opcode, rcv_cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_get_change_mode_status);

u16 mini_isp_drv_read_spi_status(void)
{
	return ispctrl_if_mast_read_spi_status();
}
EXPORT_SYMBOL(mini_isp_drv_read_spi_status);

/**
 *\brief Read memory
 *\param start_addr [In]starting address
 *\param read_size [In]TotalReadSize
 *\return Error code
 */
errcode mini_isp_drv_read_memory(u32 start_addr, u32 read_size)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_BULK_READ_MEMORY;

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	mem_dum_hdr_cfg.start_addr = start_addr;/*0x0;//0x9DC00;*/
	mem_dum_hdr_cfg.total_size = read_size;/*T_MEMSIZE;*/
	mem_dum_hdr_cfg.block_size = SPI_BULK_SIZE;
	mem_dum_hdr_cfg.dump_mode = T_MEMDUMP_CPURUN;

	/*Copy it to transmission header*/
	memcpy(&cmd_param_buf[0], &mem_dum_hdr_cfg.start_addr,
		sizeof(struct memmory_dump_hdr_info));

	err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_read_memory);


/**
 *\brief Reading Common Log
 *\param stop [In], Stop to log flag
 *\return Error code
 */
errcode mini_isp_drv_read_com_log(bool stop)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Op code*/
	u16 opcode = ISPCMD_BULK_READ_COMLOG;

	memset(cmd_param_buf, 0, T_SPI_CMD_LENGTH);

	/* Wait semaphore*/
	/*SEMAPHORE_LW_Wait( ISPCTRLIFMASTER_SEMAPHORE_LOGDUMP,*/
	/*	SEMAPHORE_WAITFOREVER );*/

	/* Force to stop log*/
	/*To inform isp to set log level as 0 for stoping log reight away*/
	if (stop)
		mini_isp_drv_set_com_log_level(0);

	if (!stop_to_log) {
		com_log_hdr_cfg.total_size = LEVEL_LOG_BUFFER_SIZE;
		com_log_hdr_cfg.block_size = SPI_BULK_SIZE;

		/*Copy it to transmission header*/
		memcpy(&cmd_param_buf[0], &com_log_hdr_cfg.total_size,
				sizeof(struct common_log_hdr_info));

		err = ispctrl_if_mast_execute_cmd(opcode, cmd_param_buf);

		/* Force to stop log*/
		if (stop)
			stop_to_log = true;
	}

	/* Post semaphore*/
	/*SEMAPHORE_LW_Post( ISPCTRLIFMASTER_SEMAPHORE_LOGDUMP );*/

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_read_com_log);


/**
 *\brief ISP self test mode
 *\return Error code
 */
errcode MiniISPDrv_SystemGetChipTestReport(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;

	return err;
}
EXPORT_SYMBOL(MiniISPDrv_SystemGetChipTestReport);


/**
 *\brief Write boot code and basic code
 *\param None
 *\return None
 */
int mini_isp_drv_boot_mini_isp(void)
{
	errcode err = ERR_SUCCESS;


	/* Write boot code*/
	err = mini_isp_drv_write_boot_code();
	if (err != ERR_SUCCESS)
		goto mini_isp_drv_boot_mini_isp_end;

	udelay(500);

	/* Write basic code*/
	err = mini_isp_drv_write_basic_code();

mini_isp_drv_boot_mini_isp_end:

	return err;

}
EXPORT_SYMBOL(mini_isp_drv_boot_mini_isp);

/**
 *\brief Open boot and FW file then write boot code and FW code
 *\param None
 *\return Error code
 */
errcode mini_isp_drv_load_fw(void)
{
	errcode err = ERR_SUCCESS;

	misp_info("mini_isp_drv_setting(0) mini_isp_drv_load_fw start");
	/* Clear load code ready flag;*/
	load_code_ready = false;
	/*spi isr task*/
	/*g_ptload_code_task = kthread_run(load_code_task, NULL, */
	/*		"miniISP_loadcode_thread");*/

	load_code_task(NULL);

	misp_info("mini_isp_drv_setting(0) mini_isp_drv_load_fw X");
	return err;

}
EXPORT_SYMBOL(mini_isp_drv_load_fw);

/**
 *\brief  Wait miniISP event
 *\param  e [In], MINI_ISP_EVENT
 *\return Errorcode
 */
int mini_isp_drv_wait_for_event(u16 e)
{
	return mini_isp_wait_for_event(e);
}
EXPORT_SYMBOL(mini_isp_drv_wait_for_event);


/**
 *\brief Set mode to miniISP
 *\param  mini_isp_mode [In], Select ISP MODE,
 *0:(isp already in state A)normal case load FW directly,
 *1 :(isp state inital in state E)set state E to state A
 *2:(isp already in state A)set state A to state E for debug ,
 *4 :Get Chip ID
 *0x1000:bypass_imx230_ov2680_bin
 *0x1001:bypass_imx230_ov2680_FR
 *0x1002:bypass_imx230_ov2680_720p
 *\return Errorcode
 */
errcode mini_isp_drv_setting(u16 mini_isp_mode)
{
	errcode err = ERR_SUCCESS;

	if (mini_isp_mode == MINI_ISP_MODE_NORMAL)
		/*normal case load FW*/
		mini_isp_drv_load_fw();
	else if (mini_isp_mode == MINI_ISP_MODE_E2A)
		/*isp, inital in E,*/
		/*set some reg value to let it know should chang to A*/
		mini_isp_e_to_a();
	else if (mini_isp_mode == MINI_ISP_MODE_A2E)
		mini_isp_a_to_e();
	else if (mini_isp_mode == MINI_ISP_MODE_GET_CHIP_ID) {
		u8 buff_id[4];
		mini_isp_get_chip_id(0xffef0020,buff_id);
	}
	else
		/*Pure bypass by sensor*/
		mini_isp_pure_bypass(mini_isp_mode);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_setting);

/**
 *\brief Read memory in E mode for bypass mode
 *\return Error code
*/
errcode mini_isp_drv_read_reg_e_mode_for_bypass_use(void)
{
	errcode err = ERR_SUCCESS;
        misp_info("duyuerong %s mini_isp_drv_read_reg_e_mode_for_bypass_use", __func__);
	/*clk_gen*/
	misp_info("%s clk_gen", __func__);
	mini_isp_register_memory_read(0xffe80000, 0xffe80c44);
	/*mipi_tx_phy_if_0*/
	misp_info("%s mipi_tx_phy_if_0", __func__);
	mini_isp_register_memory_read(0xffed1000, 0xffed1104);
	/*mipi_tx_phy_if_1*/
	misp_info("%s mipi_tx_phy_if_1", __func__);
	mini_isp_register_memory_read(0xffed6000, 0xffed6104);
	/*gen_reg*/
	misp_info("%s gen_reg", __func__);
	mini_isp_register_memory_read(0xffef0000, 0xffef0554);
	/*mipi_slvds_rx_phy_if_4L_0*/
	misp_info("%s mipi_slvds_rx_phy_if_4L_0", __func__);
	mini_isp_register_memory_read(0xfff91000, 0xfff9114c);
	/*mipi_slvds_rx_phy_if_4L_1*/
	misp_info("%s mipi_slvds_rx_phy_if_4L_1", __func__);
	mini_isp_register_memory_read(0xfff94000, 0xfff9414c);
	/*ppi_bridge_a_0*/
	misp_info("%s ppi_bridge_a_0", __func__);
	mini_isp_register_memory_read(0xfff97000, 0xfff97030);
	mini_isp_register_memory_read(0xfff97110, 0xfff97164);
	/*ppi_bridge_a_1*/
	misp_info("%s ppi_bridge_a_1", __func__);
	mini_isp_register_memory_read(0xfff98000, 0xfff98030);
	mini_isp_register_memory_read(0xfff98110, 0xfff98164);
	return err;
}
EXPORT_SYMBOL(mini_isp_drv_read_reg_e_mode_for_bypass_use);


/**
 *\brief Read memory in E mode
 *\return Error code
*/
errcode mini_isp_drv_read_reg_e_mode(void)
{
	errcode err = ERR_SUCCESS;
	misp_info("mini_isp_drv_read_reg_e_mode ppi_bridge_a_0");
/*tx_top_reg*/
misp_info("mini_isp_drv_read_reg_e_mode tx_top_reg");
mini_isp_register_memory_read(0xffec0000, 0xffec00dc);
/*tx_line_merge_21_a_0*/
misp_info("mini_isp_drv_read_reg_e_mode tx_line_merge_21_a_0");
mini_isp_register_memory_read(0xffec1000, 0xffec1168);
mini_isp_register_memory_read(0xffec1200, 0xffec1218);

/*tx_line_merge_21_b_0*/
misp_info("mini_isp_drv_read_reg_e_mode tx_line_merge_21_b_0");
mini_isp_register_memory_read(0xffec2000, 0xffec2168);
mini_isp_register_memory_read(0xffec2200, 0xffec2218);

/*tx_line_merge_21_a_1*/
misp_info("mini_isp_drv_read_reg_e_mode tx_line_merge_21_a_1");
mini_isp_register_memory_read(0xffec3000, 0xffec3168);
mini_isp_register_memory_read(0xffec3200, 0xffec3218);

/*mipi_csi2_tx_0*/
misp_info("mini_isp_drv_read_reg_e_mode mipi_csi2_tx_0");
mini_isp_register_memory_read(0xffed0000, 0xffed01c0);
/*mipi_tx_phy_if_0*/
misp_info("mini_isp_drv_read_reg_e_mode mipi_tx_phy_if_0");
mini_isp_register_memory_read(0xffed1000, 0xffed1104);
/*mipi_csi2_tx_1*/
misp_info("mini_isp_drv_read_reg_e_mode mipi_csi2_tx_1");
mini_isp_register_memory_read(0xffed5000, 0xffed51c0);
/*mipi_tx_phy_if_1*/
misp_info("mini_isp_drv_read_reg_e_mode mipi_tx_phy_if_1");
mini_isp_register_memory_read(0xffed6000, 0xffed6104);
/*raw_top_reg*/
misp_info("mini_isp_drv_read_reg_e_mode raw_top_reg");
mini_isp_register_memory_read(0xfff00000, 0xfff00108);
/*id_det_a_0*/
misp_info("mini_isp_drv_read_reg_e_mode id_det_a_0");
mini_isp_register_memory_read(0xfff01000, 0xfff01090);
mini_isp_register_memory_read(0xfff01204, 0xfff01204);
/*id_det_a_1*/
misp_info("mini_isp_drv_read_reg_e_mode id_det_a_1");
mini_isp_register_memory_read(0xfff02000, 0xfff02090);
mini_isp_register_memory_read(0xfff02204, 0xfff02204);
/*binning_a_0*/
misp_info("mini_isp_drv_read_reg_e_mode binning_a_0");
mini_isp_register_memory_read(0xfff03000, 0xfff030b4);
/*binning_a_1*/
misp_info("mini_isp_drv_read_reg_e_mode binning_a_1");
mini_isp_register_memory_read(0xfff04000, 0xfff040b4);
/*rx_line_split_a_0*/
misp_info("mini_isp_drv_read_reg_e_mode rx_line_split_a_0");
mini_isp_register_memory_read(0xfff05000, 0xfff05064);
/*rx_line_split_a_1*/
misp_info("mini_isp_drv_read_reg_e_mode rx_line_split_a_1");
mini_isp_register_memory_read(0xfff06000, 0xfff06064);


/*mipi_slvds_rx_phy_if_4L_0*/
misp_info("mini_isp_drv_read_reg_e_mode mipi_slvds_rx_phy_if_4L_0");
mini_isp_register_memory_read(0xfff91000, 0xfff9110c);
/*mipi_csi2_rx_0*/
misp_info("mini_isp_drv_read_reg_e_mode mipi_csi2_rx_0");
mini_isp_register_memory_read(0xfff92000, 0xfff92194);
/*mipi_slvds_rx_phy_if_4L_1*/
misp_info("mini_isp_drv_read_reg_e_mode mipi_slvds_rx_phy_if_4L_1");
mini_isp_register_memory_read(0xfff94000, 0xfff9410c);
/*mipi_csi2_rx_1*/
misp_info("mini_isp_drv_read_reg_e_mode mipi_csi2_rx_1");
mini_isp_register_memory_read(0xfff95000, 0xfff95194);
/*ppi_bridge_a_0*/
misp_info("mini_isp_drv_read_reg_e_mode ppi_bridge_a_0");
mini_isp_register_memory_read(0xfff97000, 0xfff97030);
mini_isp_register_memory_read(0xfff97110, 0xfff97164);
/*ppi_bridge_a_1*/
misp_info("mini_isp_drv_read_reg_e_mode ppi_bridge_a_1");
mini_isp_register_memory_read(0xfff98000, 0xfff98030);
mini_isp_register_memory_read(0xfff98110, 0xfff98164);

	return err;
}
EXPORT_SYMBOL(mini_isp_drv_read_reg_e_mode);


/******Private Function******/

static int load_code_task(void *data)
{
	/* Error code*/
	errcode err = ERR_SUCCESS;

	misp_info("misp_load_fw start");

	/* Reset mini-isp low for at least 200us, release to high for 20ms*/
	/*mini_isp_reset();*/

	/* Open boot file and FW file*/
	err = mini_isp_drv_open(BOOT_FILE_LOCATION,
				BASIC_FILE_LOCATION,
				ADVANCED_FILE_LOCATION,
				SCENARIO_TABLE_FILE_LOCATION);
	if (err != ERR_SUCCESS)
		goto load_code_task_end;



	/* Write boot code and basic code*/
	err = mini_isp_drv_boot_mini_isp();
	if (err != ERR_SUCCESS)
		goto load_code_task_end;

	/* Set load code ready flag*/
	load_code_ready = true;

load_code_task_end:

	return (int)err;
}


static u16 calibration_check_sum(u8 *input_buffer_addr, u16 input_buffer_size)
{
	u16 i;
	u32 sum = 0;
	u16 sumvalue;

	/* calculating unit is 2 bytes*/
	for (i = 0; i < input_buffer_size; i++) {
		if (0 == (i % 2))
			sum += input_buffer_addr[i];
		else
			sum += (input_buffer_addr[i] << 8);
	}

	/* Do 2's complement*/
	sumvalue = (u16)(65536 - (sum & 0x0000FFFF));  /*get 2's complement*/

	return sumvalue;
}






/******End Of File******/
