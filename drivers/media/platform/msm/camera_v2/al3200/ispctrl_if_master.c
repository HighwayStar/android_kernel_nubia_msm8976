/*
 * File: ispctrl_if_master.c
 * Description: ISP Ctrl IF Master
 *
 * (C)Copyright altek Corporation 2013
 *
 *  2013/10/14; Aaron Chuang; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */

/******Include File******/
/* Linux headers*/
#include <linux/types.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "include/miniisp.h"
#include "include/ispctrl_if_master.h"
#include "include/isp_camera_cmd.h"
#include "include/error/ispctrl_if_master_err.h"
#include "include/ispctrl_if_master_local.h"
#include "include/miniisp_ctrl.h"



/*extern struct misp_data *misp_drv_data;*/

/******Private Constant Definition******/


#define MASTERTX_BLOCK_SIZE  SPI_BULK_SIZE
#define MINI_ISP_LOG_TAG	"[ispctrl_if_master]"


/******Private Type Definition******/


struct file *g_filp[FIRMWARE_MAX];
struct spi_device *spictrl;

/******Private Function Prototype******/


static errcode execute_system_manage_cmd(struct misp_data *devdata, u16 opcode,
					u8 *param);
static errcode execute_basic_setting_cmd(struct misp_data *devdata, u16 opcode,
					u8 *param);
static errcode execute_bulk_data_cmd(struct misp_data *devdata, u16 opcode,
				u8 *param);
static errcode execute_camera_profile_cmd(struct misp_data *devdata, u16 opcode,
					u8 *param);
static errcode execute_operation_cmd(struct misp_data *devdata, u16 opcode,
					u8 *param);

static ssize_t ispctrl_if_mast_show(struct device *dev,
				   struct device_attribute *attr, char *buf);
static ssize_t ispctrl_if_mast_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);


static u16 isp_mast_calculate_check_sum(u8 *input_buffer_addr,
					u16 input_buffer_size, bool b2sCom);

static int isp_mast_log_task(void *data);


/******Private Global Variable******/

static struct task_struct *spi_log_task;

static DEVICE_ATTR(ispctrl_config, S_IRUGO | S_IWUSR, ispctrl_if_mast_show,
		ispctrl_if_mast_store);


/******Private Global Variable******/




/******Public Function******/

/*
 *\brief ISP Ctrl IF Master init
 *\param none
 *\return Error code
 */
static int __init ispctrl_if_mast_init(void)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;

	misp_info("%s - start", __func__);

	/*Reset parameters*/
	isp_mast_camera_profile_para_init();
	mast_basic_setting_para_init();

	spi_log_task = kthread_run(isp_mast_log_task, NULL,
				"isp_master_log_thread");
	if (IS_ERR(spi_log_task)) {
		err = PTR_ERR(spi_log_task);
		misp_err("%s - thread created err: %d", __func__, err);
		goto ispctrl_if_mast_init_end;
	}
	misp_info("%s - success.", __func__);

ispctrl_if_mast_init_end:

	return err;
}

static void __exit ispctrl_if_mast_exit(void)
{
	if (spi_log_task)
		kthread_stop(spi_log_task);
}


/*
 *\brief Execute SPI master command
 *\param opcode [In], Op code
 *\param param [In], CMD param buffer
 *\return Error code
 */
errcode ispctrl_if_mast_execute_cmd(u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	struct misp_data *devdata;
	misp_info("%s - start0. enter ispctrl_if_mast_execute_cmd opcode: %#04x", __func__, opcode);

	devdata = get_mini_isp_data();
	if (!devdata)
		return -ENODEV;

	misp_info("%s - start1. enter ispctrl_if_mast_execute_cmd opcode: %#04x", __func__, opcode);

	mutex_lock(&devdata->busy_lock);

	/* System Management 0x0000~0x0FFF*/
	if (opcode <= 0x0FFF)
		err = execute_system_manage_cmd(devdata, opcode, param);
	/* Basic Setting 0x1000~0x1FFF*/
	else if (opcode <= 0x1FFF)
		err = execute_basic_setting_cmd(devdata, opcode, param);
	/* Bulk Data 0x2000~0x2FFF*/
	else if (opcode <= 0x2FFF)
		err = execute_bulk_data_cmd(devdata, opcode, param);
	/* Camera Profile 0x3000~0x3FFF*/
	else if (opcode <= 0x3FFF)
		err = execute_camera_profile_cmd(devdata, opcode, param);
	/*Operation 0x4000~0x4FFF*/
	else if (opcode <= 0x4FFF)
		err = execute_operation_cmd(devdata, opcode, param);

	msleep(2);
	mutex_unlock(&devdata->busy_lock);

	return err;
}
EXPORT_SYMBOL(ispctrl_if_mast_execute_cmd);

/*
 *\brief Send command to slave
 *\param devdata [In], misp_data
 *\param opcode [In], Op code
 *\param param [In], CMD param buffer
 *\param len [In], CMD param size
 *\return Error code
 */
errcode ispctrl_mast_send_cmd_to_slave(struct misp_data *devdata,
					u16 opcode, u8 *param, u32 len)
{
	u16 *send_len, *send_opcode, total_len;
	errcode err = ERR_SUCCESS;
	u16 chksum;

	memset(devdata->tx_buf, 0, SPI_TX_BUF_SIZE);

	send_len = (u16 *)&devdata->tx_buf[0];
	send_opcode = (u16 *)&devdata->tx_buf[ISPCMD_OPCODEBYTES];

	/*[2-byte len field] + [2-byte opcode field] + (params len)*/
	total_len = ISPCMD_HDRSIZE + len;
	/* totoal - 2-byte length field*/
	*send_len = total_len - ISPCMD_LENFLDBYTES;

	*send_opcode = opcode;
	if (len > 0)
		memcpy(&devdata->tx_buf[ISPCMD_HDRSIZE], param, len);

	/*calculate checksum*/
	chksum = isp_mast_calculate_check_sum(devdata->tx_buf,
						total_len, true);
	memcpy(&devdata->tx_buf[total_len], &chksum, ISPCMD_CKSUMBYTES);

	/*add bytes for checksum*/
	total_len += ISPCMD_CKSUMBYTES;

	/**
	 * tx_buf:
	 * |--len_field(2)--|
	 *		 |--opcode(2)--|--param(len)--|
	 *					       |--cksum(2)--|
	 * len_field size: ISPCMD_LENFLDBYTES (2 bytes)
	 * opcode size   : ISPCMD_OPCODEBYTES (2 bytes)
	 * param size	: (len bytes)
	 * ISP_CMD_HDR_SIZE = ISPCMD_LENFLDBYTES + ISPCMD_OPCODEBYTES (4 bytes)
	 *
	 * total_len: (len_field_size + opcode_size + param_size + cksum_size)
	 * len(param_len) = (opcode_size + param_size), excluding cksum
	 */

	/* Send command to slave*/
	err = mini_isp_spi_send(devdata,  total_len);

	return err;
}

/*
 *\brief Receive response from slave
 *\param devdata [In], misp_data
 *\param param [Out], Respons e buffer
 *\param len [Out], Response size
 *\return Error code
 */
errcode ispctrl_mast_recv_response_from_slave(struct misp_data *devdata,
				u8 *param, u32 len, bool wait_int)
{
	errcode err = ERR_SUCCESS;
	u32 total_len;
	u16  org_chk_sum;

	memset(devdata->rx_buf, 0, SPI_RX_BUF_SIZE);
	total_len = len + ISPCMD_HDRSIZEWDUMMY + ISPCMD_CKSUMBYTES;

	/*Receive command from slave*/
	err = mini_isp_spi_recv(devdata, total_len, wait_int);
	if (ERR_SUCCESS != err)
		goto ispctrl_mast_recv_response_from_slave_end;


	/*checksum*/
	memcpy(&org_chk_sum, &devdata->rx_buf[(total_len - ISPCMD_CKSUMBYTES)],
		ISPCMD_CKSUMBYTES);
	if (org_chk_sum != isp_mast_calculate_check_sum(devdata->rx_buf,
				(total_len - ISPCMD_CKSUMBYTES), true)) {
		misp_err("%s - checksum error", __func__);
		err = ERR_MASTERCMDCKSM_INVALID;
		goto ispctrl_mast_recv_response_from_slave_end;
	}


	/* Copy param data*/
	memcpy(param, &devdata->rx_buf[ISPCMD_HDRSIZEWDUMMY], len);

ispctrl_mast_recv_response_from_slave_end:

	return err;

}

/*
 *\brief Receive Memory data from slave
 *\param devdata [In], misp_data
 *\param response_buf [Out], Response buffer
 *\param response_size [Out], Response size
 *\param wait_int [In], waiting INT flag
 *\return Error code
 */
errcode ispctrl_if_mast_recv_memory_data_from_slave(
						struct misp_data *devdata,
						u8 *response_buf,
						u32 *response_size,
						bool wait_int)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 expect_size, block_size = MASTERTX_BLOCK_SIZE;
	u16 spi_status;
	u16 i;

	expect_size = *response_size;

	misp_info("%s - Start.", __func__);

	if (wait_int) {
		err = mini_isp_wait_for_event(MINI_ISP_RCV_READY);
		if (err) {
			misp_err("%s - irq error: status=%d",
				__func__, err);
			goto ispctrl_if_mast_recv_memory_data_from_slave_end;
		}
	} else {
		for (i = 0; i < 200; i++) {
			err = mini_isp_get_spi_status(devdata,
						&spi_status);
			if (spi_status & SPI_STATUS_BIT_READY)
				break;
			msleep(5);
		}
		if (i >= 200) {
			misp_err("%s time out.", __func__);
			err = ERR_MASTER_EVENT_TIMEOUT;
			goto ispctrl_if_mast_recv_memory_data_from_slave_end;
		}

	}


	err = mini_isp_get_bulk(devdata, response_buf, expect_size,
				block_size);

ispctrl_if_mast_recv_memory_data_from_slave_end:

	return err;
}
/*
 * \brief  Master send bulk (large data) to slave
 *   \param  devdata [In], misp_data
 *   \param  buffer [In], Data buffer to be sent, address 8-byte alignment
 *   \param  filp [In], file pointer, used to read the file and send the data
 *   \param  total_size [In], file size
 *   \param  block_size [In], transfer buffer block size
 *   \param  is_raw [In], true: mini boot code  false: other files
 *   \return Error code
 */
errcode ispctrl_if_mast_send_bulk(struct misp_data *devdata, u8 *buffer,
	struct file *filp, u32 total_size, u32 block_size, bool is_raw)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;

	if ((!filp) && (buffer == NULL))
		return -ENOENT;

	misp_info("ispctrl_if_master_send_bulk Start ============");
	/* Transfer basic code*/
	err = mini_isp_send_bulk(devdata, filp, total_size, block_size,
				is_raw, buffer);

	if (err != ERR_SUCCESS)
		misp_err("ispctrl_if_master_send_bulk failed!!");
	misp_info("ispctrl_if_master_send_bulk End ============");

	/*close the file*/
	if (filp && (buffer == NULL))
		filp_close(filp, NULL);

	return err;

}


/* open boot / basic / advanced / sc table file*/
errcode ispctrl_if_mast_request_firmware(u8 *filepath, u8 firmwaretype)
{
	errcode err = ERR_SUCCESS;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	misp_info("%s filepath : %s", __func__, filepath);

	g_filp[firmwaretype] = filp_open(filepath, O_RDONLY, 0644);
	set_fs(oldfs);

	if (IS_ERR(g_filp[firmwaretype])) {
		err = PTR_ERR(g_filp[firmwaretype]);
		misp_err("%s open file failed. err: %x", __func__, err);
	} else {
		misp_info("%s open file success!", __func__);
	}
	return err;
}


u16 ispctrl_if_mast_read_spi_status(void)
{
	return mini_isp_get_status();
}

/*
 *\brief Receive ISP register response from slave
 *\param devdata [In], misp_data
 *\param response_buf [Out], Response buffer
 *\param response_size [Out], Response size
 *\param total_count [In], Total reg count
 *\return Error code
 */
errcode ispctrl_if_mast_recv_isp_register_response_from_slave(
		struct misp_data *devdata, u8 *response_buf,
		u32 *response_size, u32 total_count)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 total_len;
	u16 org_chk_sum;

	/* Last packet flag*/
	bool last_packet = false;
	/* Get Reg count*/
	u8 reg_count = 0;
	/* Total count*/
	u32 use_total_count = total_count;
	/* Total return size*/
	u32 total_ret_size = 0;
	/* Max count*/
	u8 max_reg_count = 12;
	/* Checksum*/
	u16 check_sum = 0;
	bool wait_int = true;




	/* Update expect size*/
	total_len = ISPCMD_HDRSIZEWDUMMY + ISP_REGISTER_PARA_SIZE;

	/* Multi SPI Tx transfer*/
	/*if One SPI Rx recv*/
	while (last_packet == false) {
		/* One SPI Rx recv*/
		if (use_total_count <= max_reg_count) {
			/* Set reg count*/
			reg_count = use_total_count;
			/* Set last packet*/
			last_packet = true;
		} else {
			/*Multi SPI Rx recv*/
			reg_count = max_reg_count;
		}
		/* Update expect size*/
		total_len += reg_count*ISP_REGISTER_VALUE;

		/* Add bytes for checksum*/
		if (last_packet == true)
			total_len += ISPCMD_CKSUMBYTES;


		/* Receive command from slave*/
		err = mini_isp_spi_recv(devdata, total_len, wait_int);
		if (ERR_SUCCESS != err)
			break;

		/* Last packet*/
		if (last_packet == true) {
			/* Get checksum*/
			memcpy(&org_chk_sum,
			    &devdata->rx_buf[(total_len - ISPCMD_CKSUMBYTES)],
			    ISPCMD_CKSUMBYTES);

			/* Count checksum*/
			check_sum += isp_mast_calculate_check_sum(
				&devdata->rx_buf[total_ret_size],
				total_len - ISPCMD_CKSUMBYTES, false);
			/* Do 2's complement*/
			check_sum = 65536 - check_sum;

			/* Checksum error*/
			if (org_chk_sum != check_sum)	{
				/* Set error code*/
				err = ERR_MASTERCMDCKSM_INVALID;
				break;
			}
		} else {
			/* Normal packet*/
			/* checksum is valid or not*/
			check_sum += isp_mast_calculate_check_sum(
				&devdata->rx_buf[total_ret_size], total_len,
				false);
		}

		/* Update total count*/
		use_total_count -= reg_count;
		/* Update total ret size*/
		total_ret_size += total_len;

		/* Reset expect size*/
		total_len = 0;
		/* Update max reg count*/
		max_reg_count = 16;

	}


	#ifdef OUTPUT_ISP_REGISTER
	/*write out the buffer to .arw file here*/
	#endif


	return err;

}

/******Private Function******/


/**
 *\brief Execute system manage command
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
static errcode execute_system_manage_cmd(struct misp_data *devdata, u16 opcode,
						u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;

	/* Change mode*/
	if (opcode == ISPCMD_SYSTEM_CHANGEMODE)
		err = mast_sys_manage_cmd_change_mode(devdata, opcode,
							param);
	/* Get status of Mode Change*/
	else if (opcode == ISPCMD_SYSTEM_GET_STATUSOFMODECHANGE)
		err = mast_sys_manage_cmd_get_status_of_mode_change(
						devdata, opcode, param);
	/* Get status of Last Executed Command*/
	else if (opcode == ISPCMD_SYSTEM_GET_STATUSOFLASTEXECUTEDCOMMAND)
		err = mast_sys_manage_cmd_get_status_of_last_exec_command(
						devdata, opcode, param);
	else if (opcode == ISPCMD_SYSTEM_GET_ERRORCODE)
		err = mast_sys_manage_cmd_get_error_code_command(devdata,
							opcode, param);
	/* Set ISP register*/
	else if (opcode == ISPCMD_SYSTEM_SET_ISPREGISTER)
		err = mast_sys_manage_cmd_set_isp_register(devdata,
							opcode,	param);
	/* Get ISP register*/
	else if (opcode == ISPCMD_SYSTEM_GET_ISPREGISTER)
		err = mast_sys_manage_cmd_get_isp_register(devdata,
							opcode,	param);
	/* Set common log level*/
	else if (opcode == ISPCMD_SYSTEM_SET_COMLOGLEVEL)
		err = mast_sys_manage_cmd_set_comomn_log_level(devdata,
							opcode,	param);
	/*Get chip test report*/
	else if (opcode == ISPCMD_SYSTEM_GET_CHIPTESTREPORT)
		err = mast_sys_manage_cmd_get_chip_test_report(devdata,
							opcode, param);
	else if (opcode == ISPCMD_SYSTEM_GET_IRQSTATUS)
		err = mast_sys_manage_cmd_get_irq_status(devdata, opcode,
							param);
	else if (opcode == ISPCMD_SYSTEM_GET_POLLINGCOMMANDSTATUS)
		err = mast_sys_manage_cmd_get_polling_command_status(
						devdata, opcode, param);
	return err;

}

/**
 *\brief Execute basic setting command
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
static errcode execute_basic_setting_cmd(struct misp_data *devdata, u16 opcode,
					u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;


	/* Set Anti-Flicker*/
	if (opcode == ISPCMD_BASIC_SET_DEPTH_3A_INFO)
		err = mast_basic_setting_cmd_set_depth_3a_info(devdata,
								opcode, param);
	else if (opcode == ISPCMD_BASIC_SET_DEPTH_INPUT_WOL)
		err = mast_basic_setting_cmd_set_depth_input_woi(devdata,
								opcode, param);
	return err;

}

/**
 *\brief Execute bulk data command
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
static errcode execute_bulk_data_cmd(struct misp_data *devdata,
				u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 *block_size;
	char filename[ISPCMD_FILENAME_SIZE];

	memcpy(filename, param, ISPCMD_FILENAME_SIZE);


	misp_info("ispctrl_if_master_SendBulk send bulk Start ============");

	/*set the size*/
	block_size = (u32 *)&param[8];

	/* Write Boot Code*/
	if (opcode == ISPCMD_BULK_WRITE_BOOTCODE) {
		g_isMiniISP_sendboot = 1;
		*block_size = MASTERTX_BLOCK_SIZE;
		err = mast_bulk_data_cmd_write_boot_code(devdata, param,
						g_filp[BOOT_CODE]);
		g_isMiniISP_sendboot = 0;
	} else	{
	/* Write Basic Code*/
	if (opcode == ISPCMD_BULK_WRITE_BASICCODE)
		err = mast_bulk_data_cmd_write_basic_code(devdata, param,
						g_filp[BASIC_CODE]);
	/* Write Advanced Code*/
	else if (opcode == ISPCMD_BULK_WRITE_ADVANCEDCODE)
		err = mast_bulk_data_cmd_write_advanced_code(devdata,
						param, g_filp[ADVANCED_CODE]);
	/*Write Calibration Data*/
	else if (opcode == ISPCMD_BULK_WRITE_CALIBRATION_DATA) {
		if (param[8] == 0)
			err = mast_bulk_data_cmd_write_calibration_data(
							devdata, param, NULL);
		else if (param[8] == 1)
			err = mast_bulk_data_cmd_write_calibration_data(
							devdata, param, NULL);
		else
			err = mast_bulk_data_cmd_write_calibration_data(
					devdata, param, g_filp[SCENARIO_CODE]);
	}
	/* Read Calibration Data*/
	else if (opcode == ISPCMD_BULK_READ_MEMORY)
		err = mast_bulk_data_cmd_read_memory_data(devdata, param);
	/* Read Common Log Data*/
	else if (opcode == ISPCMD_BULK_READ_COMLOG)
		err = bulk_data_cmd_read_common_log(devdata, param);
	}

	misp_info("ispctrl_if_master_send_bulk end Errcode: %d ============",
		(int)err);

	return err;

}

/**
 *\brief Execute camera profile command
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
static errcode execute_camera_profile_cmd(struct misp_data *devdata, u16 opcode,
					u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;

	/* Get System Info*/
	if (opcode == ISPCMD_CAMERA_GET_SYSTEMINFORMATION)
		err = mast_camera_profile_cmd_get_system_info(devdata,
							opcode,	param);
	/* Set Basic Parameters*/
	else if (opcode == ISPCMD_CAMERA_SET_BASICPARAMETERS)
		err = mast_camera_profile_cmd_set_basic_parameters(
						devdata, opcode, param);
	/* Get Basic Parameters*/
	else if (opcode == ISPCMD_CAMERA_GET_BASICPARAMETERS)
		err = mast_camera_profile_cmd_get_basic_parameters(
						devdata, opcode, param);
	/* Set Sensor Mode*/
	else if (opcode == ISPCMD_CAMERA_SET_SENSORMODE)
		err = mast_camera_profile_cmd_set_sensor_mode(devdata,
							opcode, param);
	/* Get Sensor Mode*/
	else if (opcode == ISPCMD_CAMERA_GET_SENSORMODE)
		err = mast_camera_profile_cmd_get_sensor_mode(devdata,
							opcode, param);
	/* Get Sensor Mode*/
	else if (opcode == ISPCMD_CAMERA_SET_OUTPUTFORMAT)
		err = mast_camera_profile_cmd_set_output_format(devdata,
							opcode, param);
	/* Preview stream on*/
	else if (opcode == ISPCMD_CAMERA_PREVIEWSTREAMONOFF)
		err = mast_camera_profile_cmd_preview_stream_on_off(
						devdata, opcode, param);
	return err;

}

/**
 *\brief Execute operation command
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
static errcode execute_operation_cmd(struct misp_data *devdata,
				u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;

	/* mini_isp_drv_open*/
	if (opcode == ISPCMD_MINIISPOPEN)
		err = mast_operation_cmd_miniisp_open(devdata,
						opcode, param);

	return err;

}

static u16 isp_mast_calculate_check_sum(u8 *input_buffer_addr,
					u16 input_buffer_size, bool b2sCom)
{
	u16 i;
	u32 sum = 0;
	u16 sumvalue;

	for (i = 0 ; i < input_buffer_size ; i++) {
		if (0 == (i%2))
			sum += input_buffer_addr[i];
		else
			sum += (input_buffer_addr[i] << 8);
	}

	/* Do 2's complement*/
	if (b2sCom == true)
		sumvalue = (u16) (65536 - (sum & 0x0000FFFF));
	/* Update total sum*/
	else
		sumvalue = sum;

	return sumvalue;
}



static ssize_t ispctrl_if_mast_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int status = 0;

	if (attr == &dev_attr_ispctrl_config) {
		/* status = snprintf(buf, PAGE_SIZE,"Rx_buffer[0]:%d,
		 *         Rx_buffer[1]:%d, Rx_buffer[2]:%d Rx_buffer[3]:%d",
		 *
		 * g_aucRxCMDBuf[0],g_aucRxCMDBuf[1],g_aucRxCMDBuf[2],
		 *          g_aucRxCMDBuf[3]);
		 */
	} else {
		status = -EINVAL;
	}
	return status;
}

static ssize_t ispctrl_if_mast_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u8 t_buf[62];
	u16 *opcode;
	u8 *param;

	if (count > 0 && count <= 62) {
		memcpy(t_buf, buf, count);
		opcode = (u16 *)&t_buf[0];
		param = &t_buf[2];
		ispctrl_if_mast_execute_cmd(*opcode, param);
	}

	return count;
}

static int isp_mast_log_task(void *data)
{
	u16 spi_state;

	misp_info("%s - Start", __func__);
	do {
		/*wait fore irq*/
		mini_isp_wait_for_event(MINI_ISP_RCV_LOGFULL |
					MINI_ISP_RCV_ERROR);

		spi_state = mini_isp_get_currentevent();

		misp_info("%s - get event: %#x", __func__, spi_state);

		//if (spi_state & MINI_ISP_RCV_LOGFULL)
		//	mini_isp_drv_read_com_log(false);

		//if (spi_state & MINI_ISP_RCV_ERROR)
		//	mini_isp_drv_get_err_code_cmd();

	} while (!kthread_should_stop());

	misp_info("%s - Thread exit", __func__);
	return 0;
}



module_init(ispctrl_if_mast_init);
module_exit(ispctrl_if_mast_exit);
MODULE_LICENSE("Dual BSD/GPL");


/******End Of File******/
