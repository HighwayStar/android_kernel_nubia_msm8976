/*
 * File:  bulk_data_cmd.c
 * Description: Mini ISP sample codes
 *
 * (C)Copyright altek Corporation 2013
 *
 *  2013/10/14; Bruce Chung; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 *  2016/05/05; Louis Wang; Linux Coding Style
 */

/******Include File******/
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>

#include "include/isp_camera_cmd.h"
#include "include/ispctrl_if_master.h"
#include "include/error/ispctrl_if_master_err.h"
#include "include/miniisp.h"
#include "include/ispctrl_if_master_local.h"

/******Private Constant Definition******/
#define LOGSIZE  (4*1024)
#define BLOCKSIZE SPI_BULK_SIZE
#define RAWBLOCKSIZE SPI_BULK_SIZE_BOOT
#define MINI_ISP_LOG_TAG	"[bulk_data_cmd]"
#define MID_PJ_EXEBIN_BUF (1024*1024)

/*Private Type Declaration*/
/*Basic code buffer address*/
static u8 *basic_code_buf_addr;
/*Advanced code buffer address*/
static u8 *advance_code_buf_addr;
/*Advanced code size*/
/*Calibration data buffer address*/
static u8 *calibration_data_buf_addr;


/******Private Function Prototype******/
static u16 calculate_check_sum(u8 *input_buffer_addr, u16 input_buffer_size);


/******Private Global Variable******/


/******Public Global Variable*******/

/******Public Function******/


/**
 *\brief Write Boot Code
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp [In], boot code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_boot_code(struct misp_data *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u32 total_size;
	off_t  currpos;

	if (filp == NULL) {
		misp_err("%s - file didn't exist.", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_boot_code_end;
	}

	/*get the file size*/
	currpos = vfs_llseek(filp, 0L, SEEK_END);
	if (currpos == -1) {
		misp_err("%s  llseek failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_boot_code_end;
	}
	total_size = (u32)currpos;
	/*misp_info("%s  filesize : %d", __func__, total_size);*/
	vfs_llseek(filp, 0L, SEEK_SET);

	/*misp_info("block_size %d", RAWBLOCKSIZE);*/

	/*Transfer boot code*/
	err = ispctrl_if_mast_send_bulk(devdata,
		basic_code_buf_addr, filp, total_size, RAWBLOCKSIZE, true);

	if (ERR_SUCCESS != err)
		goto mast_bulk_data_cmd_write_boot_code_end;

	misp_info("%s send boot code success", __func__);

mast_bulk_data_cmd_write_boot_code_end:

	return err;
}


/**
 *\brief Write Basic Code
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], basic code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_basic_code(struct misp_data *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u32 para_size = ISPCMD_EXEBIN_INFOBYTES;
	u32 *total_size = (u32 *)&param[ISPCMD_EXEBIN_ADDRBYTES];
	u32 file_total_size;
	u32 block_size = BLOCKSIZE;
	off_t currpos;
	loff_t offset;
	mm_segment_t oldfs;


	if (filp == NULL) {
		misp_err("%s - file didn't exist.", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_end;
	}

	oldfs = get_fs();
	set_fs(KERNEL_DS);


	/*get the file size*/
	currpos = vfs_llseek(filp, 0L, SEEK_END);
	if (currpos == -1) {
		set_fs(oldfs);
		misp_err("%s  llseek end failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_end;
	}

	file_total_size = (u32)currpos;
	/*misp_info("%s  filesize : %u", __func__, file_total_size);*/

	currpos = vfs_llseek(filp, 0L, SEEK_SET);
	if (currpos == -1) {
		set_fs(oldfs);
		misp_err("%s  llseek set failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_basic_code_end;
	}


	/*read the header info (first 16 bytes in the basic code)*/
	offset = filp->f_pos;
	err = vfs_read(filp, param, ISPCMD_EXEBIN_INFOBYTES, &offset);
	set_fs(oldfs);
	if (err == -1) {
		misp_err("%s Read file failed.", __func__);
		/*close the file*/
		filp_close(filp, NULL);
		goto mast_bulk_data_cmd_write_basic_code_end;
	}
	filp->f_pos = offset;

	/*To copy checksum value to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		  ISPCMD_EXEBIN_TOTALSIZEBYTES + ISPCMD_EXEBIN_BLOCKSIZEBYTES),
		(u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), sizeof(u32));
	/*Assign block size to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), &block_size, sizeof(u32));

	/*misp_info("%s param[0][1][2][3]: %02x %02x %02x %02x",
	 *__func__, param[0], param[1], param[2], param[3]);
	 *misp_info("%s param[4][5][6][7]: %02x %02x %02x %02x",
	 *__func__, param[4], param[5], param[6], param[7]);
	 *misp_info("%s param[8][9][10][11]: %02x %02x %02x %02x",
	 *__func__, param[8], param[9], param[10], param[11]);
	 *misp_info("%s param[12][13][14][15]: %02x %02x %02x %02x",
	 *__func__, param[12], param[13], param[14], param[15]);
	 */

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE, param, para_size);
	if (ERR_SUCCESS != err)
		goto mast_bulk_data_cmd_write_basic_code_end;

	/*misp_info("%s send leaking packet success", __func__);*/

	/*misp_info("block_size %d", BLOCKSIZE);*/

	/*Transfer basic code*/
	err = ispctrl_if_mast_send_bulk(devdata,
		basic_code_buf_addr, filp, *total_size, BLOCKSIZE, false);
	if (ERR_SUCCESS != err)
		goto mast_bulk_data_cmd_write_basic_code_end;

	misp_info("%s send basic code success", __func__);

	/*wait for the interrupt*/
	err = mini_isp_wait_for_event(MINI_ISP_RCV_READY); //comment by congshan


mast_bulk_data_cmd_write_basic_code_end:

	return err;
}


/**
 *\brief Write Advanced Code
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], advanced code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_advanced_code(struct misp_data *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u32 para_size = 16;
	u32 *total_size = (u32 *)&param[4];
	u32 filesize;
	off_t currpos;

	/*get the file size*/
	currpos = vfs_llseek(filp, 0L, SEEK_END);
	if (currpos == -1) {
		misp_err("%s  llseek failed", __func__);
		err = ~ERR_SUCCESS;
		goto mast_bulk_data_cmd_write_advanced_code_end;
	}
	filesize = (u32)currpos;
	misp_info("%s  filesize : %d", __func__, filesize);
	vfs_llseek(filp, 0L, SEEK_SET);

	/*set the file size to param*/
	*total_size = filesize;

	/*wait for the interrupt*/
	err = mini_isp_wait_for_event(MINI_ISP_RCV_READY);
	if (err)
		goto mast_bulk_data_cmd_write_advanced_code_end;

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_ADVANCEDCODE, param, para_size);
	if (ERR_SUCCESS != err)
		goto mast_bulk_data_cmd_write_advanced_code_end;

	/*wait for the interrupt*/
	err = mini_isp_wait_for_event(MINI_ISP_RCV_READY);
	if (err)
		goto mast_bulk_data_cmd_write_advanced_code_end;

	/*Transfer advanced code*/
	err = ispctrl_if_mast_send_bulk(devdata,
		advance_code_buf_addr, filp, filesize, BLOCKSIZE, false);
	if (ERR_SUCCESS != err)
		goto mast_bulk_data_cmd_write_advanced_code_end;

mast_bulk_data_cmd_write_advanced_code_end:

	return err;
}

/**
 *\brief Write Calibration Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], calibration data file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_calibration_data(struct misp_data *devdata,
						u8 *param, struct file *filp)
{
	errcode err = ERR_SUCCESS;
	u8 infomode;
	u16 ckecksum;
	u32 para_size = 11;
	u32 filesize;
	u32 block_size = BLOCKSIZE;
	off_t currpos;
	loff_t offset;
	mm_segment_t oldfs;

	infomode = param[8];


	if (infomode == 2) {
		if (filp == NULL) {
			misp_err("%s - file didn't exist.", __func__);
			err = ~ERR_SUCCESS;
			goto mast_bulk_data_cmd_write_calibration_data_end;
		}

		oldfs = get_fs();
		set_fs(KERNEL_DS);

		/*get the file size*/
		currpos = vfs_llseek(filp, 0L, SEEK_END);
		if (currpos == -1) {
			set_fs(oldfs);
			misp_err("%s  llseek end failed", __func__);
			err = ~ERR_SUCCESS;
			goto mast_bulk_data_cmd_write_calibration_data_end;
		}

		filesize = (u32)currpos;
		/*misp_info("%s  filesize : %u", __func__, filesize);*/

		currpos = vfs_llseek(filp, 0L, SEEK_SET);
		if (currpos == -1) {
			set_fs(oldfs);
			misp_err("%s  llseek set failed", __func__);
			err = ~ERR_SUCCESS;
			goto mast_bulk_data_cmd_write_calibration_data_end;
		}

		/*Request memory*/
		calibration_data_buf_addr = kzalloc(filesize, GFP_KERNEL);
		if (!calibration_data_buf_addr) {
			err = ~ERR_SUCCESS;
			kfree(calibration_data_buf_addr);
			goto mast_bulk_data_cmd_write_calibration_data_fail;
		}

		/*read the header info (first 16 bytes in the data)*/
		offset = filp->f_pos;
		err = vfs_read(filp, calibration_data_buf_addr, filesize, &offset);
		set_fs(oldfs);
		if (err == -1) {
			misp_err("%s Read file failed.", __func__);
			/*close the file*/
			filp_close(filp, NULL);
			kfree(calibration_data_buf_addr);
			goto mast_bulk_data_cmd_write_calibration_data_end;
		}
		filp->f_pos = offset;
		vfs_llseek(filp, 0L, SEEK_SET);

		ckecksum = calculate_check_sum((u8 *) calibration_data_buf_addr,
						filesize);

		/*Assign Info ID to correct header point*/
		memcpy((u8 *)(param + 8), &infomode, sizeof(u8));
		/*To copy Total Size to correct header point*/
		memcpy((u8 *)param, &filesize, sizeof(u32));
		/*Assign block size to correct header point*/
		memcpy((u8 *)(param + 4), &block_size, sizeof(u32));
		/*Assign check sum to correct header point*/
		memcpy((u8 *)(param + 9), &ckecksum, sizeof(u16));

		misp_info("%s SC table[0][1][2][3]:%02x %02x %02x %02x",
			__func__, param[0], param[1], param[2], param[3]);
		misp_info("%s SC table[4][5][6][7]:%02x %02x %02x %02x",
			__func__, param[4], param[5], param[6], param[7]);
		misp_info("%s SC table[8][9][10]:%02x %02x %02x",
			__func__, param[8], param[9], param[10]);

	} else	{
		memcpy(&filesize, param, sizeof(u32));
		calibration_data_buf_addr = kzalloc(filesize, GFP_KERNEL);

		if (!calibration_data_buf_addr) {
			err = ~ERR_SUCCESS;
			kfree(calibration_data_buf_addr);
			goto mast_bulk_data_cmd_write_calibration_data_fail;
		}
		memcpy(calibration_data_buf_addr,
			param+T_SPI_CMD_LENGTH, filesize);
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_CALIBRATION_DATA, param, para_size);
	if (ERR_SUCCESS != err) {
		kfree(calibration_data_buf_addr);
		goto mast_bulk_data_cmd_write_calibration_data_end;
	}

	/*misp_info("%s send leaking packet success", __func__);*/

	/*misp_info("block_size %d", BLOCKSIZE);*/

	err = mini_isp_wait_for_event(MINI_ISP_RCV_READY);
	if (err)
		goto mast_bulk_data_cmd_write_calibration_data_fail;

	err = ispctrl_if_mast_send_bulk(devdata,
		calibration_data_buf_addr, filp, filesize, BLOCKSIZE,
		false);

	if (calibration_data_buf_addr)
		kfree(calibration_data_buf_addr);

	if (ERR_SUCCESS != err)
		goto mast_bulk_data_cmd_write_calibration_data_end;

	err = mini_isp_wait_for_event(MINI_ISP_RCV_READY);
	if (err)
		goto mast_bulk_data_cmd_write_calibration_data_fail;
	
	if (infomode == 0)
		misp_info("%s write IQ calibration data success", __func__);
	else if (infomode == 1)
		misp_info("%s write depth packet data success", __func__);
	else
		misp_info("%s write scenario table success", __func__);

	goto mast_bulk_data_cmd_write_calibration_data_end;

mast_bulk_data_cmd_write_calibration_data_fail:
	misp_err("%s Allocate memory failed.", __func__);

mast_bulk_data_cmd_write_calibration_data_end:
	return err;

}

/*
 *\brief Read Memory Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_read_memory_data(struct misp_data *devdata,
							u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u8 *allocated_memmory = 0;
	u32 read_size = 0;
	/*
	 *Parameter size
	 *4bytes for start addr, 4 bytes for total size, 4 bytes for block size,
	 *4 bytes for memory dump mode
	 */
	u32 para_size = sizeof(struct memmory_dump_hdr_info);
	/*Total size*/
	u32 total_size;
	struct memmory_dump_hdr_info *memory_dump_hdr_config;
	struct file *f;
	mm_segment_t fs;

	read_size = MID_PJ_EXEBIN_BUF;

	/*Request memory*/
	allocated_memmory = kzalloc(read_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		goto allocate_memory_fail;
	}


	memory_dump_hdr_config = (struct memmory_dump_hdr_info *)param;

	/*Assign total size*/
	total_size = memory_dump_hdr_config->total_size;
	if (total_size > read_size) {
		err = ERR_MASTERCMDSIZE_MISMATCH;
		goto mast_bulk_data_cmd_read_memory_data_end;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_READ_MEMORY, param, para_size);
	if (ERR_SUCCESS != err) {
		kfree(allocated_memmory);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}
	/*Get memory data from slave*/
	err = ispctrl_if_mast_recv_memory_data_from_slave(devdata,
							allocated_memmory,
							&total_size, true);
	if (ERR_SUCCESS != err) {
		kfree(allocated_memmory);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}

	misp_info("%s - Read memory finished.", __func__);
	/*write out allocated_memmory to file here*/
	/*** add your codes here ***/
	f = filp_open("/data/misc/camera/miniISP_memory.log", O_APPEND|O_CREAT, 0777);
	if (IS_ERR(f)) {
	    pr_err("%s cannot creat file \n", __func__);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}	
	/*Get current segment descriptor*/
	fs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());
	/*write the file*/
	f->f_op->write(f, (char *)allocated_memmory, strlen(allocated_memmory),
		&f->f_pos);
	/*Restore segment descriptor*/
	set_fs(fs);
	filp_close(f, NULL);
	/*** end of the codes ***/

	goto mast_bulk_data_cmd_read_memory_data_end;

allocate_memory_fail:
	kfree(allocated_memmory);
	misp_err("%s Allocate memory failed.", __func__);
mast_bulk_data_cmd_read_memory_data_end:

	return err;


}

/*
 *\brief Read common log data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\return Error code
 */
errcode bulk_data_cmd_read_common_log(struct misp_data *devdata, u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u8   *allocated_memmory	   = 0;
	u32  read_size	  = LOGSIZE;
	struct file *f;
	mm_segment_t fs;

	/*Parameter size : 4 bytes for total size, 4 bytes for block size*/
	u32 para_size = sizeof(struct common_log_hdr_info);
	/*Total size*/
	u32 total_size;
	struct common_log_hdr_info *common_log_hdr_cfg;


	/*Request memory*/
	allocated_memmory = kzalloc(read_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		goto allocate_memory_fail;
	}


	common_log_hdr_cfg = (struct common_log_hdr_info *)param;

	/*Assign total size*/
	total_size = common_log_hdr_cfg->total_size;
	if (total_size > read_size) {
		err = ERR_MASTERCMDSIZE_MISMATCH;
		kfree(allocated_memmory);
		goto bulk_data_cmd_read_common_log_end;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_READ_COMLOG, param, para_size);
	if (ERR_SUCCESS != err) {
		kfree(allocated_memmory);
		goto bulk_data_cmd_read_common_log_end;
	}
	misp_info("%s - Start to read log.", __func__);

	/*
	 *Get memory data from slave,don't wait INT
	 *and use polling interval to wait
	 */
	err = ispctrl_if_mast_recv_memory_data_from_slave(devdata,
							allocated_memmory,
							&total_size,
							false);
	if (ERR_SUCCESS != err) {
		kfree(allocated_memmory);
		goto bulk_data_cmd_read_common_log_end;
	}
	misp_info("%s - Read log finished.", __func__);


	f = filp_open("/data/misc/camera/miniISP.log", O_APPEND|O_CREAT, 0777);
	if (IS_ERR(f)) {
	    pr_err("%s cannot creat file \n", __func__);
		goto bulk_data_cmd_read_common_log_end;
	}	
	/*Get current segment descriptor*/
	fs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());
	/*write the file*/
	f->f_op->write(f, (char *)allocated_memmory, strlen(allocated_memmory),
		&f->f_pos);
	/*Restore segment descriptor*/
	set_fs(fs);
	filp_close(f, NULL);

	goto bulk_data_cmd_read_common_log_end;
allocate_memory_fail:
	kfree(allocated_memmory);
	misp_err("%s Allocate memory failed.", __func__);
bulk_data_cmd_read_common_log_end:

	return err;
}


/******Private Function******/
static u16 calculate_check_sum(u8 *input_buffer_addr, u16 input_buffer_size)
{
	u16 i;
	u32 sum = 0;
	u16 sumvalue;

	/*calculating unit is 2 bytes */
	for (i = 0; i < input_buffer_size; i++) {
		if (0 == (i%2))
			sum += input_buffer_addr[i];
		else
			sum += (input_buffer_addr[i] << 8);
	}

	/*Do 2's complement */
	sumvalue = (u16)(65536 - (sum & 0x0000FFFF));

	return sumvalue;
}

/******End Of File******/
