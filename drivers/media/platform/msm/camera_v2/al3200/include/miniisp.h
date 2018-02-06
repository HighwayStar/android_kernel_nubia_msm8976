/*
 * File: miniisp.h
 * Description: Mini ISP sample codes
 *
 * (C)Copyright altek Corporation 2013
 *
 *  2013/11/01; Bruce Chung; Initial version
 */


#ifndef _MINI_ISP_H_
#define _MINI_ISP_H_

/******Include File******/


#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "include/mtype.h"

/******Public Constant Definition******/

#define MINIISP_DRIVER_VERSION "v1.0"

#define SPI_TX_BUF_SIZE	64
#define SPI_RX_BUF_SIZE	64
//#define SPI_BULK_SIZE (384*1024)
#define SPI_BULK_SIZE (256*1024)
#define SPI_BULK_SIZE_BOOT (8*1024)



//#define DEBUG_ALERT 1
//#define DEBUG_TURNON 1



#ifdef DEBUG_TURNON

	#ifdef DEBUG_ALERT
	#define misp_info(fmt, ...) \
			pr_info(MINI_ISP_LOG_TAG ": " fmt "\n",\
				##__VA_ARGS__)

	#define misp_warn(fmt, ...) \
			pr_warn(MINI_ISP_LOG_TAG ": " fmt "\n",\
				##__VA_ARGS__)

	#define misp_err(fmt, ...) \
			pr_err(MINI_ISP_LOG_TAG ": " fmt "\n",\
				##__VA_ARGS__)
	#else
	#define misp_info(fmt, ...) \
			pr_debug(MINI_ISP_LOG_TAG ": " fmt "\n",\
				##__VA_ARGS__)

	#define misp_warn(fmt, ...) \
			pr_debug(MINI_ISP_LOG_TAG ": " fmt "\n",\
				##__VA_ARGS__)

	#define misp_err(fmt, ...) \
			pr_debug(MINI_ISP_LOG_TAG ": " fmt "\n",\
				##__VA_ARGS__)
	#endif

#else
	#define misp_info(fmt, ...)
	#define misp_err(fmt, ...)
	#define misp_warn(fmt, ...)

#endif

/*Event Bit define*/
/*define ISP control Master event Bit*/
enum MINI_ISP_EVENT {
	MINI_ISP_RCV_WAITING  = 0, /* 0x00000000*/
	MINI_ISP_RCV_READY = (1L << 0), /* 0x00000001*/
	MINI_ISP_RCV_ERROR = (1L << 1), /* 0x00000002*/
	MINI_ISP_RCV_LOGFULL  = (1L << 2), /* 0x00000004*/
	MINI_ISP_RCV_STRMOFF = (1L << 3), /* 0x00000008*/
	MINI_ISP_RCV_PWRDWN = (1L << 4), /* 0x00000010*/
};


/* Definition for SPI status*/
#define SPI_STATUS_BIT_READY		  0x0001
#define SPI_STATUS_BIT_CMD_ERR		0x0002
#define SPI_STATUS_BIT_OTHER_ERROR	0x0004
#define SPI_STATUS_BIT_LOGFULL		0x0008
#define SPI_STATUS_BIT_STRMOFF_READY  0x0010
#define SPI_STATUS_BIT_PWRDWN_READY   0x0020

/******Public Type Declaration******/
struct misp_data {
	struct spi_device *spi;
	struct mutex busy_lock;
	struct semaphore cmd_respond;
	u8 tx_buf[SPI_TX_BUF_SIZE];
	u8 rx_buf[SPI_RX_BUF_SIZE];
};

/******Public Function Prototype******/
extern errcode mini_isp_drv_load_fw(void);
extern errcode mini_isp_drv_setting(u16 mini_isp_mode);

extern struct misp_data *get_mini_isp_data(void);
extern void mini_isp_e_to_a(void);
extern void mini_isp_a_to_e(void);
extern void mini_isp_pure_bypass(u16 mini_isp_mode);
extern int mini_isp_get_chip_id(u32 mini_isp_reg_addr, u8 *id_buf);
extern int mini_isp_spi_recv(struct misp_data *devdata, u32 len, bool wait_int);
extern int mini_isp_spi_send(struct misp_data *devdata, u32 len);
extern int mini_isp_spi_sync(struct spi_device *spi, u8 *tx_buf,
				u8 *rx_buf, u32 len);
extern int mini_isp_dma_write(struct spi_device *spi, u8 *tx_buf, u32 len);
extern int mini_isp_send_bulk(struct misp_data *devdata, struct file *filp,
				u32 total_size , u32 block_size ,
				bool is_raw, u8 *spi_Sendbulkbuffer);
extern int mini_isp_get_bulk(struct misp_data *devdata, u8 *response_buf,
					u32 total_size , u32 block_size);
extern int mini_isp_wait_for_irq(void);
extern int mini_isp_wait_for_event(u16 MINI_ISP_EVENT);
extern int mini_isp_get_spi_status(struct misp_data *devdata, u16 *spi_status);
extern u16 mini_isp_get_currentevent(void);
extern u16 mini_isp_get_status(void);
extern u32 mini_isp_register_memory_read(u32 start_reg_addr, u32 end_reg_addr);



#endif

