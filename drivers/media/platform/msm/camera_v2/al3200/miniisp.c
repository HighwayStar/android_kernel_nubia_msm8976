/*
 * File: miniisp.c
 * Description: Mini ISP sample codes
 *
 * (C)Copyright altek Corporation 2013
 *
 *  2013/11/01; Bruce Chung; Initial version
 */

/******Include File******/
/* Linux headers*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include  <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <sound/apr_audio-v2.h>
#include <sound/q6afe-v2.h>
#include <linux/clk.h>

#include "include/miniisp.h"
#include "include/miniisp_ctrl.h"
#include "include/miniisp_customer_define.h"

#include "include/error/miniisp_err.h"

/******Private Constant Definition******/


#define SPI_BUS_SPEED      50000000/*(50000000)19200000*/
#define SPI_BUS_SPEED_BOOT 9600000
#define SPI_BUFF_SIZE 8192
#define RETRYMAX 100
#define IRQ_TIMEOUT 200

#define SPI_DEBUG 1
#define DEBUG_NODE 0

/*#define DEBUG_ALERT*/
#define  BUFFER_SIZE  8192


static struct misp_data *misp_drv_data;
int g_isMiniISP_sendboot = 0;

int g_isMiniISP_Probled = 0;
/**
 *@typedef USPICTRL_MS_CB_ID
 *@brief USPI control byte definition (for master control)
 */
enum {
	/*!< Ctrl-Byte, original command */
	USPICTRL_MS_CB_ORG				  = (0x00<<6),
	/*!< Ctrl-Byte, polling status */
	USPICTRL_MS_CB_STS				  = (0x01<<6),
	/*!< Ctrl-Byte, get response */
	USPICTRL_MS_CB_RSP				  = (0x02<<6),
	/*!< Ctrl-Byte, disable Ctrl-Byte mode */
	USPICTRL_MS_CB_DIS				  = (0x03<<6),
};

#define MINI_ISP_LOG_TAG "[mini_isp]"


/*drv debug defination*/
#define _SPI_DEBUG


/******Private Global Variable******/


struct file *filp[4];
static char spi_databuffer[SPI_TX_BUF_SIZE+1]; /*add 1 for the ctrl byte*/

static u8 *spi_bulkbuffer;
static struct task_struct *spi_task;

static bool irqflag;
static u16 event = MINI_ISP_RCV_WAITING;
static u16 current_event = MINI_ISP_RCV_WAITING;

static u16 statusbyte;
static int irq_gpio = -1;
static int reset_gpio = -1;
static int vcc1_gpio = -1;
static int vcc2_gpio = -1;
static int vcc1v8_gpio = -1;
static spinlock_t spinlock; //ZTEMT: added by congshan 
static int zte_change_event = 0;//ZTEMT: added by congshan 

static DECLARE_WAIT_QUEUE_HEAD(WAITQ);

int g_isMiniISP_bypass = 0;

/******Private Function Prototype******/

#if DEBUG_NODE
static ssize_t
mini_isp_config_show(struct device *dev, struct device_attribute *attr,
			char *buf);
static ssize_t
mini_isp_config_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
static DEVICE_ATTR(mini_isp_config, S_IRUGO|S_IWUSR, mini_isp_config_show,
			mini_isp_config_store);
#endif
//ZTEMT: added by congshan start
static const char *al3200_clk_name;
static struct clk *al3200_clk_ptr;
static struct spi_device		*al3200_spi_device;
static int misp_ts_pinctrl_init(struct spi_device *spi, int on);
void vcc1v8_gpio_set_value(int value);

int al3200_clk_enable(int enable)
{
	int rc = 0;
	if(enable) {
    	rc = clk_prepare_enable(al3200_clk_ptr);
        if (rc < 0) {
            pr_err("al3200 clk_prepare_enable failed\n");
        }	
	} else  
        clk_disable_unprepare(al3200_clk_ptr);
	return rc;
}

void vcc1v8_gpio_set_value(int value)
{
	int ret = 0;
	ret = devm_gpio_request(&al3200_spi_device->dev, vcc1v8_gpio, "vcc1v8_gpio");
	if (ret) {
		misp_err("%s . request vcc1v8-gpios error", __func__);
	}
	gpio_direction_output(vcc1v8_gpio, value);
	pr_err("%s vcc1v8_gpio=%d\n",__func__, gpio_get_value(vcc1v8_gpio));
	if(ret == 0)
	    devm_gpio_free(&al3200_spi_device->dev, vcc1v8_gpio);
}

//ZTEMT: added by congshan end


/******Public Function******/
struct misp_data *get_mini_isp_data(void)
{

       //misp_err("%s - enter get_mini_isp_data", __func__);
	if (!misp_drv_data) {
		misp_err("%s - get pdata error", __func__);
		return NULL;
	} else {
		return misp_drv_data;
	}
}
EXPORT_SYMBOL(get_mini_isp_data);

#ifdef _SPI_DEBUG
void spi_data_debug(const void *buf, int data_len, int dbg_len)
{
	int len = 0, pos = 0;
	unsigned char *char_buf = (unsigned char *)buf;
	   unsigned char string[100], temp[4];

	   memset(string, 0, sizeof(string));

	len = (dbg_len > data_len) ? data_len : dbg_len;

	pos = 0;
	while (len > 0) {
		if (len > 7) {
			misp_info("%02x %02x %02x %02x %02x %02x %02x %02x",
			char_buf[pos], char_buf[pos+1], char_buf[pos+2],
			char_buf[pos+3], char_buf[pos+4], char_buf[pos+5],
			char_buf[pos+6], char_buf[pos+7]);

			len -= 8;
			pos += 8;
		} else {
			for ( ; len > 0; len--) {
				sprintf(temp, "%02x ", char_buf[pos++]);
				strcat(string, temp);
			}
			misp_info("%s", string);
		}
	}
}
#else
#define spi_data_debug(buf, data_len, dbg_len)
#endif


/*interrupt handler function */
static irqreturn_t mini_isp_irq(int irq, void *handle)
{
	unsigned long flags;//ZTEMT: added by congshan 

	spin_lock_irqsave(&spinlock, flags);//ZTEMT: added by congshan 

	misp_info("%s - enter", __func__);

	irqflag = true;

	wake_up_interruptible(&WAITQ);
	spin_unlock_irqrestore(&spinlock, flags);//ZTEMT: added by congshan 

	return IRQ_HANDLED;
}

/*interrupt waiting function */
int mini_isp_wait_for_irq(void)
{
	int state  = 0;

	misp_info("%s - entering", __func__);

	/*state = wait_event_interruptible_timeout(WAITQ,
	 *irqflag!=false, HZ/10);
	 **/
	state = wait_event_interruptible(WAITQ, irqflag != false);

	irqflag = false;
	if (state)
		misp_err("%s - irq error. err: %d", __func__, state);

	misp_info("%s - leaving", __func__);
	
	return state;
}
EXPORT_SYMBOL(mini_isp_wait_for_irq);


int mini_isp_wait_for_event(u16 e)
{
	int state  = 0;
	unsigned long flags;//ZTEMT: added by congshan 

	misp_info("%s - entering. event: %#x  current_event =%#x", __func__, e, event);

	state = wait_event_interruptible(WAITQ, event & e);
	
	spin_lock_irqsave(&spinlock, flags);//ZTEMT: added by congshan 
	current_event = event;

	event = (~e) & event;/*MINI_ISP_RCV_WAITING;*/
	spin_unlock_irqrestore(&spinlock, flags);//ZTEMT: added by congshan 

	if (state)
		misp_err("%s - irq error. err: %d", __func__, state);

	misp_info("%s - leaving. event: %#x", __func__, e);
	
	return state;
}
EXPORT_SYMBOL(mini_isp_wait_for_event);

/*
 *write command data to spi
 *return 0  successful
 *others	fail
 */
int mini_isp_spi_send(struct misp_data *devdata, u32 len)
{
	int status;
	u8 check = USPICTRL_MS_CB_ORG;

	//misp_info("%s - entering  spi_send ", __func__);

	if ((!devdata) || (len > SPI_TX_BUF_SIZE)) {
		misp_err("%s - invalid arg devdata=%p,len=%d", __func__ ,
			devdata, len);
		return -EINVAL;
	}

	/*put the ctrl byte in the first byte, then put the following data.*/
	spi_databuffer[0] = check;

	memcpy(spi_databuffer + 1, devdata->tx_buf, len);

	status = mini_isp_spi_sync(devdata->spi, spi_databuffer,
			NULL, len + 1);

	if (status) {
		misp_err("%s - sync error: status=%d", __func__, status);
		return status;
	}

	//misp_info("%s - leave  spi_send ", __func__);

	return status;
}
EXPORT_SYMBOL(mini_isp_spi_send);

/* read miniISP using spi ,this fucntion will block.
 *return 0  successful
 *others	fail
 */
int mini_isp_spi_recv(struct misp_data *devdata, u32 len, bool wait_int)
{
	int status;
	u8 ctrlbyte = USPICTRL_MS_CB_RSP;
	int i = 0;
	u16 spi_status;
     // misp_info("%s - entering  spi_recv ", __func__);
	if ((!devdata) || (len > SPI_RX_BUF_SIZE)) {
		misp_err("%s - invalid arg devdata=%p,len=%d",
			__func__, devdata, len);
		status = -EINVAL;
		goto mini_isp_spi_recv_end;
	}

	 memset(spi_databuffer, 0, SPI_RX_BUF_SIZE + 1);

	if (wait_int) {
		/*wait for the interrupt*/
		status = mini_isp_wait_for_event(MINI_ISP_RCV_READY);
	} else {
		for (i = 0; i < 200; i++) {
			status = mini_isp_get_spi_status(devdata, &spi_status);
			if (spi_status & SPI_STATUS_BIT_READY)
				break;
			msleep(5);
		}
		if (i >= 200) {
			misp_err("%s time out.", __func__);
			status = ERR_MINIISP_GETDATA_TIMEOUT;
			goto mini_isp_spi_recv_end;
		}
	}



	if (status) {
		misp_err("%s - irq error: status=%d", __func__, status);
		goto mini_isp_spi_recv_end;
	}


	/*send the ctrl byte and get the data in full duplex mode*/
	/*the data is stored from the 2nd byte*/
	spi_databuffer[0] = ctrlbyte;
	status = mini_isp_spi_sync(devdata->spi, spi_databuffer,
			spi_databuffer, len + 1);
	if (status) {
		misp_err("%s - sync error: status=%d", __func__, status);
		goto mini_isp_spi_recv_end;
	}

	memcpy(devdata->rx_buf, spi_databuffer + 1, len);

	misp_info("%s - recv buf len=%d:", __func__, len);

	//misp_info("%s - leaave   spi_recv ", __func__);
	spi_data_debug(devdata->rx_buf, SPI_RX_BUF_SIZE, len);

mini_isp_spi_recv_end:
	return status;
}
EXPORT_SYMBOL(mini_isp_spi_recv);


/*write and read commadn to spi ,this fucntion will block.
 *return 0  successful
 *others	fail
 */
int mini_isp_spi_sync(struct spi_device *spi, u8 *tx_buf, u8 *rx_buf, u32 len)
{

     //misp_info("%s - entering   ", __func__);
	struct spi_transfer t = {
		.tx_buf		= tx_buf,
		.rx_buf		= rx_buf,
		.len		= len,
		.delay_usecs = 1,
		.speed_hz	= SPI_BUS_SPEED,
		};
	struct spi_message m;

	//misp_info("%s - entering   ", __func__);

	if (g_isMiniISP_sendboot)
		t.speed_hz = SPI_BUS_SPEED_BOOT;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	//misp_info("%s - leave   ", __func__);

	return spi_sync(spi, &m);
}
EXPORT_SYMBOL(mini_isp_spi_sync);

int mini_isp_dma_write(struct spi_device *spi, u8 *tx_buf, u32 len)
{
	int state;
	dma_addr_t bus_addr;
	
	struct spi_transfer t = {
		.tx_buf		= tx_buf,
		.len		= len,
	};
	struct spi_message	m;

	misp_info("%s - entering   ", __func__);

	spi_message_init(&m);

	bus_addr = dma_map_single(&spi->dev, tx_buf, len, DMA_TO_DEVICE);

	if (!bus_addr) {
		misp_err("%s dma mapping failed.", __func__);
		state = -ENOMEM;
		goto mini_isp_dma_write_end;
	}

	t.tx_dma = bus_addr;
	m.is_dma_mapped = 1;

	spi_message_add_tail(&t, &m);

	state = spi_sync(spi, &m);

	dma_unmap_single(&spi->dev, bus_addr, len, DMA_TO_DEVICE);

	misp_info("%s - leave   ", __func__);

mini_isp_dma_write_end:

	return state;
}
EXPORT_SYMBOL(mini_isp_dma_write);



/*used to send the firmware*/
int mini_isp_send_bulk(struct misp_data *devdata, struct file *filp,
	u32 total_size, u32 block_size, bool is_raw, u8 *spi_Sendbulkbuffer)
{
	int status = 0, count = 0;
	int remain_size, one_size;
	u8 ctrlbyte = USPICTRL_MS_CB_DIS, ack[2];
	mm_segment_t oldfs;
	loff_t  offset;
	int shift = 0;

	misp_info("%s - entering   ", __func__);

	if (spi_Sendbulkbuffer != NULL) {
		misp_info("%s start. Total size: %d.", __func__, total_size);
		if (!is_raw) {
			/*send the ctrl byte*/
			status = spi_write_then_read(devdata->spi, &ctrlbyte,
						1, ack , 2);
			if (status) {
				misp_err("%s send ctrlbyte fail status: %d",
					__func__, status);
				status = -EINVAL;
				goto T_EXIT;
			}
		}

		/* Allocate basic code bulk buffer*/
		if (total_size > SPI_BULK_SIZE_BOOT)
			spi_bulkbuffer = vmalloc(SPI_BULK_SIZE);
		/* Allocate boot code bulk buffer*/
		else
			spi_bulkbuffer = vmalloc(SPI_BULK_SIZE_BOOT);

		if (!spi_bulkbuffer) {
			misp_err("%s - Can not alloc SPI bulk buffer",
				__func__);
			status = -EINVAL;
			goto T_EXIT;
		}else{

			//misp_err("%s - alloc SPI bulk buffer ok",__func__);
             }


		for (remain_size = total_size; remain_size > 0;
			remain_size -= one_size) {
			one_size = (remain_size > block_size) ?
				block_size : remain_size;


			misp_info("remain size: %d  one_size: %d.",
				remain_size, one_size);

			memcpy(spi_bulkbuffer, (spi_Sendbulkbuffer + shift),
				one_size);
			shift += one_size;

			/*send the data*/
			status = mini_isp_spi_sync(devdata->spi,
					spi_bulkbuffer, NULL, one_size);

			if (status != 0) {
				misp_err(
				"%s failed! block:%d status:%d",
				__func__, count, status);
				break;
			}

			misp_info("%s write block %d success",
				__func__, count);

			count++;
		}

	} else {
		oldfs = get_fs();

		set_fs(get_ds());

		misp_info("%s start. Total size: %d", __func__, total_size);

		if (!is_raw) {
			/*send the ctrl byte*/
			status = spi_write_then_read(devdata->spi, &ctrlbyte,
						1, ack , 2);
			if (status) {
				misp_err("%s ctrl byte fail status: %d",
				__func__, status);
				status = -EINVAL;
				goto T_EXIT;
			}
		}

		/* Allocate basic code bulk buffer*/
		if (total_size > SPI_BULK_SIZE_BOOT)
			spi_bulkbuffer = vmalloc(SPI_BULK_SIZE);
		/* Allocate boot code bulk buffer*/
		else
			spi_bulkbuffer = vmalloc(SPI_BULK_SIZE_BOOT);


		if (!spi_bulkbuffer) {
			misp_err("%s - Can not alloc SPI bulk buffer",
				__func__);
			status = -EINVAL;
			goto T_EXIT;
		}else{

			//misp_err("%s - alloc SPI bulk buffer2 ok",__func__);
             }

		for (remain_size = total_size; remain_size > 0;
			remain_size -= one_size) {
			one_size = (remain_size > block_size) ?
				block_size : remain_size;

			misp_info("remain size: %d  one_size: %d",
				remain_size, one_size);

			/*copy the firmware to the buffer*/
			offset = filp->f_pos;
			status = vfs_read(filp, spi_bulkbuffer, one_size,
					&offset);

			if (status == -1) {
				misp_info("%s Read file failed.", __func__);
				break;
			}

			filp->f_pos = offset;

			/*send the data*/
			status = mini_isp_spi_sync(devdata->spi,
				spi_bulkbuffer, NULL, one_size);
			if (status != 0) {
				misp_err("%s send fail, block:%d status: %d",
					__func__, count, status);
				break;
			}

			misp_info("%s write block %d success",
				__func__, count);

			count++;
		}
	}
T_EXIT:
	if (filp && (spi_Sendbulkbuffer == NULL))
		set_fs(oldfs);


	/* Free SPI bulk buffer*/
	if (spi_bulkbuffer != 0){
		vfree(spi_bulkbuffer);
		//misp_err("%s - vfree SPI bulk buffer ok",__func__);
      }


	if (status != ERR_SUCCESS)
		misp_err("%s error: %d", __func__, status);
	else
		misp_info("%s success", __func__);

	return status;
}
EXPORT_SYMBOL_GPL(mini_isp_send_bulk);


int mini_isp_get_bulk(struct misp_data *devdata, u8 *response_buf,
		u32 total_size, u32 block_size)
{
	int status = 0, count = 0;
	int remain_size, one_size;
	u8 ctrlbyte = USPICTRL_MS_CB_DIS, ack[2];

	misp_info("%s started.", __func__);

	status = spi_write_then_read(devdata->spi, &ctrlbyte, 1, ack , 2);
	if (status) {
		misp_err("mini_isp_send_bulk send ctrl byte failed. status:%d",
			status);
		status = -EINVAL;
		goto G_EXIT;
	}

	for (remain_size = total_size; remain_size > 0;
		remain_size -= one_size) {
		one_size = (remain_size > block_size) ?
			block_size : remain_size;

		/*get the data*/
		status = mini_isp_spi_sync(devdata->spi, response_buf,
			response_buf, one_size);
		if (status != 0) {
			misp_err("%s failed!! block:%d status: %d",
				__func__, count, status);
			break;
		 }
		 /*misp_info("%s write block %d success.", __func__, count);*/

		response_buf += one_size;
		count++;
	}

G_EXIT:

	if (status != ERR_SUCCESS)
		misp_info("%s - error: %d", __func__, status);
	else
		misp_info("%s - success.", __func__);


	return status;
}
EXPORT_SYMBOL_GPL(mini_isp_get_bulk);




u16 mini_isp_get_status(void)
{
	return statusbyte;
}
EXPORT_SYMBOL_GPL(mini_isp_get_status);

u16 mini_isp_get_currentevent(void)
{
	return current_event;
}
EXPORT_SYMBOL_GPL(mini_isp_get_currentevent);



int mini_isp_get_spi_status(struct misp_data *devdata, u16 *spi_status)
{
	int status;
	u8  statebyte = USPICTRL_MS_CB_STS,
		ack[2];
    misp_info("%s - entering   ", __func__);
	/*get spi status*/
	status = spi_write_then_read(devdata->spi, &statebyte, 1, ack , 2);

	if (status) {
		misp_err("%s - read status error: status=%d",
			__func__, status);
		goto mini_isp_get_spi_status_end;
	}
	memcpy(&statusbyte, ack , 2);
	*spi_status = statusbyte;
	misp_info("%s - statusbyte = %#x", __func__, statusbyte);
mini_isp_get_spi_status_end:

	return status;
}
EXPORT_SYMBOL(mini_isp_get_spi_status);


/****************************************************************************
*							 Private Function																	   *
****************************************************************************/

static int spi_irq_task(void *data)
{
	struct misp_data *devdata = NULL;
	int errcode;
	u16 spi_state;
	unsigned long flags;//ZTEMT: added by congshan 
	

	misp_info("%s - kernel thread start", __func__);
	do {
		/*wait fore irq*/
		mini_isp_wait_for_irq();

		devdata = get_mini_isp_data();

		errcode = mini_isp_get_spi_status(devdata, &spi_state);

		misp_info("%s - read spi register: %#x",
				__func__, spi_state);

		event = MINI_ISP_RCV_WAITING;
		spin_lock_irqsave(&spinlock, flags);//ZTEMT: added by congshan 

		if (errcode == ERR_SUCCESS) {
			if (spi_state & SPI_STATUS_BIT_LOGFULL)
				event = event | MINI_ISP_RCV_LOGFULL;

			if (spi_state & (SPI_STATUS_BIT_CMD_ERR |
				SPI_STATUS_BIT_OTHER_ERROR))
				event = event | MINI_ISP_RCV_ERROR;

			/*error events*/
			if (spi_state & SPI_STATUS_BIT_PWRDWN_READY)
				event = event | MINI_ISP_RCV_PWRDWN;

			/*ready event*/
			if (spi_state & SPI_STATUS_BIT_READY)
				event = event | MINI_ISP_RCV_READY;

			/* streamoff event*/
			if (spi_state & SPI_STATUS_BIT_STRMOFF_READY)
				event = event | MINI_ISP_RCV_STRMOFF;
			
			//ZTEMT: added by congshan start
			if (zte_change_event)
				event = MINI_ISP_RCV_WAITING;
			//ZTEMT: added by congshan end

			wake_up_interruptible(&WAITQ);
		} else {
			misp_err("%s - err: %d", __func__, errcode);

		}
		
		spin_unlock_irqrestore(&spinlock, flags);//ZTEMT: added by congshan 
	} while (!kthread_should_stop());

	misp_info("%s - end", __func__);
	return 0;
}

/*Compatible  node  must  match  dts*/
static const struct of_device_id miniisp_dt_match[] = {
				   {  .compatible  =  "altek,mini_isp",},
				   {  },
				   };

MODULE_DEVICE_TABLE(of, miniisp_dt_match);

static struct class *mini_isp_class;
static struct device *mini_isp_dev;

#define ISP_RESET 0
#define ISP_INT 1
#define ISP_VCC18_CTRL 2
#define ISP_VCC9_CTRL 3

//static struct regulator *isp_vcc;
//static struct clk *isp_clk;

void mini_isp_reset(void)
{
       /////set isp_clk
//       clk_prepare_enable(isp_clk);
//	msleep(50);	
	/* reset mini-isp keep low for at least 200us, release high for 20ms */
	gpio_direction_output(reset_gpio,  0);
	msleep(50);
	gpio_set_value(reset_gpio, 1);
	
	msleep(50);
}
EXPORT_SYMBOL(mini_isp_reset);


int mini_isp_get_chip_id(u32 mini_isp_reg_addr, u8 *id_buf)
{

	int status = 0;
	u8 cont = 0;/*invoke can't get chip ID,dead in while*/
	u8 *send_buffer_spi;
	u8 ctrlbyte = 0x19;
	u32 address = mini_isp_reg_addr;
	struct misp_data *devdata;
	u8 send_buffer_spi_value[64]={0};

	devdata = get_mini_isp_data();
	send_buffer_spi = send_buffer_spi_value;

	//misp_info("mini_isp_drv_setting(4) %s started", __func__);

	memcpy(send_buffer_spi, &ctrlbyte, 1);
	memcpy(send_buffer_spi+1, &address, 4);
	status = mini_isp_spi_sync(devdata->spi, send_buffer_spi,
		send_buffer_spi, 63);

	if (status) {
		misp_err("%s - sync error: status=%d", __func__, status);
		goto mini_isp_get_chip_id_end;
	}

#if 0
	while (cont < 64) {
		misp_info("[miniISP]Get Chip ID[cont=%d] =%x ",cont,*(send_buffer_spi+cont));
		cont++;
	}
	cont = 0;
#endif

	send_buffer_spi = send_buffer_spi+4;
	cont = 4;
	while (*send_buffer_spi == 0x00 && cont < 59) {
		send_buffer_spi++;
		cont++;
	}
#if 1
	misp_info("[miniISP]222Get Chip ID %x %x %x %x",
			*(send_buffer_spi+1), *(send_buffer_spi+2),
			*(send_buffer_spi+3), *(send_buffer_spi+4));
#endif
	if (*send_buffer_spi == 0xa5){
		pr_err("[miniISP]duyuerong Get Chip ID %x %x %x %x",
			*(send_buffer_spi+1), *(send_buffer_spi+2),
			*(send_buffer_spi+3), *(send_buffer_spi+4));
		memcpy(id_buf, send_buffer_spi + 1, 4);
	}else{
		pr_err("[miniISP]Can't get chip ID");
		memcpy(id_buf, send_buffer_spi + 1, 4);
	}
mini_isp_get_chip_id_end:

	return status;
}
EXPORT_SYMBOL(mini_isp_get_chip_id);


void mini_isp_poweron(void)
{
	/* reset mini-isp keep low for at least 200us, release to high for 20ms
	 * */
	g_isMiniISP_sendboot = 1;
	
	zte_change_event = 0; //ZTEMT: added by congshan 
	misp_ts_pinctrl_init(al3200_spi_device, true);
    gpio_set_value(reset_gpio,  0);
    gpio_set_value(vcc1_gpio, 1);
	msleep(1);
    gpio_set_value(vcc2_gpio, 1);
	vcc1v8_gpio_set_value(1);
	al3200_clk_enable(1);
	msleep(4);
    gpio_set_value(reset_gpio, 1);
    msleep(20);
	pr_err("%s\n", __func__);
}
EXPORT_SYMBOL(mini_isp_poweron);

void mini_isp_poweroff(void)
{     
    gpio_set_value(reset_gpio, 0);
    msleep(2);
	al3200_clk_enable(0);
	vcc1v8_gpio_set_value(0);
	msleep(4);
    gpio_set_value(vcc1_gpio,  0);
	msleep(1);
    gpio_set_value(vcc2_gpio,  0);
    g_isMiniISP_bypass = 0;
	g_isMiniISP_sendboot = 1;
	misp_ts_pinctrl_init(al3200_spi_device, false);
	
	zte_change_event = 1;//ZTEMT: added by congshan 
    misp_info("%s - X", __func__);
}
EXPORT_SYMBOL(mini_isp_poweroff);

static ssize_t mini_isp_mode_config_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	/*misp_info("%s - mini_isp_spi_send return %d", __func__, ret);*/
	return sprintf(buf, "load fw:0 e_to_a:1 a_to_e:2\n");;
}

static ssize_t mini_isp_mode_config_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	if ('0' == buf[0]) {
		mini_isp_e_to_a();
		mini_isp_drv_load_fw();
	} else if ('1' == buf[0]) {
		mini_isp_e_to_a();
	} else if ('2' == buf[0]) {
		mini_isp_a_to_e();
	}
	return size;
}


static DEVICE_ATTR(mini_isp_mode_config, 0660, mini_isp_mode_config_show, 
		mini_isp_mode_config_store);


static ssize_t mini_isp_reset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;

	ret = gpio_get_value(reset_gpio);
	misp_info("%s - reset_gpio is %d", __func__, ret);

	return sprintf(buf, "%d", ret);
}

static ssize_t mini_isp_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/*int ret = -EINVAL;*/


	if ('0' == buf[0])
		gpio_set_value(reset_gpio, 0);
	else
		gpio_set_value(reset_gpio, 1);

	misp_info("%s - ", __func__);

	return size;
}

static DEVICE_ATTR(mini_isp_reset, 0660, mini_isp_reset_show,
		mini_isp_reset_store);

static ssize_t mini_isp_poweron_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;

	ret = gpio_get_value(vcc1_gpio);
	misp_info("%s - vcc1_gpio is %d", __func__, ret);
	return sprintf(buf, "%d", ret);
}

static ssize_t mini_isp_poweron_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/*int ret = -EINVAL;*/
	if ('1' == buf[0])
		mini_isp_poweron();
	else
		mini_isp_poweroff();


	misp_info("%s - ", __func__);

	return size;
}

static DEVICE_ATTR(mini_isp_poweron, S_IRUSR | S_IWUSR, mini_isp_poweron_show,
		mini_isp_poweron_store);
		
static int dump_reg_range = 0;
static ssize_t mini_isp_dump_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;
	misp_info("%s - enter", __func__);
	mini_isp_a_to_e();
	if(dump_reg_range == 1)
		ret = mini_isp_drv_read_reg_e_mode_for_bypass_use();
	else
		ret = mini_isp_drv_read_reg_e_mode();

	return sprintf(buf, "dump reg success!!\n");
}

static ssize_t mini_isp_dump_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/*int ret = -EINVAL;*/

	/*0 means dump all reg value, 1 means dump bypass mode reg value */
	if ('0' == buf[0])
		dump_reg_range = 0;
	else
		dump_reg_range = 1;

	misp_info("%s - ", __func__);

	return size;
}

static DEVICE_ATTR(mini_isp_dump_reg, 0660, mini_isp_dump_reg_show,
		mini_isp_dump_reg_store);

static int misp_ts_pinctrl_init(struct spi_device *spi, int on)
{
	int retval = 0;
	struct pinctrl *misp_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;

	struct pinctrl_state *pins_state;

	/* Get pinctrl if target uses pinctrl */
	misp_pinctrl = devm_pinctrl_get(&(spi->dev));
	if (IS_ERR_OR_NULL(misp_pinctrl)) {
		dev_dbg(&spi->dev,
			"Target does not use pinctrl");
		retval = PTR_ERR(misp_pinctrl);
		misp_pinctrl = NULL;
		goto misp_ts_pinctrl_init_end;
	}

	gpio_state_active
		= pinctrl_lookup_state(misp_pinctrl,
			"pmx_ts_active");
	if (IS_ERR_OR_NULL(gpio_state_active)) {
		dev_dbg(&(spi->dev),
			"Can not get ts default pinstate");
		retval = PTR_ERR(gpio_state_active);
		misp_pinctrl = NULL;
		goto misp_ts_pinctrl_init_end;
	}

	gpio_state_suspend
		= pinctrl_lookup_state(misp_pinctrl,
			"pmx_ts_suspend");
	if (IS_ERR_OR_NULL(gpio_state_suspend)) {
		dev_err(&spi->dev,
			"Can not get ts sleep pinstate");
		retval = PTR_ERR(gpio_state_suspend);
		misp_pinctrl = NULL;
		goto misp_ts_pinctrl_init_end;
	}

	pins_state = on ? gpio_state_active
		: gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		retval = pinctrl_select_state(misp_pinctrl, pins_state);
		if (retval) {
			dev_err(&spi->dev,
				"can not set %s pins",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			goto misp_ts_pinctrl_init_end;
		}
	} else {
		dev_err(&spi->dev,
			"not a valid '%s' pinstate",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

misp_ts_pinctrl_init_end:

	return retval;
}

static int mini_isp_probe(struct spi_device *spi)
{
	struct misp_data *drv_data = NULL;
	int ret = 0;
	u8 buf[4];
	int chip_id_flag =-1;

	misp_info("%s - start", __func__);
	printk("chengjiatest: mini isp probe\n");

	/*step 1: alloc driver data struct*/
	drv_data = kmalloc(sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data) {
		ret = -ENOMEM;
		goto mini_isp_probe_out_of_memory;
	}
	al3200_spi_device = spi; //ZTEMT: added by congshan

	misp_info("%s - step1 done.", __func__);

	/*step 2: init driver data*/
	drv_data->spi = spi;
	mutex_init(&drv_data->busy_lock);
	sema_init(&drv_data->cmd_respond, 1);
	misp_info("%s - step2 done.", __func__);

	/*step 3: setup spi*/
	spi->mode = SPI_MODE_3; 
	/*spi->mode = SPI_MODE_1;*/
	spi->max_speed_hz = SPI_BUS_SPEED;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0) {
		misp_err("%s step3. probe - setup spi error", __func__);
		goto err_spi_setup;
	}
	misp_info("%s - step3 done.", __func__);


	/*step 4:config platform_data->GPIO & config platform_data->irq*/

	misp_ts_pinctrl_init(spi, true);

	vcc1_gpio = of_get_named_gpio(spi->dev.of_node, "vcc1-gpios", 0);
	misp_info("%s - probe vcc1-gpios = %d", __func__, vcc1_gpio);

	ret = devm_gpio_request(&spi->dev, vcc1_gpio, "vcc1-gpios");
	if (ret) {
		misp_err("%s -step 4. request vcc1-gpio error", __func__);
		goto err_gpio1_config;
	}


	vcc2_gpio = of_get_named_gpio(spi->dev.of_node, "vcc2-gpios", 0);
	misp_info("%s - probe vcc2-gpios = %d", __func__, vcc2_gpio);

	ret = devm_gpio_request(&spi->dev, vcc2_gpio, "vcc2-gpios");
	if (ret) {
		misp_err("%s -step 4. request vcc2-gpios error", __func__);
		goto err_gpio2_config;
	}

	//ZTEMT: added by congshan start
	vcc1v8_gpio = of_get_named_gpio(spi->dev.of_node, "vcc1v8-gpios", 0);
	misp_info("%s - probe vcc1v8_gpio = %d", __func__, vcc1v8_gpio);
	//ZTEMT: added by congshan end

	reset_gpio = of_get_named_gpio(spi->dev.of_node, "reset-gpios", 0);
	misp_info("%s - probe reset_gpio = %d", __func__, reset_gpio);

	ret = devm_gpio_request(&spi->dev, reset_gpio, "reset-gpios");
	if (ret) {
		misp_err("%s -step 4. request reset gpio error", __func__);
		goto err_gpio_config;
	}

	irq_gpio = of_get_named_gpio(spi->dev.of_node,  "irq-gpios",  0);
	misp_info("%s - probe irq_gpio = %d", __func__, irq_gpio);

	ret = devm_gpio_request(&spi->dev, irq_gpio, "irq-gpios");
	if (ret) {
		misp_err("%s -step 4. request irq gpio error", __func__);
		goto err_reset_config;
	}
	gpio_direction_input(irq_gpio);

	spi->irq = gpio_to_irq(irq_gpio);

	misp_err("%s - probe spi->irq=%d %d ", __func__, spi->irq,
			gpio_to_irq(irq_gpio));

	ret = request_threaded_irq(spi->irq, NULL, mini_isp_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_FALLING, "mini_isp", drv_data);
	if (ret) {
		misp_err("%s - step4. probe - request irq error", __func__);
		goto err_irq_config;
	}
	misp_info("%s - step4 done. irq number:%d", __func__, spi->irq);

	/*step 5:other additional config*/

	misp_info("%s - step5 done", __func__);
	//ZTEMT: added by congshan start
	ret = of_property_read_string(spi->dev.of_node, "clock-names",
					 &al3200_clk_name);
	if (ret < 0) {
		pr_err("%s reading al3200 clock-name failed\n",
			__func__);
	}
	al3200_clk_ptr = devm_clk_get(&spi->dev, al3200_clk_name);
	gpio_direction_output(reset_gpio,  0);
	gpio_direction_output(vcc1_gpio,  1);
	msleep(1);
	gpio_direction_output(vcc2_gpio,  1);
	vcc1v8_gpio_set_value(1);
	al3200_clk_enable(1);
	msleep(4);
	gpio_set_value(reset_gpio,  1);
	msleep(20);
	//ZTEMT: added by congshan end
	/*step 6:kthread for irq*/

	/*spi isr task*/
	spi_task = kthread_run(spi_irq_task, NULL, "spi_irq_thread");
	if (IS_ERR(spi_task)) {
		ret = PTR_ERR(spi_task);
		misp_err("%s - step6 err: %d", __func__, ret);
		goto err_dev_attr;
	}
	misp_info("%s - step6 done.", __func__);
	/*setp last: set driver_data to device*/
	spi_set_drvdata(spi, drv_data);

	misp_drv_data = drv_data;

	misp_info("%s -success", __func__);

	mini_isp_class = class_create(THIS_MODULE, "mini_isp");
	if (IS_ERR(mini_isp_class))
		misp_err("Failed to create class(mini_isp_class)!");

	mini_isp_dev = device_create(mini_isp_class, NULL, 0,
				NULL, "mini_isp_device");
	if (IS_ERR(mini_isp_dev))
		misp_err("Failed to create device(mini_isp_dev)!");

	/*mini_isp */
	ret = device_create_file(mini_isp_dev, &dev_attr_mini_isp_mode_config);
	if (ret < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_mode_config.attr.name);

	ret = device_create_file(mini_isp_dev, &dev_attr_mini_isp_reset);
	if (ret < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_reset.attr.name);

	ret = device_create_file(mini_isp_dev, &dev_attr_mini_isp_poweron);
	if (ret < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_poweron.attr.name);

	ret = device_create_file(mini_isp_dev, &dev_attr_mini_isp_dump_reg);
	if (ret < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_dump_reg.attr.name);

	chip_id_flag = mini_isp_get_chip_id(0xffef0020,buf);
	misp_info("%s -chip_id_flag = %d", __func__,chip_id_flag);
	buf[0] = 1;
	chip_id_flag = 1;
	if(chip_id_flag<0){
		misp_info("%s  enter -chip_id_flag = %d", __func__,chip_id_flag);
           g_isMiniISP_Probled = 0;
		   }
	    else
	     g_isMiniISP_Probled = 1;
      //////power off
	mini_isp_poweroff(); //ZTEMT: added by congshan 
	misp_ts_pinctrl_init(spi, false);//ZTEMT: added by congshan 
	goto mini_isp_probe_end;

mini_isp_probe_out_of_memory:
	misp_err("probe - can not alloc driver data");
	goto mini_isp_probe_end;
err_dev_attr:
	free_irq(spi->irq, drv_data);
err_irq_config:
	gpio_free(irq_gpio);
err_reset_config:
	gpio_free(reset_gpio);
err_gpio_config:
	gpio_free(vcc2_gpio);
err_gpio2_config:
	gpio_free(vcc1_gpio);
err_gpio1_config:
err_spi_setup:
	kfree(drv_data);
	misp_drv_data = NULL;
mini_isp_probe_end:
	return ret;
}


static struct spi_driver mini_isp_drv = {
	.driver = {
		.name   = "mini_isp",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
		.of_match_table = miniisp_dt_match,
	},
	.probe = mini_isp_probe,
	.remove = NULL,
};

static int __init mini_isp_init(void)
{
	int state;

	misp_info("%s - start", __func__);
	state = spi_register_driver(&mini_isp_drv);
	spin_lock_init(&spinlock);//ZTEMT: added by congshan 

	if (state) {
		misp_err("%s - regsiter failed. Errorcode:%d",
			__func__, state);
		goto mini_isp_init_end;
	}
	misp_info("%s - success", __func__);
mini_isp_init_end:
	return state;
}

static void __exit mini_isp_exit(void)
{
	misp_info("%s", __func__);
       //misp_info("duyuerong mini_isp_exit   %s ", __func__);
	free_irq(misp_drv_data->spi->irq, misp_drv_data);
	gpio_free(irq_gpio);
	kfree(misp_drv_data);
	spi_unregister_driver(&mini_isp_drv);

	if (spi_task)
		kthread_stop(spi_task);
}

void mini_isp_send_register_chang_buffer(u32 reg_addr, u32 reg_new_value)
{
	u8 *send_buffer_spi;
	u8 ctrlbyte = 0x09;
	u32 address = reg_addr, value = reg_new_value;
	struct misp_data *devdata;
	u8 send_buffer_spi_value[64];

	//   misp_info("enter mini_isp_send_register_chang_buffer reg_addr =%x value = %x", address,value);

	devdata = get_mini_isp_data();
	send_buffer_spi = send_buffer_spi_value;

	memcpy(send_buffer_spi, &ctrlbyte, 1);
	memcpy(send_buffer_spi+1, &address, 4);
	memcpy(send_buffer_spi+5, &value, 4);
	g_isMiniISP_sendboot = 1;
	mini_isp_spi_sync(devdata->spi, send_buffer_spi, NULL, 9);
	g_isMiniISP_sendboot = 0;
}
EXPORT_SYMBOL(mini_isp_send_register_chang_buffer);

void mini_isp_e_to_a(void)
{
	misp_info("mini_isp_drv_setting(1) mini_isp_e_to_a E");
	mini_isp_send_register_chang_buffer(0xffe40050, 0x00000001);
	mini_isp_send_register_chang_buffer(0xffef0308, 0);
	udelay(70);
	mini_isp_send_register_chang_buffer(0xffef030c, 0);
	mini_isp_send_register_chang_buffer(0xffe81080, 1);
	mini_isp_send_register_chang_buffer(0xffef0088, 0xffbf7fff);
	mini_isp_send_register_chang_buffer(0xffe800c4, 0xfffffff0);
	udelay(100);
	mini_isp_send_register_chang_buffer(0xffef0240, 0);
	misp_info("mini_isp_drv_setting(1) mini_isp_e_to_a X");
}
EXPORT_SYMBOL(mini_isp_e_to_a);

void mini_isp_a_to_e(void)
{
	u8 *send_buffer_spi;
	u8 ctrlbyte = 0x20;

	struct misp_data *devdata;
	u8 send_buffer_spi_value[64];

	misp_info("mini_isp_drv_setting(2) mini_isp_a_to_e E");
	devdata = get_mini_isp_data();
	send_buffer_spi = send_buffer_spi_value;

	memcpy(send_buffer_spi, &ctrlbyte, 1);
	mini_isp_spi_sync(devdata->spi, send_buffer_spi, NULL, 1);
	misp_info("mini_isp_drv_setting(2) mini_isp_e_to_a X");
}
EXPORT_SYMBOL(mini_isp_a_to_e);

void mini_isp_pure_bypass(u16 mini_isp_mode)
{

	misp_info("mini_isp_drv_setting(%x) set bypass mode", mini_isp_mode);
       if(mini_isp_mode == g_isMiniISP_bypass)
	   return;
	   else 
	   	g_isMiniISP_bypass = mini_isp_mode;
	/*CPU on*/
	// change to 32bit
	
	mini_isp_send_register_chang_buffer(0xffe40050, 0x1);
	/* access general_reg io power on d2*/
	mini_isp_send_register_chang_buffer(0xffef0308, 0x0);
	/* delay 70us*/
	udelay(70);
	/* access general_reg io iso on d2*/
	mini_isp_send_register_chang_buffer(0xffef030c, 0x0);
	/* access clk_gen io*/
	mini_isp_send_register_chang_buffer(0xffe81080, 0x1);
	/*access gen_reg io for release uspi_e/base_arc sram pd*/
	mini_isp_send_register_chang_buffer(0xffef0088, 0xffbf7fff);
	/*access clk_gen io release cpu clk/rst*/
	mini_isp_send_register_chang_buffer(0xffe800c4, 0xfffffff0);
	udelay(10);
	switch (mini_isp_mode){
	case MINI_ISP_MODE_BYPASS:
		misp_info("enter bypass mode   %x ", mini_isp_mode);
#if 1
           /*========================================*/
           /*===== ID:0  Dual CAM FR Bypass mode   */
           /*===== MCK=19.20MHZ   */
           /*===== RX0=1296.00Mbps  4 Lane    */
           /*===== RX1=1296.00Mbps  4 Lane    */
           /*========================================*/
           /* SCID=0*/
           /*===== General Reg control  =====*/
           mini_isp_send_register_chang_buffer(0xffe40000, 0x00000008);
           mini_isp_send_register_chang_buffer(0xffe40004, 0x00006621);
           mini_isp_send_register_chang_buffer(0xffe40008, 0x00006621);
           mini_isp_send_register_chang_buffer(0xffe4000c, 0x00006621);
           mini_isp_send_register_chang_buffer(0xffe40010, 0x00006621);
           mini_isp_send_register_chang_buffer(0xffe40050, 0x00000001);
           /*===== PLL Setting 0+1 =====*/
           mini_isp_send_register_chang_buffer(0xffef00f4, 0x00000001);
           mini_isp_send_register_chang_buffer(0xffe81004, 0x00000400);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffe81040, 0x00000236);
           mini_isp_send_register_chang_buffer(0xffe81044, 0x0000011b);
           mini_isp_send_register_chang_buffer(0xffe81048, 0x00000103);
           mini_isp_send_register_chang_buffer(0xffe81004, 0x00000000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffe800e0, 0x0000080b);
           mini_isp_send_register_chang_buffer(0xffe80100, 0x0000080b);
           mini_isp_send_register_chang_buffer(0xffe80120, 0x0000080b);
           mini_isp_send_register_chang_buffer(0xffe81004, 0x00000800);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffe81058, 0x0000037c);
           mini_isp_send_register_chang_buffer(0xffe8105c, 0x000001be);
           mini_isp_send_register_chang_buffer(0xffe81060, 0x00000105);
           mini_isp_send_register_chang_buffer(0xffe81004, 0x00000000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffe80b00, 0x00000842);
           mini_isp_send_register_chang_buffer(0xffe80880, 0x00000800);
           mini_isp_send_register_chang_buffer(0xffe80940, 0x00000800);
           mini_isp_send_register_chang_buffer(0xffe808c0, 0x00000800);
           mini_isp_send_register_chang_buffer(0xffe80980, 0x00000800);
           mini_isp_send_register_chang_buffer(0xffe80400, 0x00000802);
           mini_isp_send_register_chang_buffer(0xffe80440, 0x00000802);
           /*===== Clock control 0+1 =====*/
           mini_isp_send_register_chang_buffer(0xffe80404, 0x000000ca);
           mini_isp_send_register_chang_buffer(0xffe80404, 0x000000c0);
           mini_isp_send_register_chang_buffer(0xffe80444, 0x000000ca);
           mini_isp_send_register_chang_buffer(0xffe80444, 0x000000c0);
           mini_isp_send_register_chang_buffer(0xffe80484, 0x00000002);
           mini_isp_send_register_chang_buffer(0xffe80484, 0x00000000);
           mini_isp_send_register_chang_buffer(0xffe804a4, 0x00000002);
           mini_isp_send_register_chang_buffer(0xffe804a4, 0x00000000);
           mini_isp_send_register_chang_buffer(0xffe804c4, 0x00000002);
           mini_isp_send_register_chang_buffer(0xffe804c4, 0x00000000);
           mini_isp_send_register_chang_buffer(0xffe804e4, 0x00000002);
           mini_isp_send_register_chang_buffer(0xffe804e4, 0x00000000);
           mini_isp_send_register_chang_buffer(0xffe80884, 0x00000002);
           mini_isp_send_register_chang_buffer(0xffe80884, 0x00000000);
           mini_isp_send_register_chang_buffer(0xffe80944, 0x00000002);
           mini_isp_send_register_chang_buffer(0xffe80944, 0x00000000);
           mini_isp_send_register_chang_buffer(0xffe80b04, 0x00002a2a);
           mini_isp_send_register_chang_buffer(0xffe80b04, 0x00002828);
           /*===== MTX 0 (bitrate=1360Mbps) =====*/
           mini_isp_send_register_chang_buffer(0xffed1044, 0x000000b8);
           mini_isp_send_register_chang_buffer(0xffed1044, 0x00000098);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed1044, 0x00000088);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed1030, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed1030, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed1034, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed1034, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed1038, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed1038, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed103c, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed103c, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed1040, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed1040, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed1008, 0x00000b12);
           mini_isp_send_register_chang_buffer(0xffed100c, 0x0000020b);
           mini_isp_send_register_chang_buffer(0xffed1010, 0x0000002a);
           mini_isp_send_register_chang_buffer(0xffed1014, 0x00000f0b);
           mini_isp_send_register_chang_buffer(0xffed1018, 0x00000a0b);
           mini_isp_send_register_chang_buffer(0xffed101c, 0x00000a13);
           mini_isp_send_register_chang_buffer(0xffed1000, 0x00000000);
           /*===== PPIB 0 =====*/
           mini_isp_send_register_chang_buffer(0xfff97004, 0x02003210);
           mini_isp_send_register_chang_buffer(0xfff97008, 0x00003210);
           mini_isp_send_register_chang_buffer(0xfff9700c, 0x14500090);
           mini_isp_send_register_chang_buffer(0xfff97010, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff97024, 0x0000002a);
           mini_isp_send_register_chang_buffer(0xfff97028, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff9702c, 0x0000000c);
           mini_isp_send_register_chang_buffer(0xfff97030, 0x14141414);
           mini_isp_send_register_chang_buffer(0xfff97000, 0x00000000);
           /*===== MRX 0 =====*/
           mini_isp_send_register_chang_buffer(0xfff91000, 0x1000000b);
           mini_isp_send_register_chang_buffer(0xfff91004, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff91008, 0x0401f033);
           mini_isp_send_register_chang_buffer(0xfff91010, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff91014, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff91024, 0x0000000f);
           mini_isp_send_register_chang_buffer(0xfff91028, 0x00003a3a);
           mini_isp_send_register_chang_buffer(0xfff9103c, 0x0000003f);
           mini_isp_send_register_chang_buffer(0xfff91040, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff9104c, 0x00000011);
           mini_isp_send_register_chang_buffer(0xfff91068, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff9106c, 0x00002020);
           mini_isp_send_register_chang_buffer(0xfff91000, 0x1000000a);
           /*===== MTX 1 (bitrate=1360Mbps) =====*/
           mini_isp_send_register_chang_buffer(0xffed6044, 0x000000b8);
           mini_isp_send_register_chang_buffer(0xffed6044, 0x00000098);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed6044, 0x00000088);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed6030, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed6030, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed6034, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed6034, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed6038, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed6038, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed603c, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed603c, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed6040, 0x00080000);
           udelay(350);
           mini_isp_send_register_chang_buffer(0xffed6040, 0x00080002);
           mini_isp_send_register_chang_buffer(0xffed6008, 0x00000b12);
           mini_isp_send_register_chang_buffer(0xffed600c, 0x0000020b);
           mini_isp_send_register_chang_buffer(0xffed6010, 0x0000002a);
           mini_isp_send_register_chang_buffer(0xffed6014, 0x00000f0b);
           mini_isp_send_register_chang_buffer(0xffed6018, 0x00000a0b);
           mini_isp_send_register_chang_buffer(0xffed601c, 0x00000a13);
           mini_isp_send_register_chang_buffer(0xffed6000, 0x00000000);
           /*===== PPIB 1 =====*/
           mini_isp_send_register_chang_buffer(0xfff98004, 0x02003210);
           mini_isp_send_register_chang_buffer(0xfff98008, 0x00003210);
           mini_isp_send_register_chang_buffer(0xfff9800c, 0x14500090);
           mini_isp_send_register_chang_buffer(0xfff98010, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff98024, 0x0000002a);
           mini_isp_send_register_chang_buffer(0xfff98028, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff9802c, 0x0000000c);
           mini_isp_send_register_chang_buffer(0xfff98030, 0x14141414);
           mini_isp_send_register_chang_buffer(0xfff98000, 0x00000000);
           /*===== MRX 1 =====*/
           mini_isp_send_register_chang_buffer(0xfff94000, 0x1000000b);
           mini_isp_send_register_chang_buffer(0xfff94004, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff94008, 0x0401f033);
           mini_isp_send_register_chang_buffer(0xfff94010, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff94014, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff94024, 0x0000000f);
           mini_isp_send_register_chang_buffer(0xfff94028, 0x00003a3a);
           mini_isp_send_register_chang_buffer(0xfff9403c, 0x0000003f);
           mini_isp_send_register_chang_buffer(0xfff94040, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff9404c, 0x00000011);
           mini_isp_send_register_chang_buffer(0xfff94068, 0x00000000);
           mini_isp_send_register_chang_buffer(0xfff9406c, 0x00002020);
           mini_isp_send_register_chang_buffer(0xfff94000, 0x1000000a);
#else
		/*========================================*/
		/*===== ID:0  Dual CAM FR Bypass mode   */
		/*===== MCK=19.20MHZ   */
		/*===== RX0=996.00Mbps  4 Lane    */
		/*===== RX1=996.00Mbps  4 Lane    */
		/*========================================*/
		/* SCID=0*/
		/*===== General Reg control  =====*/
		mini_isp_send_register_chang_buffer(0xffe40000, 0x00000008);
		mini_isp_send_register_chang_buffer(0xffe40004, 0x00006621);
		mini_isp_send_register_chang_buffer(0xffe40008, 0x00006621);
		mini_isp_send_register_chang_buffer(0xffe4000c, 0x00006621);
		mini_isp_send_register_chang_buffer(0xffe40010, 0x00006621);
		mini_isp_send_register_chang_buffer(0xffe40050, 0x00000001);
		/*===== PLL Setting 0+1 =====*/
		mini_isp_send_register_chang_buffer(0xffef00f4, 0x00000001);
		mini_isp_send_register_chang_buffer(0xffe81004, 0x00000400);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffe81040, 0x00000a36);
		mini_isp_send_register_chang_buffer(0xffe81044, 0x0000051b);
		mini_isp_send_register_chang_buffer(0xffe81048, 0x00000117);
		mini_isp_send_register_chang_buffer(0xffe81004, 0x00000000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffe800e0, 0x00000808);
		mini_isp_send_register_chang_buffer(0xffe80100, 0x00000808);
		mini_isp_send_register_chang_buffer(0xffe80120, 0x00000808);
		mini_isp_send_register_chang_buffer(0xffe81004, 0x00000800);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffe81058, 0x0000008e);
		mini_isp_send_register_chang_buffer(0xffe8105c, 0x00000047);
		mini_isp_send_register_chang_buffer(0xffe81060, 0x00000100);
		mini_isp_send_register_chang_buffer(0xffe81004, 0x00000000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffe80b00, 0x00000832);
		mini_isp_send_register_chang_buffer(0xffe80880, 0x00000800);
		mini_isp_send_register_chang_buffer(0xffe80940, 0x00000800);
		mini_isp_send_register_chang_buffer(0xffe808c0, 0x00000800);
		mini_isp_send_register_chang_buffer(0xffe80980, 0x00000800);
		mini_isp_send_register_chang_buffer(0xffe80400, 0x00000802);
		mini_isp_send_register_chang_buffer(0xffe80440, 0x00000802);
		/*===== Clock control 0+1 =====*/
		mini_isp_send_register_chang_buffer(0xffe80404, 0x000000ca);
		mini_isp_send_register_chang_buffer(0xffe80404, 0x000000c0);
		mini_isp_send_register_chang_buffer(0xffe80444, 0x000000ca);
		mini_isp_send_register_chang_buffer(0xffe80444, 0x000000c0);
		mini_isp_send_register_chang_buffer(0xffe80484, 0x00000002);
		mini_isp_send_register_chang_buffer(0xffe80484, 0x00000000);
		mini_isp_send_register_chang_buffer(0xffe804a4, 0x00000002);
		mini_isp_send_register_chang_buffer(0xffe804a4, 0x00000000);
		mini_isp_send_register_chang_buffer(0xffe804c4, 0x00000002);
		mini_isp_send_register_chang_buffer(0xffe804c4, 0x00000000);
		mini_isp_send_register_chang_buffer(0xffe804e4, 0x00000002);
		mini_isp_send_register_chang_buffer(0xffe804e4, 0x00000000);
		mini_isp_send_register_chang_buffer(0xffe80884, 0x00000002);
		mini_isp_send_register_chang_buffer(0xffe80884, 0x00000000);
		mini_isp_send_register_chang_buffer(0xffe80944, 0x00000002);
		mini_isp_send_register_chang_buffer(0xffe80944, 0x00000000);
		mini_isp_send_register_chang_buffer(0xffe80b04, 0x00002a2a);
		mini_isp_send_register_chang_buffer(0xffe80b04, 0x00002828);
		/*===== MTX 0 (bitrate=1046Mbps) =====*/
		mini_isp_send_register_chang_buffer(0xffed1044, 0x000000b8);
		mini_isp_send_register_chang_buffer(0xffed1044, 0x00000098);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed1044, 0x00000088);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed1030, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed1030, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed1034, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed1034, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed1038, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed1038, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed103c, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed103c, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed1040, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed1040, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed1008, 0x00000810);
		mini_isp_send_register_chang_buffer(0xffed100c, 0x00000208);
		mini_isp_send_register_chang_buffer(0xffed1010, 0x00000021);
		mini_isp_send_register_chang_buffer(0xffed1014, 0x00000d08);
		mini_isp_send_register_chang_buffer(0xffed1018, 0x00000809);
		mini_isp_send_register_chang_buffer(0xffed101c, 0x0000080f);
		mini_isp_send_register_chang_buffer(0xffed1000, 0x00000000);
		/*===== PPIB 0 =====*/
		mini_isp_send_register_chang_buffer(0xfff97004, 0x02003210);
		mini_isp_send_register_chang_buffer(0xfff97008, 0x00003210);
		mini_isp_send_register_chang_buffer(0xfff9700c, 0x145000ac);
		mini_isp_send_register_chang_buffer(0xfff97010, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff97024, 0x00000023);
		mini_isp_send_register_chang_buffer(0xfff97028, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff9702c, 0x0000000a);
		mini_isp_send_register_chang_buffer(0xfff97030, 0x0d0d0d0d);
		mini_isp_send_register_chang_buffer(0xfff97000, 0x00000000);
		/*===== MRX 0 =====*/
		mini_isp_send_register_chang_buffer(0xfff91000, 0x1000000b);
		mini_isp_send_register_chang_buffer(0xfff91004, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff91008, 0x0401f033);
		mini_isp_send_register_chang_buffer(0xfff91010, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff91014, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff91024, 0x0000000f);
		mini_isp_send_register_chang_buffer(0xfff91028, 0x00002d2d);
		mini_isp_send_register_chang_buffer(0xfff9103c, 0x0000003f);
		mini_isp_send_register_chang_buffer(0xfff91040, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff9104c, 0x00000011);
		mini_isp_send_register_chang_buffer(0xfff91068, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff9106c, 0x00001919);
		mini_isp_send_register_chang_buffer(0xfff91000, 0x1000000a);
		/*===== MTX 1 (bitrate=1046Mbps) =====*/
		mini_isp_send_register_chang_buffer(0xffed6044, 0x000000b8);
		mini_isp_send_register_chang_buffer(0xffed6044, 0x00000098);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed6044, 0x00000088);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed6030, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed6030, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed6034, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed6034, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed6038, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed6038, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed603c, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed603c, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed6040, 0x00080000);
		udelay(350);
		mini_isp_send_register_chang_buffer(0xffed6040, 0x00080002);
		mini_isp_send_register_chang_buffer(0xffed6008, 0x00000810);
		mini_isp_send_register_chang_buffer(0xffed600c, 0x00000208);
		mini_isp_send_register_chang_buffer(0xffed6010, 0x00000021);
		mini_isp_send_register_chang_buffer(0xffed6014, 0x00000d08);
		mini_isp_send_register_chang_buffer(0xffed6018, 0x00000809);
		mini_isp_send_register_chang_buffer(0xffed601c, 0x0000080f);
		mini_isp_send_register_chang_buffer(0xffed6000, 0x00000000);
		/*===== PPIB 1 =====*/
		mini_isp_send_register_chang_buffer(0xfff98004, 0x02003210);
		mini_isp_send_register_chang_buffer(0xfff98008, 0x00003210);
		mini_isp_send_register_chang_buffer(0xfff9800c, 0x145000ac);
		mini_isp_send_register_chang_buffer(0xfff98010, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff98024, 0x00000023);
		mini_isp_send_register_chang_buffer(0xfff98028, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff9802c, 0x0000000a);
		mini_isp_send_register_chang_buffer(0xfff98030, 0x0d0d0d0d);
		mini_isp_send_register_chang_buffer(0xfff98000, 0x00000000);
		/*===== MRX 1 =====*/
		mini_isp_send_register_chang_buffer(0xfff94000, 0x1000000b);
		mini_isp_send_register_chang_buffer(0xfff94004, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff94008, 0x0401f033);
		mini_isp_send_register_chang_buffer(0xfff94010, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff94014, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff94024, 0x0000000f);
		mini_isp_send_register_chang_buffer(0xfff94028, 0x00002d2d);
		mini_isp_send_register_chang_buffer(0xfff9403c, 0x0000003f);
		mini_isp_send_register_chang_buffer(0xfff94040, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff9404c, 0x00000011);
		mini_isp_send_register_chang_buffer(0xfff94068, 0x00000000);
		mini_isp_send_register_chang_buffer(0xfff9406c, 0x00001919);
		mini_isp_send_register_chang_buffer(0xfff94000, 0x1000000a);
        #endif
		break;

	default:
		break;
	}
}
EXPORT_SYMBOL(mini_isp_pure_bypass);

u32 mini_isp_register_memory_read(u32 start_reg_addr, u32 end_reg_addr)
{
	u32 count;
	struct misp_data *devdata;
	u8 *send_buffer_spi;
	u8 send_buffer_spi_value[128];
	u8   *allocated_memmory	   = 0;
	u32  ouput_size;
	u8 ctrlbyte;
	u32 get_count = 0;
	u8   *keep_allocated_memmory;
	u8 filename[80];
	struct file *f;
	mm_segment_t fs;

	//misp_info("duyuerong enter mini_isp_register_memory_read step 1");

	allocated_memmory = kzalloc(4 * 1024, GFP_KERNEL);
	keep_allocated_memmory = allocated_memmory;
	if (!allocated_memmory)
		goto allocate_memory_fail;

	count = ((end_reg_addr - start_reg_addr) / 4) + 1;
	ouput_size = count + 2;
	devdata = get_mini_isp_data();
	//misp_info("duyuerong enter mini_isp_register_memory_read step 2");
	send_buffer_spi = send_buffer_spi_value;
	msleep(1000);
	memset(send_buffer_spi, 0, SPI_TX_BUF_SIZE);
	memcpy(keep_allocated_memmory, &start_reg_addr, 4);
	keep_allocated_memmory = keep_allocated_memmory + 4;
	memcpy(keep_allocated_memmory, &count, 4);
	keep_allocated_memmory = keep_allocated_memmory + 4;
	while (start_reg_addr <= end_reg_addr) {
		get_count = 0;
		send_buffer_spi = send_buffer_spi_value;
		ctrlbyte = 0x19;
		memset(send_buffer_spi, 0, SPI_TX_BUF_SIZE);
		memcpy(send_buffer_spi, &ctrlbyte, 1);
		memcpy(send_buffer_spi + 1, &start_reg_addr, 4);		
		 
		mini_isp_spi_sync(devdata->spi, send_buffer_spi,
			send_buffer_spi, 64);
		 
		while (*send_buffer_spi == 0x00 && get_count < 63) {
			send_buffer_spi++;
			get_count++;
		}
		 ///misp_info("duyuerong enter mini_isp_register_memory_read step 4");
		 /*
		 misp_info("duyuerong%s pass and write  end *** %x- %x ---%x -%x %x %x %x",
		 	__func__, start_reg_addr, get_count,
		 	*(send_buffer_spi), *(send_buffer_spi + 1),
		 	*(send_buffer_spi + 2), *(send_buffer_spi + 3),
		 	*(send_buffer_spi + 4));
		 */
		if (*send_buffer_spi == 0xa5)
			memcpy(keep_allocated_memmory, send_buffer_spi + 1, 4);
		start_reg_addr = start_reg_addr + 4;
		keep_allocated_memmory = keep_allocated_memmory + 4;
	}
#if 1
      //misp_info("duyuerongenter mini_isp_register_memory_read step 5"); 
	  
	sprintf(filename, "%s/disp_0x%x-0x%x.regx",
		MINIISP_INFO_DUMPLOCATION, end_reg_addr - (count - 1) * 4,
		end_reg_addr);
	//misp_info("duyuerongenter mini_isp_register_memory_read step 6");
	f = filp_open(filename, O_APPEND | O_CREAT, 0777);
	if (IS_ERR(f)) {
	    pr_err("%s cannot creat file \n", __func__);
		goto mini_isp_register_memory_read_end;
	}	
	/*Get current segment descriptor*/
	fs = get_fs();
	//misp_info("duyuerongenter mini_isp_register_memory_read step 7");
	/*Set segment descriptor associated*/
	set_fs(get_ds());
	//misp_info("duyuerongenter mini_isp_register_memory_read step 8");
	/*write the file*/
	f->f_op->write(f, (char *)allocated_memmory, ouput_size * 4, &f->f_pos);
	/*Restore segment descriptor*/
	set_fs(fs);
	//misp_info("enter mini_isp_register_memory_read step 9");
	filp_close(f, NULL);
	kfree(allocated_memmory);
#endif
	goto mini_isp_register_memory_read_end;
allocate_memory_fail:
	kfree(allocated_memmory);
	misp_err("%s Allocate memory failed.", __func__);
mini_isp_register_memory_read_end:
	return ~0;
}
EXPORT_SYMBOL(mini_isp_register_memory_read);

module_init(mini_isp_init);
module_exit(mini_isp_exit);
MODULE_LICENSE("Dual BSD/GPL");
