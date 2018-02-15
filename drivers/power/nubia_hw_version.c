/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <nubia_hw_version.h>
#include <linux/io.h>


#ifdef CONFIG_NUBIA_HW_VERSION_DEBUG
static int debug_value=1;
#else
static int debug_value=0;
#endif
#define nubia_hw_version_debug(fmt, args...) do {if(debug_value==1)printk(KERN_DEBUG "[nubia_hw_version]"fmt, ##args);} while(0)

#ifdef CONFIG_NUBIA_HW_VER_BY_ADC
static int nubia_hw_pcb_mv = 0;
#else
static char nubia_hw_pcb_gpio_str[10]="0,0";
#endif
static char nubia_hw_rf_gpio_str[10]="0,0";
#ifdef CONFIG_NUBIA_HW_CONFIG_BY_GPIO
static char nubia_hw_config_gpio_str[10]="0,0";
#else
static int nubia_hw_config_mv = 0;
#endif

#ifdef CONFIG_NUBIA_HW_VER_BY_ADC
static int  nubia_hw_pcb_setup_config_adc(char *param)
{
	int magic_num = 0;
    get_option(&param, &magic_num);
	nubia_hw_pcb_mv = magic_num;
    return 0;
}
early_param("pcb_setup", nubia_hw_pcb_setup_config_adc);
#else
static int  nubia_hw_pcb_setup_config_gpio(char *param)
{
    memcpy(nubia_hw_pcb_gpio_str, param, strlen(param));
    return 0;
}
early_param("pcb_setup", nubia_hw_pcb_setup_config_gpio);
#endif

static int  nubia_hw_rf_setup_config_gpio(char *param)
{
    memcpy(nubia_hw_rf_gpio_str, param, strlen(param));
    return 0;
}
early_param("rf_setup", nubia_hw_rf_setup_config_gpio);

#ifdef CONFIG_NUBIA_HW_CONFIG_BY_GPIO
static int  nubia_hw_config_setup_config_gpio(char *param)
{
    memcpy(nubia_hw_config_gpio_str, param, strlen(param));
    return 0;
}
early_param("config_setup", nubia_hw_config_setup_config_gpio);
#else
static int  nubia_hw_config_setup_config_adc(char *param)
{
	int magic_num = 0;
    get_option(&param, &magic_num);
	nubia_hw_config_mv = magic_num;
    return 0;
}
early_param("config_setup", nubia_hw_config_setup_config_adc);
#endif

//开始进入第一大部分-读取pcb版本部分
#ifdef CONFIG_NUBIA_HW_VER_BY_ADC
const struct hw_pcb_adc_map_str* nubia_get_pcb_table_item_by_adc(const struct hw_pcb_adc_map_str *pts,
	   uint32_t tablesize, int input)
{
    uint32_t i = 0;

	if ( NULL == pts )
	{
	  return -EINVAL;
	}

	while (i < tablesize)
	{
		if ( (pts[i].low_mv <= input) && (input <= pts[i].high_mv) )
			break;
		else
			i++;
	}

	if ( i < tablesize )
		return &pts[i];
    else
		return NULL;
}
#else
const struct hw_pcb_gpio_map_str* nubia_get_pcb_table_item_by_gpio(const struct hw_pcb_gpio_map_str *pts,
		uint32_t tablesize, char * gpio_str)
{
	uint32_t i = 0;
    char * gpio_str_temp = gpio_str;
	int gpio_value_A = 0;
	int gpio_value_B = 0;

	if ( NULL == pts || NULL == gpio_str )
	{
		return NULL;
	}

	sscanf(gpio_str_temp ,"%d,%d", &gpio_value_A, &gpio_value_B);

	while (i < tablesize)
	{
		if ( (pts[i].gpio_A == gpio_value_A) && (pts[i].gpio_B == gpio_value_B))
			break;
		else
			i++;
	}

	if ( i < tablesize )
		return &pts[i];
	else
		return NULL;
}
#endif
//读取pcb type，给其他驱动模块使用
int nubia_get_hw_id(void)
{
#ifdef CONFIG_NUBIA_HW_VER_BY_ADC
    const struct hw_pcb_adc_map_str *pts_item;
    pts_item = nubia_get_pcb_table_item_by_adc(hw_pcb_adc_map,
					ARRAY_SIZE(hw_pcb_adc_map),
					nubia_hw_pcb_mv);
#else
	const struct hw_pcb_gpio_map_str *pts_item;
	pts_item = nubia_get_pcb_table_item_by_gpio(hw_pcb_gpio_map,
                        ARRAY_SIZE(hw_pcb_gpio_map),
                        nubia_hw_pcb_gpio_str);
#endif
    if ( NULL != pts_item){
		nubia_hw_version_debug("pcb_type=%x\n", pts_item->pcb_type);
		return pts_item->pcb_type;
	}
	else
		return HW_UNKNOW;
}
EXPORT_SYMBOL_GPL(nubia_get_hw_id);

//读取pcb版本，给其他驱动模块使用
void nubia_get_hw_pcb_version(char* result)
{
#ifdef CONFIG_NUBIA_HW_VER_BY_ADC
	const struct hw_pcb_adc_map_str *pts_item;
	pts_item = nubia_get_pcb_table_item_by_adc(hw_pcb_adc_map,
						ARRAY_SIZE(hw_pcb_adc_map),
						nubia_hw_pcb_mv);
#else
	const struct hw_pcb_gpio_map_str *pts_item;
	pts_item = nubia_get_pcb_table_item_by_gpio(hw_pcb_gpio_map,
							ARRAY_SIZE(hw_pcb_gpio_map),
							nubia_hw_pcb_gpio_str);
#endif
    if(!result)
		return;

    if(NULL != pts_item){
        strcpy(result,pts_item->pcb_ver);
     }else
	    sprintf(result, "%s","unknow");
}
EXPORT_SYMBOL_GPL(nubia_get_hw_pcb_version);

//读取pcb版本，给上层同事使用
static ssize_t nubia_hw_pcb_version_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
    if ( NULL == buf )
	{
	  return 0;
    }
    nubia_get_hw_pcb_version(buf);
    nubia_hw_version_debug("pcb_ver=%s\n",buf);
    return sprintf(buf,"%s",buf);
}
static struct kobj_attribute pcb_version_attr=
    __ATTR(pcb_version, 0664, nubia_hw_pcb_version_show, NULL);

//读取Pcb部分结束

//开始进入第二部分-读取rf部分
const char* nubia_get_rf_band_by_gpio(const struct hw_rf_band_gpio_map_str *pts,
		uint32_t tablesize, char * gpio_str)
{
	uint32_t i = 0;
    char * gpio_str_temp = gpio_str;
	int gpio_value_A = 0;
	int gpio_value_B = 0;


	if ( NULL == pts || NULL == gpio_str )
	{
	  return "unknow";
	}

	sscanf(gpio_str_temp ,"%d,%d", &gpio_value_A, &gpio_value_B);

	while (i < tablesize)
	{
		if ( (pts[i].gpio_A == gpio_value_A) && (pts[i].gpio_B == gpio_value_B))
			break;
		else
			i++;
	}

	if ( i < tablesize )
		return pts[i].rf_band;
	else
		return "unknow";

}
EXPORT_SYMBOL_GPL(nubia_get_rf_band_by_gpio);

//读取射频类型给上层使用
static ssize_t nubia_hw_rf_band_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	const char* rf_buf;

	if ( NULL == buf )
	{
		return 0;
	}

	rf_buf = nubia_get_rf_band_by_gpio(hw_rf_band_gpio_map,
                        ARRAY_SIZE(hw_rf_band_gpio_map),
	                        nubia_hw_rf_gpio_str);

	nubia_hw_version_debug("hw_rf_band=%s\n", rf_buf);

	return sprintf(buf,"%s",rf_buf);
}

static struct kobj_attribute hw_rf_band_attr=
    __ATTR(operators, 0664, nubia_hw_rf_band_show, NULL);

//读取射频频段完成

//开始读取第三大部分-软件相关的一些配置
#ifdef CONFIG_NUBIA_HW_CONFIG_BY_GPIO
const struct hw_config_gpio_map_st* nubia_get_config_table_item_by_gpio(const struct hw_config_gpio_map_st *pts,
		uint32_t tablesize, char * gpio_str)
{
	uint32_t i = 0;
    char * gpio_str_temp = gpio_str;
	int gpio_value_A = 0;
	int gpio_value_B = 0;


	if ( NULL == pts || NULL == gpio_str )
	{
		return NULL;
	}

	sscanf(gpio_str_temp ,"%d,%d", &gpio_value_A, &gpio_value_B);

	while (i < tablesize)
	{
		if ( (pts[i].gpio_A == gpio_value_A) && (pts[i].gpio_B == gpio_value_B))
			break;
		else
			i++;
	}

	if ( i < tablesize )
		return &pts[i];
	else
		return NULL;

}
#else
const struct hw_config_adc_map_st* nubia_get_config_table_item_by_adc(const struct hw_config_adc_map_st *pts,
	   uint32_t tablesize, int input)
{
    uint32_t i = 0;

	if ( NULL == pts )
	{
	  return NULL;
	}

	while (i < tablesize)
	{
		if ( (pts[i].low_mv <= input) && (input <= pts[i].high_mv) )
			break;
		else
			i++;
	}

	if ( i < tablesize )
		return &pts[i];
    else
		return NULL;

}
#endif

//读取wifi 类型
const char* nubia_get_hw_wifi(void)
{
#ifdef CONFIG_NUBIA_HW_CONFIG_BY_GPIO
	const struct hw_config_gpio_map_st *pts_item;
	pts_item = nubia_get_config_table_item_by_gpio(hw_config_gpio_map,
                        ARRAY_SIZE(hw_config_gpio_map),
	                        nubia_hw_config_gpio_str);
#else
    const struct hw_config_adc_map_st *pts_item;
	pts_item = nubia_get_config_table_item_by_adc(hw_config_adc_map,
						ARRAY_SIZE(hw_config_adc_map),
						nubia_hw_config_mv);
#endif

	if ( NULL != pts_item){
		nubia_hw_version_debug("wifi_type=%s\n", pts_item->wifi_type);
		return pts_item->wifi_type;
	}
	else
		return "unknow";
}
EXPORT_SYMBOL_GPL(nubia_get_hw_wifi);


//读取配置标准，如高配，低配等
void nubia_get_config_standard(char* result)
{
#ifdef CONFIG_NUBIA_HW_CONFIG_BY_GPIO
	const struct hw_config_gpio_map_st *pts_item;
	pts_item = nubia_get_config_table_item_by_gpio(hw_config_gpio_map,
							ARRAY_SIZE(hw_config_gpio_map),
								nubia_hw_config_gpio_str);
#else
	const struct hw_config_adc_map_st *pts_item;
	pts_item = nubia_get_config_table_item_by_adc(hw_config_adc_map,
							ARRAY_SIZE(hw_config_adc_map),
							nubia_hw_config_mv);
#endif

    if(!result)
		return;

    if(pts_item != NULL){
        strcpy(result,pts_item->config_type);
     }else
	    sprintf(result, "%s","unknow");
}
EXPORT_SYMBOL_GPL(nubia_get_config_standard);


static ssize_t nubia_config_standard_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	if (NULL == buf)
	{
	  return 0;
	}

    nubia_get_config_standard(buf);

	nubia_hw_version_debug("config_type=%s\n", buf);

	return sprintf(buf,"%s",buf);
}

static struct kobj_attribute config_standard_attr=
    __ATTR(config, 0664, nubia_config_standard_show, NULL);


//调试值的读取和写入
static ssize_t debug_value_store(struct kobject *kobj,
	    struct kobj_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d", &debug_value);
    return count;
}

static ssize_t debug_value_show(struct kobject *kobj,
	   struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", debug_value);
}

static struct kobj_attribute debug_value_attr=
    __ATTR(debug_value, 0664, debug_value_show, debug_value_store);


#ifdef CONFIG_NUBIA_HW_CPU_SERIAL_NUM
static uint32_t    g_cpu_serial_num;

static void nubia_cpu_serial_num_init( void )
{
    /*Check the spec to get cpu serial number address on the plateform you use*/
    //MSM8952  0x00058008
    //MSM8976  0x000A60A8
#ifdef CONFIG_NUBIA_HW_CPU_SERIAL_NUM_MSM8952
    #define QFPROM_CPU_SERIAL_NUM_LSB  0x00058008
    #define QFPROM_CPU_SERIAL_NUM_MSB  0
#else
    #define QFPROM_CPU_SERIAL_NUM_LSB  0xA60A8
    #define QFPROM_CPU_SERIAL_NUM_MSB  0
#endif

    void __iomem *serial_num_addr;

	if(!request_mem_region(QFPROM_CPU_SERIAL_NUM_LSB, 4 ,"cpu_serial_num")){
		printk(KERN_ERR "nubia could not request_mem_region\n");
		g_cpu_serial_num = 0;
		return;
	}

    serial_num_addr = ioremap(QFPROM_CPU_SERIAL_NUM_LSB, 4);
    if( !serial_num_addr ) {
        printk(KERN_ERR "nubia could not map QFPROM_CPU_SERIAL_NUM_LSB address\n");
        g_cpu_serial_num = 0;
    }else {
        g_cpu_serial_num=readl_relaxed(serial_num_addr);
        printk(KERN_ERR "nubia cpu_serial_num=0x%x\n", g_cpu_serial_num);
        iounmap(serial_num_addr);
    }
    return;
 }

static uint32_t nubia_get_cpu_serial_num( void )
{
    if ( g_cpu_serial_num == 0 )
        nubia_cpu_serial_num_init();
    return g_cpu_serial_num;
}

static ssize_t nubia_cpu_serial_num_show(struct kobject *kobj,
	   struct kobj_attribute *attr, char *buf)
{
    uint32_t chip_num = nubia_get_cpu_serial_num();
    return sprintf(buf,"0x%x",chip_num);
}

static struct kobj_attribute cpu_serial_num_attr=
    __ATTR(chip_serial, 0664, nubia_cpu_serial_num_show, NULL);

static struct kobj_attribute cpu_serial_num_attr2=
    __ATTR(cpu_serial_num, 0664, nubia_cpu_serial_num_show, NULL);

#endif



static struct attribute *nubia_hw_version_attrs[] = {
    &debug_value_attr.attr,
    &pcb_version_attr.attr,
    &hw_rf_band_attr.attr,
    &config_standard_attr.attr,
#ifdef CONFIG_NUBIA_HW_CPU_SERIAL_NUM
	&cpu_serial_num_attr.attr,
	&cpu_serial_num_attr2.attr,
#endif

    NULL,
};

static struct attribute_group nubia_hw_version_attr_group = {
    .attrs = nubia_hw_version_attrs,
};

struct kobject *hw_version_kobj;

int __init nubia_hw_version_init(void)
{
    int rc = 0;

    nubia_hw_version_debug("nubia_hw_version creat attributes start \n");

    hw_version_kobj = kobject_create_and_add("nubia_hw_version", NULL);
    if (!hw_version_kobj)
	{
	  printk(KERN_ERR "%s: nubia_hw_version kobj create error\n", __func__);
	  return -ENOMEM;
    }

    rc = sysfs_create_group(hw_version_kobj,&nubia_hw_version_attr_group);
    if(rc)
    {
      printk(KERN_ERR "%s: failed to create nubia_hw_version group attributes\n", __func__);
    }

#ifdef CONFIG_NUBIA_HW_CPU_SERIAL_NUM
	nubia_cpu_serial_num_init();
#endif


    nubia_hw_version_debug("nubia_hw_version creat attributes end \n");
    return rc;
}

static void __exit nubia_hw_version_exit(void)
{
    sysfs_remove_group(hw_version_kobj,&nubia_hw_version_attr_group);
    kobject_put(hw_version_kobj);
}

module_init(nubia_hw_version_init);
module_exit(nubia_hw_version_exit);

MODULE_DESCRIPTION("nubia_hw_version driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:nubia_hw_version" );
