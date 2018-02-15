#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

/**************************debug******************************/

static int gf_request_named_gpio(struct gf_dev *gf_dev,const char *label, int *gpio)
{
    struct device *dev = &gf_dev->spi->dev;
    struct device_node *np = dev->of_node;
    int rc = of_get_named_gpio(np, label, 0);
    gf_dbg("goodix enter %s,label=%s,dev=%p,np=%p\n",__func__,label,dev,np);
    if (rc < 0) {
        gf_err("goodix failed to get '%s'\n", label);
        return rc;
    }
    *gpio = rc;
    rc = devm_gpio_request(dev, *gpio, label);
    if (rc) {
        gf_err("goodix failed to request gpio %d\n", *gpio);
        return rc;
    }
    gf_info("goodix Success! %s %d\n", label, *gpio);
    return 0;
}

/*GPIO pins reference.*/
int gf_pins_request(struct gf_dev* gf_dev)
{
    int rc = 0;

    if(gf_dev == NULL) {
        gf_err("gf_dev is NULL ,exit !!\n");
        return -1;
    }

    /*get reset resource*/
    rc = gf_request_named_gpio(gf_dev,"goodix,gpio_rst",&gf_dev->reset_gpio);
    if(rc) {
        gf_err("goodix Failed to request reset GPIO. rc = %d\n", rc);
        return -1;
    }
    /*get irq resourece*/
    rc = gf_request_named_gpio(gf_dev,"goodix,gpio_int",&gf_dev->irq_gpio);
    if(rc) {
        gf_err("goodix Failed to request IRQ GPIO. rc = %d\n", rc);
        return -1;
    }
    /*get pwr resourece*/
    rc = gf_request_named_gpio(gf_dev,"goodix,gpio_pwr",&gf_dev->pwr_gpio);
    if(rc) {
        gf_err("goodix Failed to request PWR GPIO. rc = %d\n", rc);
        rc = -1;
    }

    gf_info("goodix --------gf_parse_dts end---OK.--------\n");

    return rc;
}

int gf_pins_status_init(struct gf_dev * gf_dev)
{
    if(gf_dev == NULL) {
        gf_err("gf_dev is NULL ,exit !!\n");
        return -1;
    }

    gpio_direction_output(gf_dev->irq_gpio, 0);
    gpio_direction_input(gf_dev->irq_gpio);

    if (gpio_is_valid(gf_dev->reset_gpio)){
        gpio_direction_output(gf_dev->reset_gpio, 0);
        gpio_set_value(gf_dev->reset_gpio, 0);
        gf_info("goodix: set reset_gpio low\n");
    }
    if (gpio_is_valid(gf_dev->pwr_gpio))
    {
        gpio_direction_output(gf_dev->pwr_gpio, 1);
        gpio_set_value(gf_dev->pwr_gpio, 1);
        gf_info("goodix_power_on\n");
    }
    mdelay(10);
    if (gpio_is_valid(gf_dev->reset_gpio))
    {
        gpio_set_value(gf_dev->reset_gpio, 1);
        gf_info("goodix: set reset_gpio high\n");
    }

    return 0;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
    FUNC_ENTRY();
    if (gpio_is_valid(gf_dev->irq_gpio))
    {
        gf_disable_irq(gf_dev);
        devm_gpio_free(&gf_dev->spi->dev,gf_dev->irq_gpio);
        gf_dev->irq_gpio = -EINVAL;
        gf_info("goodix remove irq_gpio success\n");
    }

    if (gpio_is_valid(gf_dev->reset_gpio))
    {
        devm_gpio_free(&gf_dev->spi->dev,gf_dev->reset_gpio);
        gf_dev->reset_gpio = -EINVAL;
        gf_info("goodix remove reset_gpio success\n");
    }

    if (gpio_is_valid(gf_dev->pwr_gpio))
    {
        devm_gpio_free(&gf_dev->spi->dev,gf_dev->pwr_gpio);
        gf_dev->pwr_gpio = -EINVAL;
        gf_info("goodix remove pwr_gpio success\n");
    }

}

/*power management*/
int gf_power_on(struct gf_dev* gf_dev)
{
    int rc = 0;
    FUNC_ENTRY();
    if (gpio_is_valid(gf_dev->pwr_gpio)) {
        gpio_set_value(gf_dev->pwr_gpio, 1);
    }
    msleep(12); //set 12ms to meet power-on-sequence >=10ms in spec
    gf_info("goodix:power on ok\n");

    return rc;
}

int gf_power_off(struct gf_dev* gf_dev)
{
    int rc = 0;
    FUNC_ENTRY();
    if (gpio_is_valid(gf_dev->pwr_gpio)) {
        gpio_set_value(gf_dev->pwr_gpio, 0);//ryan.tuo change from 1 to 0
    }
    gf_info("goodix:power off \n");
    return rc;
}


/********************************************************************
 *CPU output low level in RST pin to reset GF. This is the MUST action for GF.
 *Take care of this function. IO Pin driver strength / glitch and so on.
 ********************************************************************/
int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
    FUNC_ENTRY();
    if(gf_dev == NULL) {
       gf_dbg("goodix Input buff is NULL.\n");
       return -1;
    }
     if (gpio_is_valid(gf_dev->reset_gpio))
    {
        gpio_set_value(gf_dev->reset_gpio, 1);
        mdelay(1);
        gpio_set_value(gf_dev->reset_gpio, 0);
        mdelay(20);
        gpio_set_value(gf_dev->reset_gpio, 1);
        gf_dbg("goodix: to set reset_gpio value to reset\n");
    }
    mdelay(delay_ms);

    gf_dbg("goodix gf_hw_reset end\n");
    return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
    FUNC_ENTRY();
    if(gf_dev == NULL) {
        gf_err("goodix Input buff is NULL.\n");
        return -1;
    } else {
        return gpio_to_irq(gf_dev->irq_gpio);
    }
}

void gf_enable_irq(struct gf_dev *gf_dev)
{
    FUNC_ENTRY();
    if (gf_dev->irq_enabled) {
        gf_info("goodix IRQ has been enabled.\n");
    } else {
        enable_irq(gf_dev->irq);
        gf_dev->irq_enabled = 1;
    }
}

void gf_disable_irq(struct gf_dev *gf_dev)
{
    FUNC_ENTRY();
    if (gf_dev->irq_enabled) {
        gf_dev->irq_enabled = 0;
        disable_irq(gf_dev->irq);
    } else {
        gf_info("goodix IRQ has been disabled.\n");
    }
}

