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
#define GF_DEBUG
/*#undef  GF_DEBUG*/

#ifdef  GF_DEBUG
#define gf_dbg(fmt, args...) do { \
					pr_warn("gf:" fmt, ##args);\
		} while (0)
#define FUNC_ENTRY()  pr_warn("goodix:%s, entry\n", __func__)
#define FUNC_EXIT()  pr_warn("goodix:%s, exit\n", __func__)
#else
#define gf_dbg(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
#endif

extern void gf_disable_irq(struct gf_dev *gf_dev);
extern void gf_free_irq(struct gf_dev *gf_dev);

static int gf3208_request_named_gpio(struct gf_dev *gf_dev,const char *label, int *gpio)
{
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		gf_dbg("goodix failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		gf_dbg("goodix failed to request gpio %d\n", *gpio);
		return rc;
	}
	return 0;
}

static int select_pin_ctl(struct gf_dev *gf_dev, const char *name)
{
	size_t i;
	int rc;
	for (i = 0; i < ARRAY_SIZE(gf_dev->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(gf_dev->fingerprint_pinctrl,gf_dev->pinctrl_state[i]);
			if (rc)
				gf_dbg("goodix cannot select '%s'\n", name);
            else
                gf_dbg("goodix Selected '%s'\n", name);
                goto exit;
		}
	}
	rc = -EINVAL;
exit:
	return rc;
}

/*GPIO pins reference.*/
int gf_parse_dts(struct gf_dev* gf_dev)
{
    int rc = 0;
    int i = 0;
    /*get reset resource*/
    rc = gf3208_request_named_gpio(gf_dev,"goodix,gpio_rst",&gf_dev->reset_gpio);
    if(rc) {
        gf_dbg("goodix Failed to request gpio_rst,rc = %d\n", rc);
        return -1;
    }
    /*get irq resourece*/
    rc = gf3208_request_named_gpio(gf_dev,"goodix,gpio_int",&gf_dev->irq_gpio);
    if(rc) {
        gf_dbg("goodix Failed to request IRQ GPIO. rc = %d\n", rc);
        return -1;
    }
    /*get pwr resourece*/
    rc = gf3208_request_named_gpio(gf_dev,"goodix,gpio_pwr",&gf_dev->pwr_gpio);
    if(rc) {
        gf_dbg("goodix Failed to request PWR GPIO. rc = %d\n", rc);
        rc = -1;
    }

    gpio_direction_output(gf_dev->irq_gpio, 0);
    gpio_direction_input(gf_dev->irq_gpio);

    gf_dev->fingerprint_pinctrl = devm_pinctrl_get(&gf_dev->spi->dev);
    for (i = 0; i < ARRAY_SIZE(gf_dev->pinctrl_state); i++) {
        const char *n = pctl_names[i];
        struct pinctrl_state *state =
            pinctrl_lookup_state(gf_dev->fingerprint_pinctrl, n);
        if (IS_ERR(state)) {
            gf_dbg("goodix cannot find %s\n", n);
            rc = -EINVAL;
            }
        gf_dev->pinctrl_state[i] = state;
	}
	rc = select_pin_ctl(gf_dev, "goodixfp_reset_active");
	if (rc)
		return rc;

	rc = select_pin_ctl(gf_dev, "goodixfp_irq_active");
	if (rc)
		return rc;



   pr_warn("goodix --------gf_parse_dts end---OK.--------\n");

	 return rc;
}

void gf_cleanup(struct gf_dev	* gf_dev)
{
    gf_dbg("enter %s\n",__func__);
    if (gpio_is_valid(gf_dev->irq_gpio))
    {
        gf_disable_irq(gf_dev);
        gf_free_irq(gf_dev);
        devm_gpio_free(&gf_dev->spi->dev,gf_dev->irq_gpio);
        gf_dev->irq_gpio 		= 	-EINVAL;
    }

    if (gpio_is_valid(gf_dev->reset_gpio))
    {
        devm_gpio_free(&gf_dev->spi->dev,gf_dev->reset_gpio);
        gf_dev->reset_gpio 		= 	-EINVAL;
    }

    if (gpio_is_valid(gf_dev->pwr_gpio))
    {
        devm_gpio_free(&gf_dev->spi->dev,gf_dev->pwr_gpio);
        gf_dev->pwr_gpio 		= 	-EINVAL;
    }

	if (gf_dev->fingerprint_pinctrl != NULL){
  	  	devm_pinctrl_put(gf_dev->fingerprint_pinctrl);
		gf_dev->fingerprint_pinctrl=NULL;
	}
    gf_dbg("goodix gf_cleanup success\n");

}

/*power management*/
int gf_power_on(struct gf_dev* gf_dev)
{
    int rc = 0;
    if (gpio_is_valid(gf_dev->pwr_gpio))
    {
        gpio_direction_output(gf_dev->pwr_gpio, 0);
        gpio_set_value(gf_dev->pwr_gpio, 0);
    }
    if (gpio_is_valid(gf_dev->reset_gpio))
    {
        gpio_direction_output(gf_dev->reset_gpio, 0);
        gpio_set_value(gf_dev->reset_gpio, 0);
    }
    if (gpio_is_valid(gf_dev->pwr_gpio))
    {
        gpio_set_value(gf_dev->pwr_gpio, 1);
    }
    mdelay(10);
    if (gpio_is_valid(gf_dev->reset_gpio))
    {
        gpio_set_value(gf_dev->reset_gpio, 1);
    }
    pr_warn("goodix:power on ok\n");
    return rc;
}

int gf_power_off(struct gf_dev* gf_dev)
{
    int rc = 0;
    if (gpio_is_valid(gf_dev->pwr_gpio)) {
        gpio_set_value(gf_dev->pwr_gpio, 0);//ryan.tuo change from 1 to 0
    }
    pr_warn("goodix:power off \n");
    return rc;
}


/********************************************************************
 *CPU output low level in RST pin to reset GF. This is the MUST action for GF.
 *Take care of this function. IO Pin driver strength / glitch and so on.
 ********************************************************************/
int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
    if(gf_dev == NULL) {
       gf_dbg("goodix Input buff is NULL.\n");
       return -1;
    }
     if (gpio_is_valid(gf_dev->reset_gpio))
        {
            gpio_set_value(gf_dev->reset_gpio, 1);
            mdelay(1);
            gpio_set_value(gf_dev->reset_gpio, 0);
            mdelay(3);
            gpio_set_value(gf_dev->reset_gpio, 1);
        }
     gf_dbg("goodix gf_hw_reset\n");
     mdelay(delay_ms);
     return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
    if(gf_dev == NULL) {
        gf_dbg("goodix Input buff is NULL.\n");
        return -1;
    } else {
        return gpio_to_irq(gf_dev->irq_gpio);
    }
}


