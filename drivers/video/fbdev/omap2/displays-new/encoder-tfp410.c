/*
 * TFP410 DPI-to-DVI encoder driver
 *
 * Copyright (C) 2013 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>

#if IS_ENABLED(CONFIG_I2C)
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>

struct tfp410_encoder_params {
	enum {
		TFP410_INPUT_EDGE_FALLING = 0,
		TFP410_INPUT_EDGE_RISING
	} input_edge;

	enum {
		TFP410_INPUT_WIDTH_12BIT = 0,
		TFP410_INPUT_WIDTH_24BIT
	} input_width;

	enum {
		TFP410_INPUT_SINGLE_EDGE = 0,
		TFP410_INPUT_DUAL_EDGE
	} input_dual;

	enum {
		TFP410_PLL_FILTER_ON = 0,
		TFP410_PLL_FILTER_OFF,
	} pll_filter;

	int input_skew; /** < Allowed range [-4, 3], use 0 for no de-skew. */
	int duallink_skew; /** < Allowed range [-4, 3]. */
};

#define tfp410_info(client, format, ...)		\
	dev_info(&client->dev, format, __VA_ARGS__)
#define tfp410_err(client, format, ...)			\
	dev_err(&client->dev, format, __VA_ARGS__)

/* HW register definitions */

#define TFP410_VENDOR_LO			0x0
#define TFP410_VENDOR_HI			0x1
#define TFP410_DEVICE_LO			0x2
#define TFP410_DEVICE_HI			0x3
#define TFP410_REVISION				0x4
#define TFP410_FREQ_MIN				0x6
#define TFP410_FREQ_MAX				0x7

#define TFP410_CONTROL0				0x8
#define TFP410_CONTROL0_POWER_ON		0x01
#define TFP410_CONTROL0_EDGE_RISING		0x02
#define TFP410_CONTROL0_INPUT_24BIT		0x04
#define TFP410_CONTROL0_DUAL_EDGE		0x08
#define TFP410_CONTROL0_HSYNC_ON		0x10
#define TFP410_CONTROL0_VSYNC_ON		0x20

#define TFP410_DETECT				0x9
#define TFP410_DETECT_INTR_STAT			0x01
#define TFP410_DETECT_HOTPLUG_STAT		0x02
#define TFP410_DETECT_RECEIVER_STAT		0x04
#define TFP410_DETECT_INTR_MODE_RECEIVER	0x00
#define TFP410_DETECT_INTR_MODE_HOTPLUG		0x08
#define TFP410_DETECT_OUT_MODE_HIGH		0x00
#define TFP410_DETECT_OUT_MODE_INTR		0x10
#define TFP410_DETECT_OUT_MODE_RECEIVER		0x20
#define TFP410_DETECT_OUT_MODE_HOTPLUG		0x30
#define TFP410_DETECT_VSWING_STAT		0x80

#define TFP410_CONTROL1				0xa
#define TFP410_CONTROL1_DESKEW_ENABLE		0x10
#define TFP410_CONTROL1_DESKEW_INCR_SHIFT	5

#define TFP410_GPIO				0xb

#define TFP410_CONTROL2				0xc
#define TFP410_CONTROL2_FILTER_ENABLE		0x01
#define TFP410_CONTROL2_FILTER_SETTING_SHIFT	1
#define TFP410_CONTROL2_DUALLINK_MASTER		0x40
#define TFP410_CONTROL2_SYNC_CONT		0x80

#define TFP410_DUALLINK				0xd
#define TFP410_DUALLINK_ENABLE			0x10
#define TFP410_DUALLINK_SKEW_SHIFT		5

#define TFP410_PLLZONE				0xe
#define TFP410_PLLZONE_STAT			0x08
#define TFP410_PLLZONE_FORCE_ON			0x10
#define TFP410_PLLZONE_FORCE_HIGH		0x20

/* HW access functions */

static void
tfp410_write(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	uint8_t buf[] = {addr, val};
	int ret;

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0)
		tfp410_err(client, "Error %d writing to subaddress 0x%x\n",
			   ret, addr);
}

static uint8_t
tfp410_read(struct i2c_client *client, uint8_t addr)
{
	uint8_t val;
	int ret;

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, &val, sizeof(val));
	if (ret < 0)
		goto fail;

	return val;

fail:
	tfp410_err(client, "Error %d reading from subaddress 0x%x\n",
		   ret, addr);
	return 0;
}

static void
tfp410_set_power_state(struct i2c_client *client, bool on)
{
	uint8_t control0 = tfp410_read(client, TFP410_CONTROL0);

	if (on)
		control0 |= TFP410_CONTROL0_POWER_ON;
	else
		control0 &= ~TFP410_CONTROL0_POWER_ON;

	tfp410_write(client, TFP410_CONTROL0, control0);
}

static void
tfp410_init_state(struct i2c_client *client,
		  struct tfp410_encoder_params *config,
		  bool duallink)
{
	tfp410_write(client, TFP410_CONTROL0,
		     TFP410_CONTROL0_HSYNC_ON |
		     TFP410_CONTROL0_VSYNC_ON |
		     (config->input_edge ? TFP410_CONTROL0_EDGE_RISING : 0) |
		     (config->input_width ? TFP410_CONTROL0_INPUT_24BIT : 0) |
		     (config->input_dual ? TFP410_CONTROL0_DUAL_EDGE : 0));

	tfp410_write(client, TFP410_DETECT,
		     TFP410_DETECT_INTR_STAT |
		     TFP410_DETECT_OUT_MODE_RECEIVER);

	tfp410_write(client, TFP410_CONTROL1,
		     (config->input_skew ? TFP410_CONTROL1_DESKEW_ENABLE : 0) |
		     (((config->input_skew + 4) & 0x7)
		      << TFP410_CONTROL1_DESKEW_INCR_SHIFT));

	tfp410_write(client, TFP410_CONTROL2,
		     TFP410_CONTROL2_SYNC_CONT |
		     (config->pll_filter ? 0 : TFP410_CONTROL2_FILTER_ENABLE) |
		     (4 << TFP410_CONTROL2_FILTER_SETTING_SHIFT));

	tfp410_write(client, TFP410_PLLZONE, 0);

	if (duallink)
		tfp410_write(client, TFP410_DUALLINK,
			     TFP410_DUALLINK_ENABLE |
			     (((config->duallink_skew + 4) & 0x7)
			      << TFP410_DUALLINK_SKEW_SHIFT));
	else
		tfp410_write(client, TFP410_DUALLINK, 0);
}

/* I2C driver functions */

static int
tfp410_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tfp410_encoder_params *config;

	int vendor = tfp410_read(client, TFP410_VENDOR_HI) << 8 |
		tfp410_read(client, TFP410_VENDOR_LO);
	int device = tfp410_read(client, TFP410_DEVICE_HI) << 8 |
		tfp410_read(client, TFP410_DEVICE_LO);
	int rev = tfp410_read(client, TFP410_REVISION);

	if (vendor != 0x1 || device != 0x6) {
		tfp410_info(client, "Unknown device %x:%x.%x\n",
			   vendor, device, rev);
		return -ENODEV;
	}

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->input_width = TFP410_CONTROL0_INPUT_24BIT;

	tfp410_init_state(client, config, 0);

	tfp410_set_power_state(client, 1);

	tfp410_info(client, "Detected device %x:%x.%x\n",
		    vendor, device, rev);

	kfree(config);

	return 0;
}

static int
tfp410_i2c_remove(struct i2c_client *client)
{
	tfp410_set_power_state(client, 0);
	return 0;
}

static struct i2c_device_id tfp410_ids[] = {
	{ "tfp410_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfp410_ids);

static struct i2c_driver tfp410_i2c_driver = {
	.probe = tfp410_i2c_probe,
	.remove = tfp410_i2c_remove,
	.driver = {
		.name = "tfp410_i2c",
	},
	.id_table = tfp410_ids,
};
#endif

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	int pd_gpio;
	int data_lines;

	struct omap_video_timings timings;
};

#define to_panel_data(x) container_of(x, struct panel_drv_data, dssdev)

static int tfp410_connect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return -EBUSY;

	r = in->ops.dpi->connect(in, dssdev);
	if (r)
		return r;

	dst->src = dssdev;
	dssdev->dst = dst;

	return 0;
}

static void tfp410_disconnect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	WARN_ON(!omapdss_device_is_connected(dssdev));
	if (!omapdss_device_is_connected(dssdev))
		return;

	WARN_ON(dst != dssdev->dst);
	if (dst != dssdev->dst)
		return;

	dst->src = NULL;
	dssdev->dst = NULL;

	in->ops.dpi->disconnect(in, &ddata->dssdev);
}

static int tfp410_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

	in->ops.dpi->set_timings(in, &ddata->timings);
	if (ddata->data_lines)
		in->ops.dpi->set_data_lines(in, ddata->data_lines);

	r = in->ops.dpi->enable(in);
	if (r)
		return r;

	if (gpio_is_valid(ddata->pd_gpio))
		gpio_set_value_cansleep(ddata->pd_gpio, 1);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void tfp410_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	if (gpio_is_valid(ddata->pd_gpio))
		gpio_set_value_cansleep(ddata->pd_gpio, 0);

	in->ops.dpi->disable(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static void tfp410_fix_timings(struct omap_video_timings *timings)
{
	timings->data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE;
	timings->sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE;
	timings->de_level = OMAPDSS_SIG_ACTIVE_HIGH;
}

static void tfp410_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	tfp410_fix_timings(timings);

	ddata->timings = *timings;
	dssdev->panel.timings = *timings;

	in->ops.dpi->set_timings(in, timings);
}

static void tfp410_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	*timings = ddata->timings;
}

static int tfp410_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	tfp410_fix_timings(timings);

	return in->ops.dpi->check_timings(in, timings);
}

static const struct omapdss_dvi_ops tfp410_dvi_ops = {
	.connect	= tfp410_connect,
	.disconnect	= tfp410_disconnect,

	.enable		= tfp410_enable,
	.disable	= tfp410_disable,

	.check_timings	= tfp410_check_timings,
	.set_timings	= tfp410_set_timings,
	.get_timings	= tfp410_get_timings,
};

static int tfp410_probe_pdata(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct encoder_tfp410_platform_data *pdata;
	struct omap_dss_device *dssdev, *in;

	pdata = dev_get_platdata(&pdev->dev);

	ddata->pd_gpio = pdata->power_down_gpio;

	ddata->data_lines = pdata->data_lines;

	in = omap_dss_find_output(pdata->source);
	if (in == NULL) {
		dev_err(&pdev->dev, "Failed to find video source\n");
		return -ENODEV;
	}

	ddata->in = in;

	dssdev = &ddata->dssdev;
	dssdev->name = pdata->name;

	return 0;
}

static int tfp410_probe_of(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct device_node *node = pdev->dev.of_node;
	struct omap_dss_device *in;
	int gpio;

	gpio = of_get_named_gpio(node, "powerdown-gpios", 0);

	if (gpio_is_valid(gpio) || gpio == -ENOENT) {
		ddata->pd_gpio = gpio;
	} else {
		dev_err(&pdev->dev, "failed to parse PD gpio\n");
		return gpio;
	}

	in = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(in)) {
		dev_err(&pdev->dev, "failed to find video source\n");
		return PTR_ERR(in);
	}

	ddata->in = in;

	return 0;
}

static int tfp410_probe(struct platform_device *pdev)
{
	struct panel_drv_data *ddata;
	struct omap_dss_device *dssdev;
	int r;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	platform_set_drvdata(pdev, ddata);

	if (dev_get_platdata(&pdev->dev)) {
		r = tfp410_probe_pdata(pdev);
		if (r)
			return r;
	} else if (pdev->dev.of_node) {
		r = tfp410_probe_of(pdev);
		if (r)
			return r;
	} else {
		return -ENODEV;
	}

	if (gpio_is_valid(ddata->pd_gpio)) {
		r = devm_gpio_request_one(&pdev->dev, ddata->pd_gpio,
				GPIOF_OUT_INIT_LOW, "tfp410 PD");
		if (r) {
			dev_err(&pdev->dev, "Failed to request PD GPIO %d\n",
					ddata->pd_gpio);
			goto err_gpio;
		}
	}

	dssdev = &ddata->dssdev;
	dssdev->ops.dvi = &tfp410_dvi_ops;
	dssdev->dev = &pdev->dev;
	dssdev->type = OMAP_DISPLAY_TYPE_DPI;
	dssdev->output_type = OMAP_DISPLAY_TYPE_DVI;
	dssdev->owner = THIS_MODULE;
	dssdev->phy.dpi.data_lines = ddata->data_lines;
	dssdev->port_num = 1;

	r = omapdss_register_output(dssdev);
	if (r) {
		dev_err(&pdev->dev, "Failed to register output\n");
		goto err_reg;
	}

#if IS_ENABLED(CONFIG_I2C)
	r = i2c_add_driver(&tfp410_i2c_driver);
	if (r != 0) {
		printk(KERN_ERR "Failed to register TFP410 I2C driver: %d\n",
		       r);
	}
#endif

	return 0;
err_reg:
err_gpio:
	omap_dss_put_device(ddata->in);
	return r;
}

static int __exit tfp410_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	omapdss_unregister_output(&ddata->dssdev);

	WARN_ON(omapdss_device_is_enabled(dssdev));
	if (omapdss_device_is_enabled(dssdev))
		tfp410_disable(dssdev);

	WARN_ON(omapdss_device_is_connected(dssdev));
	if (omapdss_device_is_connected(dssdev))
		tfp410_disconnect(dssdev, dssdev->dst);

#if IS_ENABLED(CONFIG_I2C)
	i2c_del_driver(&tfp410_i2c_driver);
#endif

	omap_dss_put_device(in);

	return 0;
}

static const struct of_device_id tfp410_of_match[] = {
	{ .compatible = "omapdss,ti,tfp410", },
	{},
};

MODULE_DEVICE_TABLE(of, tfp410_of_match);

static struct platform_driver tfp410_driver = {
	.probe	= tfp410_probe,
	.remove	= __exit_p(tfp410_remove),
	.driver	= {
		.name	= "tfp410",
		.of_match_table = tfp410_of_match,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(tfp410_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com> and Francisco Jerez <currojerez@riseup.net>");
MODULE_DESCRIPTION("TFP410 DPI to DVI encoder driver");
MODULE_LICENSE("GPL");
