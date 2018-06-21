/*
 * LCD driver for Victor LCD
 *
 * Copyright (C) 2018 Anki Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

#define RSHIFT			0x001C

#define LCD_FRAME_WIDTH		184
#define LCD_FRAME_HEIGHT	96
#define BYTES_PER_PIXEL		2
#define FRAMEBUFFER_SIZE	(LCD_FRAME_WIDTH * LCD_FRAME_HEIGHT * BYTES_PER_PIXEL)
#define MAX_XFER_SIZE		FRAMEBUFFER_SIZE

struct vicspi_device {
	struct spi_device *spi;
	struct fb_info *info;
	int cmd_gpio;
	int gpio_reset1;
	int gpio_reset2;
	u32 pseudo_palette[256];
};

static int vicspi_write(struct vicspi_device *vic, int cmd, const u8 *wbuf,
			   int wlen)
{
	struct spi_message	m_cmd;
	struct spi_transfer	*x, x_cmd;
	int			i, num_xfers, ret = 0;
	u8			*pbuf = wbuf;

	/*
	 * We should be able to do this in one message.
	 * But as the controller driver appears to be totally unaware of
	 * such thing as external chip select, let's do it like this.
	 */
	if (cmd) {
		spi_message_init(&m_cmd);
		memset(&x_cmd, 0, sizeof(x_cmd));
		x_cmd.tx_buf = &cmd;
		x_cmd.len = 1;
		spi_message_add_tail(&x_cmd, &m_cmd);
		gpio_set_value(vic->cmd_gpio, 0);
		ret = spi_sync(vic->spi, &m_cmd);
		gpio_set_value(vic->cmd_gpio, 1);
		if (ret) {
			dev_err(&vic->spi->dev, "cmd %x failed\n", cmd);
			goto out;
		}
	}

	if (wlen == 0)
		goto out;

	num_xfers = (wlen + MAX_XFER_SIZE - 1) / MAX_XFER_SIZE;

	x = kzalloc(sizeof (struct spi_transfer) * num_xfers, GFP_KERNEL);
	for (i = 0; i < num_xfers;
			i++, pbuf += MAX_XFER_SIZE, wlen -= MAX_XFER_SIZE) {
		x[i].tx_buf = pbuf;
		x[i].len = (wlen < MAX_XFER_SIZE ? wlen : MAX_XFER_SIZE);
		dev_info(&vic->spi->dev, "xfer %d length %d\n",
			i, x[i].len);
	}

	ret = spi_sync_transfer(vic->spi, x, num_xfers);
	if (ret < 0)
		dev_err(&vic->spi->dev, "sending data failed, error %d\n", ret);

	kfree(x);

out:
	return ret;
}

static inline void vicspi_cmd(struct vicspi_device *vic, int cmd)
{
	vicspi_write(vic, cmd, NULL, 0);
}

static void vicspi_update_lcd(struct vicspi_device *vic, struct fb_info *info,
		unsigned int dx, unsigned int dy,
		unsigned int w, unsigned int h)
{
	/* TODO: update the rectangle */
	vicspi_write(vic, 0x2c, info->screen_base, FRAMEBUFFER_SIZE);
}

static int vicspi_lcd_checkvar(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	switch (var->bits_per_pixel) {
	case 16:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
static void vicspi_fillrect(struct fb_info *info,
			   const struct fb_fillrect *rect)
{
	struct vicspi_device *vic = info->par;

	sys_fillrect(info, rect);

	/* update the physical lcd */
	vicspi_update_lcd(vic, info, rect->dx, rect->dy,
				rect->width, rect->height);
}

static void vicspi_copyarea(struct fb_info *info,
			   const struct fb_copyarea *area)
{
	struct vicspi_device *vic = info->par;

	sys_copyarea(info, area);

	/* update the physical lcd */
	vicspi_update_lcd(vic, info, area->dx, area->dy,
				area->width, area->height);
}

static void vicspi_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct vicspi_device *vic = info->par;

	sys_imageblit(info, image);

	/* update the physical lcd */
	vicspi_update_lcd(vic, info, image->dx, image->dy,
				image->width, image->height);
}

static void vicspi_init_lcd(struct vicspi_device *vic)
{
	static const struct {
		int	cmd;
		int	wlen;
		u8	wbuf[14];
		u16	wait; /* ms */
	} *init, init_ctrl[] = {
		{ 0x10, 1, { 0x00 }, 120 }, /* Sleep */
		{ 0x36, 1, { 0x00 } }, /* RGB565 */
		{ 0xB0, 2, { 0x00, 0x08 } },  /* unswapped bytes in pixel */
		{ 0xB7, 1, { 0x72 } }, // Gate control (VGH 14.97, VGL -8.23)
		{ 0xBB, 1, { 0x36 } }, // VCOMS 1.45v
		{ 0xC0, 1, { 0x2C } },
		{ 0xC2, 1, { 0x01 } },
		{ 0xC3, 1, { 0x14 } }, // VRH 4.55v
		{ 0xC4, 1, { 0x20 } },
		{ 0xC6, 1, { 0x0F } },
		{ 0xD0, 2, { 0xA4, 0xA1 } }, // Power control 1
		{ 0xE0, 14, { 0xD0, 0x10, 0x16, 0x0A, 0x0A, 0x26, 0x3C, 0x53, 0x53, 0x18, 0x15, 0x12, 0x36, 0x3C } }, // +ve voltage gamma control
		{ 0xE1, 14, { 0xD0, 0x11, 0x19, 0x0A, 0x09, 0x25, 0x3D, 0x35, 0x54, 0x17, 0x15, 0x12, 0x36, 0x3C } }, // -ve voltage gamma control
		{ 0xE9, 3, { 0x05, 0x05, 0x01 } },
		{ 0x3A, 1, { 0x55 } },
		{ 0x55, 1, { 0x03 } }, // Content Adaptive Brightness Control: 0x03 = Color Enhancement Off, Moving Image Mode
		{ 0x21, 1, { 0x00 } },
		{ 0x2A, 4, { 0x00, RSHIFT, (LCD_FRAME_WIDTH + RSHIFT - 1) >> 8, (LCD_FRAME_WIDTH + RSHIFT - 1) & 0xFF } },
		{ 0x2B, 4, { 0x00, 0x00, (LCD_FRAME_HEIGHT -1) >> 8, (LCD_FRAME_HEIGHT -1) & 0xFF } },
		{ 0x11, 1, { 0x00 }, 120 }, /* Sleep off */
		{ 0x29, 1, { 0x00 }, 400 }, /* Display On */
		{ 0 }
	};

	init = init_ctrl;
	while (init->cmd != 0) {
		vicspi_write(vic, init->cmd, init->wbuf, init->wlen);
		if (init->wait)
			msleep(init->wait);
		init++;
	}
}

static int vicspi_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	return 0;
}

static ssize_t vicspi_lcd_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct vicspi_device *vic = info->par;
	unsigned long p = *ppos;

	if (*ppos >= FRAMEBUFFER_SIZE)
		return -ENOSPC;

	if (count + *ppos > FRAMEBUFFER_SIZE)
		count = FRAMEBUFFER_SIZE - *ppos;

	pr_debug("%s: count %d *ppos %x\n", __func__, count, p);
	copy_from_user(info->screen_base + *ppos, buf, count);
	vicspi_write(vic, 0x2c, info->screen_base, count + p);
	*ppos += count;

	return count;
}

static int vicspi_setcolreg(unsigned regno, unsigned red, unsigned green,
			    unsigned blue, unsigned transp,
			    struct fb_info *info)
{
	// TODO: limits

	pr_debug("%s: regno %d\n", __func__, regno);
	((u32 *)info->pseudo_palette)[regno] = (red & 0xf800) |
						((green & 0xf800) >> 5) |
						((blue & 0xf800) >> 11);
	return 0;
}

static void vicspi_lcd_clear(struct fb_info *info)
{
	struct vicspi_device *vic = info->par;

	memset(info->screen_base, 0, FRAMEBUFFER_SIZE);
	vicspi_write(vic, 0x2c, info->screen_base, FRAMEBUFFER_SIZE);
}

static int vicspi_blank(int blank_mode, struct fb_info *info)
{
	// TODO
	return 0;
}

static struct fb_ops vicspi_fb_ops = {
	.owner = THIS_MODULE,
	.fb_pan_display = vicspi_pan_display,
	.fb_read = fb_sys_read,
	.fb_write = vicspi_lcd_write,
	.fb_check_var = vicspi_lcd_checkvar,
	.fb_fillrect = vicspi_fillrect,
	.fb_imageblit = vicspi_imageblit,
	.fb_copyarea = vicspi_copyarea,
	.fb_blank = vicspi_blank,
	.fb_setcolreg = vicspi_setcolreg,
};

static struct fb_fix_screeninfo vicspi_screeninfo = {
	.id = "vicspi_lcd",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.accel = FB_ACCEL_NONE,
	.line_length = LCD_FRAME_WIDTH * BYTES_PER_PIXEL,
};

static struct fb_var_screeninfo vicspi_var = {
	.xres = LCD_FRAME_WIDTH,
	.yres = LCD_FRAME_HEIGHT,
	.xres_virtual = LCD_FRAME_WIDTH,
	.yres_virtual = LCD_FRAME_HEIGHT,
	.bits_per_pixel = BYTES_PER_PIXEL * 8,
	.nonstd = 1,
};

static void vicspi_lcd_reset(struct vicspi_device *vic)
{
	gpio_set_value(vic->gpio_reset1, 0);
	if (vic->gpio_reset2 != 0)
		gpio_set_value(vic->gpio_reset2, 0);
	udelay(100);
	gpio_set_value(vic->gpio_reset1, 1);
	if (vic->gpio_reset2 != 0)
		gpio_set_value(vic->gpio_reset2, 1);
	udelay(100);
}

static int vicspi_lcd_probe(struct spi_device *spi)
{
	struct fb_info *fbi;
	struct device_node *np = spi->dev.of_node;
	struct vicspi_device *vic;
	unsigned char *videomem = kzalloc(FRAMEBUFFER_SIZE, GFP_KERNEL);
	int ret = 0;

	if (!videomem)
		return -ENOMEM;

	vicspi_screeninfo.smem_start = virt_to_phys(videomem);
	vicspi_screeninfo.smem_len = FRAMEBUFFER_SIZE;

	fbi = framebuffer_alloc(sizeof(struct vicspi_device), &spi->dev);
	if (fbi == NULL) {
		dev_err(&spi->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	fbi->screen_base = videomem;
	dev_info(&spi->dev, "screen_base set to %p\n", fbi->screen_base);

	vic = fbi->par;
	vic->info = fbi;
	spi_set_drvdata(spi, fbi);

	vic->spi = spi;
	vic->cmd_gpio = of_get_named_gpio(np, "cmd-gpio", 0); // 110
	vic->gpio_reset1 = of_get_named_gpio(np, "gpio-reset1", 0);
	vic->gpio_reset2 = of_get_named_gpio(np, "gpio-reset2", 0);

	dev_info(&spi->dev, "GPIO: CMD %d reset1 %d reset2 %d\n",
		vic->cmd_gpio, vic->gpio_reset1, vic->gpio_reset2);

	/* TODO: GPIO request proper error handling */
	ret = gpio_request(vic->cmd_gpio, dev_name(&spi->dev));
	if (ret)
		dev_err(&spi->dev, "can't request gpio %d\n", vic->cmd_gpio);
	ret = gpio_request_one(vic->gpio_reset1, GPIOF_OPEN_DRAIN,
				dev_name(&spi->dev));
	if (ret)
		dev_err(&spi->dev, "can't request gpio %d\n", vic->gpio_reset1);
	if (vic->gpio_reset2 != 0) {
		ret = gpio_request_one(vic->gpio_reset2, GPIOF_OPEN_DRAIN,
					dev_name(&spi->dev));
		if (ret)
			dev_err(&spi->dev, "can't request gpio %d\n",
				vic->gpio_reset2);
	}

	ret = gpio_direction_output(vic->cmd_gpio, 1);
	if (ret)
		dev_err(&spi->dev, "can't set gpio %d direction\n",
			vic->cmd_gpio);
	ret = gpio_direction_output(vic->gpio_reset1, 0);
	if (ret)
		dev_err(&spi->dev, "can't set gpio %d direction\n",
			vic->gpio_reset1);

	if (vic->gpio_reset2 != 0) {
		ret = gpio_direction_output(vic->gpio_reset2, 0);
		if (ret)
			dev_err(&spi->dev, "can't set gpio %d direction\n",
				vic->gpio_reset2);
	}

	fbi->fbops = &vicspi_fb_ops;
	fbi->fix = vicspi_screeninfo;
	fbi->var = vicspi_var;

	fbi->pseudo_palette = vic->pseudo_palette;

	ret = register_framebuffer(fbi);
	if (ret) {
		dev_err(&spi->dev, "failed to register framebuffer\n");
		framebuffer_release(fbi);
		return ret;
	}

	vicspi_lcd_reset(vic);
	vicspi_init_lcd(vic);
	/* eventually we'll have the fb_blank function instead */
	vicspi_lcd_clear(fbi);
	return 0;
}

static int vicspi_lcd_remove(struct spi_device *spi)
{
	struct fb_info *fbi = spi_get_drvdata(spi);
	struct vicspi_device *vic = fbi->par;

	gpio_free(vic->cmd_gpio);
	gpio_free(vic->gpio_reset1);
	if (vic->gpio_reset2)
		gpio_free(vic->gpio_reset2);
	unregister_framebuffer(fbi);
	kfree(fbi->screen_base);
	framebuffer_release(fbi);
	return 0;
}

static const struct of_device_id vicspi_lcd_ids[] = {
	{ .compatible = "vicspi_lcd" }, { }
};
MODULE_DEVICE_TABLE(of, vicspi_lcd_ids);

static struct spi_driver vicspi_lcd_driver = {
	.driver = {
		.name	= "vicspi_lcd",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(vicspi_lcd_ids),
	},
	.probe	= vicspi_lcd_probe,
	.remove	= vicspi_lcd_remove,
};

module_spi_driver(vicspi_lcd_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vitaly Wool");
MODULE_DESCRIPTION("SPI LCD driver");
