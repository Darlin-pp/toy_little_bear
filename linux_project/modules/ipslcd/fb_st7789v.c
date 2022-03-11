/*
 * FB driver for the st7789s LCD display controller
 *
 * This display uses 9-bit SPI: Data/Command bit + 8 data bits
 * For platforms that doesn't support 9-bit, the driver is capable
 * of emulating this using 8-bit transfer.
 * This is done by transferring eight 9-bit words in 9 bytes.
 *
 * Copyright (C) 2013 Christian Vogelgsang
 * Based on adafruit22fb.c by Noralf Tronnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/slab.h>

#include "fbtft.h"

/*
author : zxy
describe: 搞了好几天也没搞好，正愁着，看了一眼zhihui大佬的驱动，原来这个驱动是叫st7789vw，
		  调试时候，单独写的驱动模块，屏幕初始化，按照中景源码写的，调试OK
		  使用fbtft这一部分接近内核，就有点儿陌生了，调了好几天，确定了以下代码，和其他屏幕不同，
		  如此的小屏幕，还是发命令，发字节啥的都自己写一遍吧，都交给通用源码恐难以消受。
		  不管怎样，这也算搞好了。
*/


#define DRVNAME		"fb_st7789v"
#define WIDTH		240//如果旋转了角度，这里的宽高需要对调
#define HEIGHT		135
#define DEFAULT_GAMMA \
    "D0 04 0D 11 13 2B 3F 54 4C 18 0D 0B 1F 23\n" \
    "D0 04 0C 11 13 2C 3F 44 51 2F 1F 1F 20 23"	//颜色细调，非专业人士请勿靠近，默认

/* 写命令 */
static int ipslcd_write_command(struct fbtft_par *par, u8 cmd)
{
	int ret;

	gpio_set_value(par->gpio.dc, 0);
	ret = par->fbtftops.write(par, &cmd, 1);
	if (ret < 0)
		dev_err(par->info->device,
			"write() failed and returned %d\n", ret);
	gpio_set_value(par->gpio.dc, 1);
	
	return ret;
}

/* 写数据 8bit */
static int ipslcd_write_data8(struct fbtft_par *par, u8 data)
{
	int ret;

	ret = par->fbtftops.write(par, &data, 1);
	if (ret < 0)
		dev_err(par->info->device,
			"write() failed and returned %d\n", ret);

	return ret;
}

/* 硬件复位 */
static void reset(struct fbtft_par *par)
{
	if(par->gpio.reset == -1)
		return;

	gpio_set_value(par->gpio.reset, 0);
	mdelay(200);
	gpio_set_value(par->gpio.reset, 1);
	mdelay(200);

	printk("Reset screen done.\n");
}


/* st7789初始化函数 */
static int init_display(struct fbtft_par *par)
{
	par->fbtftops.reset(par);//硬件复位，防止之前的设置干扰当前配置

	printk("**********************************zxy:screen init_display...*************************************************\r\n");

	//************* Start Initial Sequence **********//
	ipslcd_write_command(par, 0x11);
	mdelay(120);
	ipslcd_write_command(par, 0x36);//修改屏幕旋转
	ipslcd_write_data8(par, 0x70);//0x70 正横屏 0xA0 反横屏

	ipslcd_write_command(par, 0x3A);
	ipslcd_write_data8(par, 0x05);

	ipslcd_write_command(par, 0xB2);
	ipslcd_write_data8(par, 0x0C);
	ipslcd_write_data8(par, 0x0C);
	ipslcd_write_data8(par, 0x00);
	ipslcd_write_data8(par, 0x33);
	ipslcd_write_data8(par, 0x33); 

	ipslcd_write_command(par, 0xB7); 
	ipslcd_write_data8(par, 0x35);  

	ipslcd_write_command(par, 0xBB);
	ipslcd_write_data8(par, 0x19);

	ipslcd_write_command(par, 0xC0);
	ipslcd_write_data8(par, 0x2C);

	ipslcd_write_command(par, 0xC2);
	ipslcd_write_data8(par, 0x01);

	ipslcd_write_command(par, 0xC3);
	ipslcd_write_data8(par, 0x12);   

	ipslcd_write_command(par, 0xC4);
	ipslcd_write_data8(par, 0x20);  

	ipslcd_write_command(par, 0xC6); 
	ipslcd_write_data8(par, 0x0F);    

	ipslcd_write_command(par, 0xD0); 
	ipslcd_write_data8(par, 0xA4);
	ipslcd_write_data8(par, 0xA1);

	ipslcd_write_command(par, 0xE0);
	ipslcd_write_data8(par, 0xD0);
	ipslcd_write_data8(par, 0x04);
	ipslcd_write_data8(par, 0x0D);
	ipslcd_write_data8(par, 0x11);
	ipslcd_write_data8(par, 0x13);
	ipslcd_write_data8(par, 0x2B);
	ipslcd_write_data8(par, 0x3F);
	ipslcd_write_data8(par, 0x54);
	ipslcd_write_data8(par, 0x4C);
	ipslcd_write_data8(par, 0x18);
	ipslcd_write_data8(par, 0x0D);
	ipslcd_write_data8(par, 0x0B);
	ipslcd_write_data8(par, 0x1F);
	ipslcd_write_data8(par, 0x23);

	ipslcd_write_command(par, 0xE1);
	ipslcd_write_data8(par, 0xD0);
	ipslcd_write_data8(par, 0x04);
	ipslcd_write_data8(par, 0x0C);
	ipslcd_write_data8(par, 0x11);
	ipslcd_write_data8(par, 0x13);
	ipslcd_write_data8(par, 0x2C);
	ipslcd_write_data8(par, 0x3F);
	ipslcd_write_data8(par, 0x44);
	ipslcd_write_data8(par, 0x51);
	ipslcd_write_data8(par, 0x2F);
	ipslcd_write_data8(par, 0x1F);
	ipslcd_write_data8(par, 0x1F);
	ipslcd_write_data8(par, 0x20);
	ipslcd_write_data8(par, 0x23);

	ipslcd_write_command(par, 0x21); 

	ipslcd_write_command(par, 0x29);

	//ipslcd_test_fill(par, 0x001F);

	printk("**********************************zxy:screen init_display...finish*************************************************\r\n");

	return 0;
}

/* st7789设置窗口函数 */
static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	/* adjustment */
	xs += 40; xe += 40;
	ys += 53; ye += 53;

	/* Column address set */
	write_reg(par, 0x2A,
		(xs >> 8) & 0xFF, xs & 0xFF, (xe >> 8) & 0xFF, xe & 0xFF);

	/* Row address set */
	write_reg(par, 0x2B,
		(ys >> 8) & 0xFF, ys & 0xFF, (ye >> 8) & 0xFF, ye & 0xFF);

	/* Memory write */
	write_reg(par, 0x2C);
}




/* 一些硬件参数 初始化、窗口 */
static struct fbtft_display display = {
	.regwidth = 8,	//IC 8bit
	.width = WIDTH,
	.height = HEIGHT,
	.gamma_num = 2,	//gamma的暂时默认
	.gamma_len = 14,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.reset = reset,
		.init_display = init_display,
		.set_addr_win = set_addr_win,
	},
};
FBTFT_REGISTER_DRIVER(DRVNAME, "zj,st7789v", &display);	//兼容性compatible fbtft.h

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:st7789v");
MODULE_ALIAS("platform:st7789v");

MODULE_DESCRIPTION("FB driver for the st7789v LCD display controller");
MODULE_AUTHOR("zxy");
MODULE_LICENSE("GPL");
