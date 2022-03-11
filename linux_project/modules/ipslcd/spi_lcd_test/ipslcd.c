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
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/fs.h> 
#include <linux/module.h> 
#include <linux/uaccess.h> 

/***************************************************************
文件名		: ipslcd.c
作者	  	: zxy
版本	   	: V1.3
***************************************************************/

/* 暂时放到这儿的东西 */
#define LCD_W 240
#define LCD_H 135

//画笔颜色
#define WHITE        0xFFFF
#define BLACK        0x0000   
#define BLUE         0x001F  
#define BRED         0XF81F
#define GRED         0XFFE0
#define GBLUE        0X07FF
#define RED          0xF800
#define MAGENTA      0xF81F
#define GREEN        0x07E0
#define CYAN         0x7FFF
#define YELLOW       0xFFE0
#define BROWN        0XBC40 //棕色
#define BRRED        0XFC07 //棕红色
#define GRAY         0X8430 //灰色

void show_fb(struct fb_info *fbi, struct spi_device *spi);
struct fb_info * fb_init(struct spi_device *spi);
void fb_del(struct spi_device *spi);
/*---------------------------------------------------------------------------------------------------*/
/* fb设备部分 */


/* 管理spi和thread的一个结构体 */
typedef struct{
    struct spi_device *spi;     //记录fb_info对象对应的spi设备对象
    struct task_struct *thread; //记录线程对象的地址，此线程专用于把显存数据发送到屏的驱动ic
}lcd_data_t;


static int ipslcd_fb_setcolreg(unsigned int regno, unsigned int red,
			                    unsigned int green, unsigned int blue,
			                    unsigned int transp, struct fb_info *info)
{

    return 0;
}


/* 对fb的操作函数 */
struct fb_ops fops = {
    .owner = THIS_MODULE,
    .fb_setcolreg = ipslcd_fb_setcolreg,
    .fb_fillrect = cfb_fillrect,
    .fb_copyarea = cfb_copyarea,
    .fb_imageblit = cfb_imageblit,
};


/* 线程函数 */
int thread_func(void *data)
{
    struct fb_info *fbi = (struct fb_info *)data;
    lcd_data_t *ldata = fbi->par;   //貌似获取了fbi某个成员变量par

    while(1)
    {
        if(kthread_should_stop())
            break;
        show_fb(fbi, ldata->spi);
    }

    return 0;
}

/* 此函数在spi设备驱动的probe函数里被调用 */
struct fb_info * fb_init(struct spi_device *spi)
{
    struct fb_info *fbi;
    u8 *v_addr;
    u32 p_addr;
    lcd_data_t *ldata;

    v_addr = dma_alloc_coherent(NULL, LCD_W * LCD_H * 2, &p_addr, GFP_KERNEL);

    fbi = framebuffer_alloc(sizeof(lcd_data_t), NULL);//额外分配lcd_data_t类型空间
    ldata = fbi->par;       //datal指针指向额外分配的空间

    ldata->spi = spi;

    /* 设置fbi的变量 RGB这里需要注意长度和偏移*/
    fbi->var.xres = LCD_W;
    fbi->var.yres = LCD_H;
    fbi->var.xres_virtual = LCD_W;
    fbi->var.yres_virtual = LCD_H;
    fbi->var.bits_per_pixel = 16;//每个像素16bit  2字节
    fbi->var.red.offset = 11;
    fbi->var.red.length = 5;
    fbi->var.green.offset = 5;
    fbi->var.green.length = 6;
    fbi->var.blue.offset = 0;
    fbi->var.blue.length = 5;

    /* 设置fbi的常量 */
    strcpy(fbi->fix.id, "fb_ipslcd");
    fbi->fix.smem_start = p_addr; //显存的物理地址
    fbi->fix.smem_len = LCD_W * LCD_H * 2;
    fbi->fix.type = FB_TYPE_PACKED_PIXELS;
    fbi->fix.visual = FB_VISUAL_TRUECOLOR;
    fbi->fix.line_length = LCD_W * 2;

    /* 其他重要的成员 */
    fbi->fbops = &fops;
    fbi->screen_base = v_addr;//显存虚拟地址
    fbi->screen_size = LCD_W * LCD_H * 2;//显存大小

    spi_set_drvdata(spi, fbi);  //spi成员关联了fbi
    register_framebuffer(fbi);  //注册初始化完成的fbi
    ldata->thread = kthread_run(thread_func, fbi, spi->modalias);//spi和fbi更“深入”的交流

    return fbi;
}

/* 此函数在spi设备驱动remove时被调用 */
void fb_del(struct spi_device *spi)
{
    struct fb_info *fbi = spi_get_drvdata(spi); //得到之前存在spi设备中的fbi
    lcd_data_t *ldata = fbi->par;

    kthread_stop(ldata->thread); //让刷图线程退出
    unregister_framebuffer(fbi);
    dma_free_coherent(NULL, fbi->screen_size, fbi->screen_base, fbi->fix.smem_start);//把物理地址和虚拟地址放进去
    framebuffer_release(fbi);
}










/*---------------------------------------------------------------------------------------------------*/
/* spi设备部分 */


/* 开门见山 设备结构体 */
#define IPSLCD_CNT 1
#define IPSLCD_NAME "ipslcd"


struct ipslcd_dev{
    dev_t devid;                //设备号 unsigned int
    struct cdev cdev;           //字符设备
    struct class *class;        //类
    struct device *device;      //设备
    struct device_node *nd;     //设备节点
    int major;					//主设备号
    void *private_data;			//私有数据 	

    int res_gpios;	            //复位所使用的GPIO编号
	int dc_gpios;				//命令所使用的GPIO编号
};

struct ipslcd_dev ipslcddev;

/* 写一个8bit到设备 */
static s32 ipslcd_write_regs(struct spi_device *spi, u8 *buf, unsigned int len)
{
	int ret;

    struct spi_transfer *t;
	struct spi_message m;
	
	t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);	/* 申请内存 */
	//gpio_set_value(dev->cs_gpio, 0);			/* 片选拉低 */

	t->tx_buf = buf;			/* 要发送的数据 */
	t->len = len;					/* 字节 */
	spi_message_init(&m);		/* 初始化spi_message */
	spi_message_add_tail(t, &m);/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);	/* 同步发送 */

	kfree(t);					/* 释放内存 */
	//gpio_set_value(dev->cs_gpio, 1);/* 片选拉高，释放ICM20608 */
	return ret;
}

/* 写一个命令command */
void ipslcd_write_command(struct spi_device *spi, u8 cmd)
{
    gpio_set_value(ipslcddev.dc_gpios, 0);   //写命令 低电平
    ipslcd_write_regs(spi, &cmd, 1);
    gpio_set_value(ipslcddev.dc_gpios, 1);   //一般很少写命令，写完命令，就切换为写数据
}

/* 写一个数据8位data */
void ipslcd_write_data8(struct spi_device *spi, u8 data)
{
    ipslcd_write_regs(spi, &data, 1);
}

/* 分两次写一个数据16位data */
void ipslcd_write_data16(struct spi_device *spi, u16 data)
{
    u8 data_h8,data_l8;
    //获取高8位和低8位
    data_h8 = data>>8;
    data_l8 = (u8)data;
    //发送2字节 人家设备就是一次只发8bit,要是一下发16bit就不好了
    ipslcd_write_regs(spi, &data_h8, 1);
    ipslcd_write_regs(spi, &data_l8, 1);
}

/* ipslcd设置起始地址和结束地址 和横竖屏设置有关，具体查阅源码*/
void ipslcd_address_set(struct spi_device *spi, u16 x1, u16 y1, u16 x2, u16 y2)
{
    ipslcd_write_command(spi, 0x2a);    //列地址设置
    ipslcd_write_data16(spi, x1+40);
    ipslcd_write_data16(spi, x2+40);
    ipslcd_write_command(spi, 0x2b);    //行地址设置
    ipslcd_write_data16(spi, y1+53);
    ipslcd_write_data16(spi, y2+53);
    ipslcd_write_command(spi, 0x2c);    //存储器写
}

/* framebuffer线程刷屏函数 fbi显存的数据给spi发送*/
void show_fb(struct fb_info *fbi, struct spi_device *spi)
{
    u8 *p = (u8 *)fbi->screen_base;
    u16 i;

    //创建窗口，
    //0x2c存储器可以一下收很多数据，而设置窗口时
    //列地址和行地址的设置只能一次收8bit数据
    ipslcd_address_set(spi, 0, 0, LCD_W-1, LCD_H-1);
    /*
    for(i = 0; i < LCD_W * LCD_H; i++)
    {
        ipslcd_write_regs(spi, p + 2*i, 1);
        ipslcd_write_regs(spi, p + 2*i + 1, 1);
    }
    */
    ipslcd_write_regs(spi, p, LCD_W*LCD_H*2);

}

/*---------------------------------------------------------------------------------------------------*/
/* 测试函数 */

/* spi刷屏幕填充函数 */
void ipslcd_fill(struct spi_device *spi, u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    u16 i,j;
    ipslcd_address_set(spi, x1, y1, x2-1, y2-1);//设置显示范围
    for(i=y1; i<y2; i++)
    {
        for(j=x1; j<x2; j++)
        {
            ipslcd_write_regs(spi, &color, 2);
        }
    }

}

/* fb设备清屏函数 直接往显存里放数据*/
void ipslcd_fb_fill(struct spi_device *spi, struct fb_info *fbi, u16 color)
{
    u8 *p = (u8 *)fbi->screen_base;
    u16 i;

    //ipslcd_address_set(spi, 0, 0, LCD_W-1, LCD_H-1);
    for(i = 0; i < LCD_W * LCD_H; i++)
    {
        p[2*i] = color>>8;      //高8位
        p[2*i+1] = (u8)color;   //低8位
    }
}

/*---------------------------------------------------------------------------------------------------*/

/* ipslcd初始化函数 照着厂家给的51驱动移植*/
void ipslcd_device_init(struct spi_device *spi)
{
    gpio_set_value(ipslcddev.res_gpios, 0);
    mdelay(500);
    gpio_set_value(ipslcddev.res_gpios, 1);
    mdelay(500);

    /************* Start Initial Sequence **********/
    //设置横竖屏显示 可在此查阅源码修改
    ipslcd_write_command(spi, 0x11);
    mdelay(220);
    ipslcd_write_command(spi, 0x36);
    ipslcd_write_data8(spi, 0x70);

    ipslcd_write_command(spi, 0x3A);
    ipslcd_write_data8(spi, 0x05);

    ipslcd_write_command(spi, 0xB2);
    ipslcd_write_data8(spi, 0x0C);
    ipslcd_write_data8(spi, 0x0C);
    ipslcd_write_data8(spi, 0x00);
    ipslcd_write_data8(spi, 0x33);
    ipslcd_write_data8(spi, 0x33);

    ipslcd_write_command(spi, 0xB7);
    ipslcd_write_data8(spi, 0x35);

    ipslcd_write_command(spi, 0xBB);
    ipslcd_write_data8(spi, 0x19);

    ipslcd_write_command(spi, 0xC0);
    ipslcd_write_data8(spi, 0x2C);

    ipslcd_write_command(spi, 0xC2);
    ipslcd_write_data8(spi, 0x01);

    ipslcd_write_command(spi, 0xC3);
    ipslcd_write_data8(spi, 0x12);

    ipslcd_write_command(spi, 0xC4);
    ipslcd_write_data8(spi, 0x20);

    ipslcd_write_command(spi, 0xC6);
    ipslcd_write_data8(spi, 0x0F);

    ipslcd_write_command(spi, 0xD0);
    ipslcd_write_data8(spi, 0xA4);
    ipslcd_write_data8(spi, 0xA1);

    ipslcd_write_command(spi, 0xE0);
    ipslcd_write_data8(spi, 0xD0);
    ipslcd_write_data8(spi, 0x04);
    ipslcd_write_data8(spi, 0x0D);
    ipslcd_write_data8(spi, 0x11);
    ipslcd_write_data8(spi, 0x13);
    ipslcd_write_data8(spi, 0x2B);
    ipslcd_write_data8(spi, 0x3F);
    ipslcd_write_data8(spi, 0x54);
    ipslcd_write_data8(spi, 0x4C);
    ipslcd_write_data8(spi, 0x18);
    ipslcd_write_data8(spi, 0x0D);
    ipslcd_write_data8(spi, 0x0B);
    ipslcd_write_data8(spi, 0x1F);
    ipslcd_write_data8(spi, 0x23);

    ipslcd_write_command(spi, 0xE1);
    ipslcd_write_data8(spi, 0xD0);
    ipslcd_write_data8(spi, 0x04);
    ipslcd_write_data8(spi, 0x0C);
    ipslcd_write_data8(spi, 0x11);
    ipslcd_write_data8(spi, 0x13);
    ipslcd_write_data8(spi, 0x2C);
    ipslcd_write_data8(spi, 0x3F);
    ipslcd_write_data8(spi, 0x44);
    ipslcd_write_data8(spi, 0x51);
    ipslcd_write_data8(spi, 0x2F);
    ipslcd_write_data8(spi, 0x1F);
    ipslcd_write_data8(spi, 0x1F);
    ipslcd_write_data8(spi, 0x20);
    ipslcd_write_data8(spi, 0x23);

    ipslcd_write_command(spi, 0x21);

    ipslcd_write_command(spi, 0x29);


    //测试刷屏
    ipslcd_fill(spi, 0, 0, LCD_W, LCD_H, RED);   //红
    mdelay(1000);
    ipslcd_fill(spi, 0, 0, LCD_W, LCD_H, GREEN);   //绿
    mdelay(1000);
    ipslcd_fill(spi, 0, 0, LCD_W, LCD_H, BLUE);   //蓝
    mdelay(1000);

    printk("ipslcd init finish \t\n");

}


/* 打开设备 */
static int ipslcd_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &ipslcddev; /* 设置私有数据 */
    ipslcd_device_init(ipslcddev.private_data);
	return 0;
}

/* 从设备读取数据 */
/*
static ssize_t ipslcd_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	return 0;
}
*/

/* 向设备写数据 */
/*
static ssize_t ipslcd_write(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    int ret = 0;
    unsigned int databuf[1];
    struct ipslcd_dev *dev = filp->private_data;

    ret = copy_from_user(databuf, buf, cnt);
	if(ret < 0) {
		printk("kernel write failed!\r\n");
		return -EFAULT;
	}

    //填充颜色
    ipslcd_fill(dev, 50, 100, 50, 100, 0x001F);

    ipslcd_fill(dev, 10, 30, 10, 30, databuf[0]);


	return 0;
}
*/

/* 关闭/释放设备 */
static int ipslcd_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* ipslcd操作函数 */
static const struct file_operations ipslcd_ops = {
    .owner = THIS_MODULE,
    .open = ipslcd_open,
    //.read = ipslcd_read,
    //.write = ipslcd_write,
    .release = ipslcd_release,
};


/*spi驱动的probe函数 匹配成功即运行此函数 */
static int ipslcd_probe(struct spi_device *spi)
{
    printk("spi_ipslcd match success \t\n");

    int ret;
    struct fb_info *fbi;

    /* 1、构建设备号 */
    if(ipslcddev.major) //如果有主设备号的话
    {
        ipslcddev.devid = MKDEV(ipslcddev.major, 0);    //创建设备号
        register_chrdev_region(ipslcddev.devid, IPSLCD_CNT, IPSLCD_NAME);   //注册设备号
    }
    else
    {
        alloc_chrdev_region(&ipslcddev.devid, 0, IPSLCD_CNT, IPSLCD_NAME);   //如果没有，还得申请一个设备号
        ipslcddev.major = MAJOR(ipslcddev.devid);
    }
    /* 2、注册设备 */
    cdev_init(&ipslcddev.cdev, &ipslcd_ops);
    cdev_add(&ipslcddev.cdev, ipslcddev.devid, IPSLCD_CNT);
    /* 3、创建类 */
    ipslcddev.class = class_create(THIS_MODULE, IPSLCD_NAME);
    if (IS_ERR(ipslcddev.class)) 
    {
		return PTR_ERR(ipslcddev.class);
	}
    /* 4、创建设备 */
	ipslcddev.device = device_create(ipslcddev.class, NULL, ipslcddev.devid, NULL, IPSLCD_NAME);
	if (IS_ERR(ipslcddev.device)) 
    {
		return PTR_ERR(ipslcddev.device);
	}

    /* 1、获取设备树中res dc信号的节点 */
    ipslcddev.nd = of_find_node_by_path("/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000");
    if(ipslcddev.nd == NULL)
    {
        printk("ecspi3 node not find!\r\n");
        return -EINVAL;
    }
    /* 2、 获取设备树中的gpio属性，得到gpio编号 */
    ipslcddev.res_gpios = of_get_named_gpio(ipslcddev.nd, "res-gpios", 0);
    if(ipslcddev.res_gpios < 0) 
    {
		printk("can't get res-gpios");
		return -EINVAL;
	}
    ipslcddev.dc_gpios = of_get_named_gpio(ipslcddev.nd, "dc-gpios", 0);
    if(ipslcddev.dc_gpios < 0) 
    {
		printk("can't get dc-gpios");
		return -EINVAL;
	}
    /* 3、设置GPIO初始化输出 */
    ret = gpio_direction_output(ipslcddev.res_gpios, 1);
    if(ret < 0) 
    {
		printk("can't set res_gpios!\r\n");
	}
    ret = gpio_direction_output(ipslcddev.dc_gpios, 1);
    if(ret < 0) 
    {
		printk("can't set dc_gpios!\r\n");
	}

    /*初始化spi_device */
    spi->mode = SPI_MODE_0; /*MODE0，CPOL=0，CPHA=0*/
    spi_setup(spi);
    ipslcddev.private_data = spi;

    /* 初始化屏幕 */
    ipslcd_device_init(spi);

    /* 初始化fb设备 */
    fbi = fb_init(spi);

    /* 初始化完了 刷个屏吧 */
    ipslcd_fb_fill(spi, fbi, RED);
    printk("framebuffer RED finish...");
    mdelay(500);
    ipslcd_fb_fill(spi, fbi, GREEN);
    printk("framebuffer GREEN finish...");
    mdelay(100);
    ipslcd_fb_fill(spi, fbi, BLUE);
    printk("framebuffer BLUE finish...");
    mdelay(100);
    ipslcd_fb_fill(spi, fbi, WHITE);
    printk("framebuffer WHITE finish...");

    return 0;
}

/*spi驱动的remove函数 */
static int ipslcd_remove(struct spi_device *spi)
{

    /* fb设备回收 */
    fb_del(spi);

    /* 删除设备 */
    cdev_del(&ipslcddev.cdev);
	unregister_chrdev_region(ipslcddev.devid, IPSLCD_CNT);

    /* 注销掉类和设备 */
	device_destroy(ipslcddev.class, ipslcddev.devid);
	class_destroy(ipslcddev.class);

    return 0;
}

/* 传统匹配方式ID列表 */
static const struct spi_device_id ipslcd_id[] = {
    {"zhongjing,ipslcd", 0},
    {}
};

/* 设备树匹配列表 */
static const struct of_device_id ipslcd_of_match[] = {
    { .compatible = "zhongjing,ipslcd" },
    { /* Sentinel */ }
};

/*SPI驱动结构体 */
static struct spi_driver ipslcd_driver = {
    .probe = ipslcd_probe,
    .remove = ipslcd_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "ipslcd",
        .of_match_table = ipslcd_of_match,
    },
    .id_table = ipslcd_id,
};



/*驱动入口函数 */
static int __init ipslcd_init(void)
{
    return spi_register_driver(&ipslcd_driver);
}
/*驱动出口函数 */
static void __exit ipslcd_exit(void)
{
    spi_unregister_driver(&ipslcd_driver);
}

module_init(ipslcd_init);
module_exit(ipslcd_exit);
MODULE_LICENSE("GPL");//GPL协议
MODULE_AUTHOR("zxy");




