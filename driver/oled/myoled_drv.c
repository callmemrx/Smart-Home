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
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>
#include <linux/cdev.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>

#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>

#define OLED_IOC_INIT 			123
#define OLED_IOC_SET_POS 		124

//为0 表示命令，为1表示数据
#define OLED_CMD 	0
#define OLED_DATA 	1

static struct fb_info *myfb_info;
static unsigned int pseudo_palette[16];
static struct task_struct *oled_thread;
static unsigned char *oled_buf; 			
static struct spi_device *oled;

static int major = 0; 
static int minor = 0;
dev_t devno;
struct cdev cdev;

static struct gpio_desc *dc_gpio;

/* from pxafb.c */
static inline unsigned int chan_to_field(unsigned int chan,
					 struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int myoled_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	unsigned int val;

	/* dprintk("setcol: regno=%d, rgb=%d,%d,%d\n",
		   regno, red, green, blue); */

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseudo-palette */

		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red,   &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue,  &info->var.blue);

			pal[regno] = val;
		}
		break;

	default:
		return 1;	/* unknown type */
	}

	return 0;
}

static struct fb_ops myfb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= myoled_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static void dc_pin_init(void)
{
	gpiod_direction_output(dc_gpio, 1);
}

static void oled_set_dc_pin(int val)
{
	gpiod_set_value(dc_gpio, val);
}

static void spi_write_datas(const unsigned char *buf, int len)
{
	spi_write(oled, buf, len);
}

/*oled_write_cmd_data向特定地址写入数据或者命令
 *uc_cmd:为1则表示写入数据，为0表示写入命令
 */
static void oled_write_cmd_data(unsigned char uc_data,unsigned char uc_cmd)
{
	if(uc_cmd==0)
	{
		oled_set_dc_pin(0);
	}
	else
	{
		oled_set_dc_pin(1);//拉高，表示写入数据
	}
	spi_write_datas(&uc_data, 1);//写入
}


/* oled_init初始化oled，包括SPI控制器得初始化 */
static int oled_init(void)
{
	oled_write_cmd_data(0xae,OLED_CMD);//关闭显示

	oled_write_cmd_data(0x00,OLED_CMD);//设置 lower column address
	oled_write_cmd_data(0x10,OLED_CMD);//设置 higher column address

	oled_write_cmd_data(0x40,OLED_CMD);//设置 display start line

	oled_write_cmd_data(0xB0,OLED_CMD);//设置page address

	oled_write_cmd_data(0x81,OLED_CMD);// contract control
	oled_write_cmd_data(0x66,OLED_CMD);//128

	oled_write_cmd_data(0xa1,OLED_CMD);//设置 segment remap

	oled_write_cmd_data(0xa6,OLED_CMD);//normal /reverse

	oled_write_cmd_data(0xa8,OLED_CMD);//multiple ratio
	oled_write_cmd_data(0x3f,OLED_CMD);//duty = 1/64

	oled_write_cmd_data(0xc8,OLED_CMD);//com scan direction

	oled_write_cmd_data(0xd3,OLED_CMD);//set displat offset
	oled_write_cmd_data(0x00,OLED_CMD);//

	oled_write_cmd_data(0xd5,OLED_CMD);//set osc division
	oled_write_cmd_data(0x80,OLED_CMD);//

	oled_write_cmd_data(0xd9,OLED_CMD);//ser pre-charge period
	oled_write_cmd_data(0x1f,OLED_CMD);//

	oled_write_cmd_data(0xda,OLED_CMD);//set com pins
	oled_write_cmd_data(0x12,OLED_CMD);//

	oled_write_cmd_data(0xdb,OLED_CMD);//set vcomh
	oled_write_cmd_data(0x30,OLED_CMD);//

	oled_write_cmd_data(0x8d,OLED_CMD);//set charge pump disable 
	oled_write_cmd_data(0x14,OLED_CMD);//

	oled_write_cmd_data(0xaf,OLED_CMD);//set dispkay on

	return 0;
}		  			 		  						  					  				 	   		  	  	 	  

/* OLED_DIsp_Set_Pos设置要显示的位置 */
static void OLED_DIsp_Set_Pos(int x, int y)
{ 	oled_write_cmd_data(0xb0+y,OLED_CMD);
	oled_write_cmd_data((x&0x0f),OLED_CMD); 
	oled_write_cmd_data(((x&0xf0)>>4)|0x10,OLED_CMD);
}   	      	   			 

static long spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int x, y;
	
	/* 根据cmd操作硬件 */
	switch (cmd)
	{
		case OLED_IOC_INIT:  /* init */ 
		{
			dc_pin_init();
			oled_init();
			break;
		}
		case OLED_IOC_SET_POS:  /* set pos */ 
		{
			x = arg & 0xff;
			y = (arg >> 8) & 0xff;
			OLED_DIsp_Set_Pos(x, y);
			break;
		}
	}
	return 0;
}

static ssize_t spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	char *ker_buf;
	int err;

	ker_buf = kmalloc(count, GFP_KERNEL);
	err = copy_from_user(ker_buf, buf, count);
	
	oled_set_dc_pin(1);//拉高，表示写入数据
	spi_write_datas(ker_buf, count);
	kfree(ker_buf);
	return count;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	.write =	spidev_write,
	.unlocked_ioctl = spidev_ioctl,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "my,oled" },
	{},
};

static int oled_thread_func(void *param)
{
	unsigned char *p[8];
	unsigned char data[8];
	int i;
	int j;
	int line;
	int bit;
	unsigned char byte;
	unsigned char *fb  = myfb_info->screen_base;
	int k;
	
	while (!kthread_should_stop()) 
	{
		/* 1. 从Framebuffer得到数据 */
		/* 2. 转换格式 */
		k = 0;
		for (i = 0; i < 8; i++)
		{
			for (line = 0; line < 8; line++)
				p[line] = &fb[i*128 + line * 16];
			
			for (j = 0; j < 16; j++)
			{
				for (line = 0; line < 8; line++)
				{
					data[line] = *p[line];
					p[line] += 1;
				}

				for (bit = 0; bit < 8; bit++)
				{
					byte =  (((data[0]>>bit) & 1) << 0) |
							(((data[1]>>bit) & 1) << 1) |
							(((data[2]>>bit) & 1) << 2) |
							(((data[3]>>bit) & 1) << 3) |
							(((data[4]>>bit) & 1) << 4) |
							(((data[5]>>bit) & 1) << 5) |
							(((data[6]>>bit) & 1) << 6) |
							(((data[7]>>bit) & 1) << 7);

					oled_buf[k++] = byte;
				}
				
			}
		}
		
		/* 3. 通过SPI发送给OLED */
		for (i = 0; i < 8; i++)
		{
			OLED_DIsp_Set_Pos(0, i);
			oled_set_dc_pin(1);
			spi_write_datas(&oled_buf[i*128], 128);
		}
				
		/* 4. 休眠一会 */
		schedule_timeout_interruptible(HZ);
	}
	return 0;
}

static int spidev_probe(struct spi_device *spi)
{
	dma_addr_t phy_addr;
	int err;
	
	/* 1. 记录spi_device */
	oled = spi;

	/* 2. 注册字符设备 */
	err = alloc_chrdev_region(&devno, minor, 1, "myoled");
	major = MAJOR(devno);
	spidev_class = class_create(THIS_MODULE, "myoled");

	cdev_init(&cdev, &spidev_fops);
    cdev.owner = THIS_MODULE;
	cdev_add(&cdev, MKDEV(major, minor), 1);
	device_create(spidev_class, NULL,MKDEV(major, minor), NULL, "myoled");
	
	/* 3. 获得GPIO引脚 */
	dc_gpio = gpiod_get(&spi->dev, "dc", 0);
	
	/* A. 分配fb_info */
	myfb_info = framebuffer_alloc(0, NULL);

	/* B. 设置fb_info */
	/* B.1 var : LCD分辨率、颜色格式 */
	myfb_info->var.xres_virtual = myfb_info->var.xres = 128;
	myfb_info->var.yres_virtual = myfb_info->var.yres = 64;
	
	myfb_info->var.bits_per_pixel = 1;  	/* rgb565 */	

	/* B.2 fix */
	strcpy(myfb_info->fix.id, "my_oled");
	myfb_info->fix.smem_len = myfb_info->var.xres * myfb_info->var.yres * myfb_info->var.bits_per_pixel / 8;

	myfb_info->flags |= FBINFO_MODULE; 		/* 禁止显示LOGO，否则段错误 */

	/* fb的虚拟地址 */
	myfb_info->screen_base = dma_alloc_wc(NULL, myfb_info->fix.smem_len, &phy_addr, GFP_KERNEL);			
	myfb_info->fix.smem_start = phy_addr;   /* fb的物理地址 */
	
	myfb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	myfb_info->fix.visual = FB_VISUAL_MONO10;

	myfb_info->fix.line_length = myfb_info->var.xres * myfb_info->var.bits_per_pixel / 8;	

	/* c. fbops */
	myfb_info->fbops = &myfb_ops;
	myfb_info->pseudo_palette = pseudo_palette;

	/* C. 注册fb_info */
	register_framebuffer(myfb_info);

	/* D. 创建内核线程 */
	oled_buf = kmalloc(1024, GFP_KERNEL);   /* 必须kmalloc显式分配内存，否则段错误 */ 
	dc_pin_init();
	oled_init();
	
 	oled_thread = kthread_run(oled_thread_func, NULL, "oled_kthead");

	return 0;
}

static int spidev_remove(struct spi_device *spi)
{
	kthread_stop(oled_thread);
	kfree(oled_buf);
	
	/* A. 反注册fb_info */
	unregister_framebuffer(myfb_info);

	/* B. 释放内存 */
	dma_free_wc(NULL, myfb_info->fix.smem_len, myfb_info->screen_base,
		    myfb_info->fix.smem_start);

	/* C. 释放fb_info */
	framebuffer_release(myfb_info);
	
	gpiod_put(dc_gpio);
	
	/* 反注册字符设 */
	device_destroy(spidev_class, MKDEV(major, minor));
	cdev_del(&cdev);
	class_destroy(spidev_class);
	unregister_chrdev_region(devno, 1);

	return 0;
}

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"my_spi_oled_drv",
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,
};

static int __init spidev_init(void)
{
	int status;

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
	}
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
}
module_exit(spidev_exit);

MODULE_LICENSE("GPL");

