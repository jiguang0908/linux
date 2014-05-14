#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c-id.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <plat/regs-iic.h>
#include <plat/iic.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <linux/time.h>
#include <linux/errno.h>
#include <linux/cpufreq.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <mach/regs-irq.h>
#include <mach/regs-gpio.h>
#include <mach/irqs.h>


#define DRV_NAME "i2c-s3c2440"


struct s3ci2c_if {
	unsigned int  __iomem		*reg; /* memory mapped registers */
	struct i2c_msg		*msgs; /* messages currently handled */
	int			msgs_num; /* nb of msgs to do */
	int			msgs_done; /* nb of msgs finally handled */
	struct i2c_adapter	adap;
	struct clk *clk;
};

static struct s3ci2c_if *iface;

void s3ci2c_master_start(void)
{
	struct i2c_msg *msgs = iface->msgs;
	if(msgs->flags&I2C_M_RD){
		printk("read addr=%d,num=%d",msgs->addr,msgs->len);
	}else{
		printk("write addr=%d,num=%d",msgs->addr,msgs->len);

	}

}
//	void s3ci2c_master_stop(void)
//	{
//	
//	
//	}
static int s3ci2c_master_xfer(struct i2c_adapter *adap,
				struct i2c_msg *msgs, int num)
{
	iface->msgs = msgs;
	iface->msgs_num = num;
	iface->msgs_done = 0;
	s3ci2c_master_start();	
	return 0;
}
static u32 s3ci2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;

}

static irqreturn_t s3ci2c_interrupt_entry(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static struct i2c_algorithm s3ci2c_algorithm = {
	.master_xfer   = s3ci2c_master_xfer,
	.functionality = s3ci2c_functionality,
};

static void  s3ci2c_exit(void)
{
	i2c_del_adapter(&iface->adap);
	free_irq(IRQ_IIC,NULL);
	clk_disable(iface->clk);
	iounmap(iface->reg);	
	kfree(iface);
}

static int  s3ci2c_init(void)
{
	unsigned int value;
	int rc;

	iface = kmalloc(sizeof(struct s3ci2c_if),GFP_KERNEL);
	if(iface == NULL){
		return -ENOMEM;
	}
	memset(iface,0,sizeof(struct s3ci2c_if));
	
	iface->reg = ioremap(0x54000000,0x14);
	if (!iface->reg) {
		rc = -ENOMEM;
		goto fail;
	}
	
	//ÅäÖÃI2CÓ²¼þ¿ØÖÆÆ÷
	s3c2410_gpio_cfgpin(S3C2410_GPE(14),S3C2410_GPE14_IICSCL);
	s3c2410_gpio_cfgpin(S3C2410_GPE(15),S3C2410_GPE15_IICSDA);

	value = S3C2410_IICCON_ACKEN|S3C2410_IICCON_IRQEN|
			S3C2410_IICCON_SCALE(0)|S3C2410_IICCON_TXDIV_512;
	iowrite32(value,iface->reg+S3C2410_IICCON);
	value = S3C2410_IICSTAT_MASTER_TX | S3C2410_IICSTAT_TXRXEN;
	iowrite32(value,iface->reg+S3C2410_IICSTAT);
	
	iface->clk = clk_get(NULL, "i2c");
	if (IS_ERR(iface->clk)) {
		rc = PTR_ERR(iface->clk);
		goto fail;
	}
	rc = clk_enable(iface->clk);
	if (rc < 0)
		goto fail;

	rc = request_irq(IRQ_IIC, s3ci2c_interrupt_entry,
			 IRQF_DISABLED, DRV_NAME,NULL);
	if (rc) {
		printk("s3c2440-i2c: cant get IRQ %d\n", IRQ_IIC);
		goto fail;
	}
	

	strlcpy(iface->adap.name,DRV_NAME, 12);
	iface->adap.owner = THIS_MODULE;
	iface->adap.algo = &s3ci2c_algorithm;

	pr_info("I2C: s3c2440 I2C driver\n");
	rc = i2c_add_adapter(&iface->adap);
	if (rc)
		goto fail;
	return 0;
fail:
	s3ci2c_exit();
	return rc;
}

MODULE_DESCRIPTION("I2C-Bus adapter routines for S6000 I2C");
MODULE_LICENSE("GPL");

module_init(s3ci2c_init);
module_exit(s3ci2c_exit);


