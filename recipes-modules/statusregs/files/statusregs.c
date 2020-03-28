/*  statusregs.c - The simplest kernel module.

* Copyright (C) 2013 - 2016 Xilinx, Inc
* Modified from sample kernel module code, Dan McGraw M0WUT
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License along
*   with this program. If not, see <http://www.gnu.org/licenses/>.

*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

#include "statusregs.h"

/* Standard module information, edit as appropriate */
MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("M0WUT");
MODULE_DESCRIPTION
    ("statusregs - Provides SDR status in JSON format");

#define DRIVER_NAME "statusregs"

#define BUF_SIZE 2048

static int major;
static int deviceOpen = 0;
static char buffer[BUF_SIZE];
static char *msgPtr;



static struct statusregs_local *lp = NULL;

static unsigned int read(int offset){
	unsigned int x;
	x = ioread32(lp->base_addr + 4 * offset);
	#ifdef DEBUG 
		printk(KERN_INFO "Read value %#010x from register %d", x, offset);
	#endif
	return x;
}


static void write(int offset, unsigned int data){
	#ifdef DEBUG
		printk(KERN_INFO "Writing data %010x to address %d", data, offset);
	#endif
	iowrite32(data, lp->base_addr + 4 * offset);
}

static struct file_operations fops = {
	.read = status_read,
	.write = status_write,
	.open = status_open,
	.release = status_release
};


static int status_open(struct inode *inode_p, struct file *file_p){
	if(deviceOpen)
		return -EBUSY;

	deviceOpen++;
	try_module_get(THIS_MODULE);
	
	// Write JSON to buffer
	sprintf(buffer,
"{\n\
	\"IDconfirmed\": \"%s\",\n\
	\"bitstreamVersion\": %u.%u,\n\
	\"displayFrequency\": %u,\n\
	\"displayEnabled\": \"%s\",\n\
	\"mode\": \"%s\",\n\
	\"adcRandom\": \"%s\",\n\
	\"adcDither\": \"%s\",\n\
	\"phaseAccumulator1\": %u, \n\
	\"adcClockFrequency\": %u \n\
}\n",
	((read(OFFSET_ID) & 0xFFFF0000) == 0xBEEF0000 ? "True" : "False"),
	(read(OFFSET_ID) >> 8) & 0xFF,
	read(OFFSET_ID) & 0xFF,
	read(OFFSET_DISPFREQ) * (read(OFFSET_DISPMODE) & 0x01 ? 10 : 1),
	(read(OFFSET_DISPMODE) & 0x02 ? "True" : "False"),
	((read(OFFSET_RXMODE) >> 8) & 0xFF) == 0 ? "AM" : "Undefined",
	(read(OFFSET_RXMODE) & 0x02 ? "True" : "False"),
	(read(OFFSET_RXMODE) & 0x01 ? "True" : "False"),
	read(OFFSET_PHACC1),
	lp->adc.clockFreq);

	msgPtr = buffer;

	return 0;
}

// Initialise all variables in statusregs_local device
void init_vars(void){
	// Display
	lp->display.enabled = 1;

	// ADC
	lp->adc.clockFreq = 80e6;
	lp->adc.random = 1;
	lp->adc.dither = 1;

	// RF
	lp->rf.frequency = 1e6;
	lp->rf.mode = AM;
	lp->rf.transverterOffset = 0;
}

// Update all status registers that can be written to
void update_all(void){

	int bigFreq;  // Is frequency needing to use high range display

	bigFreq = (lp->rf.frequency > 4e9 ? 1 : 0);

	// Display
	if(bigFreq)
		write(OFFSET_DISPFREQ, lp->rf.frequency);  // DEBUG
	else
		write(OFFSET_DISPFREQ, lp->rf.frequency);

	write(OFFSET_DISPMODE,
		(lp->display.enabled << 1) |
		bigFreq
	);

	// RXMODE
	write(OFFSET_RXMODE, 
		((lp->rf.mode << 8) & 0xFF00) | 
		(lp->adc.random << 1) |
		(lp->adc.dither)
	);

	// Phase Accumulator
	write(OFFSET_PHACC1, 7);  // DEBUG
	printk(KERN_INFO "Phase Accumulator Value: %lu", read(OFFSET_PHACC1));		
}



static int status_release(struct inode *inode_p, struct file *file_p){
	deviceOpen--;

	module_put(THIS_MODULE);
	return 0;
}
static ssize_t status_read(struct file *file_p, char *outBuffer_p, size_t length, loff_t *offset){

	int bytesRead;
	bytesRead = 0;

	if(*msgPtr == 0)
		return 0;
	else {
		// Copy the message to the output buffer as long as the output buffer has space and we haven't finished
		while(length && *msgPtr){
			put_user(*msgPtr++, outBuffer_p++);
			length--;
			bytesRead++;
		}
	}

	return bytesRead;
}

static ssize_t status_write(struct file *file_p, const char *inBuffer_p, size_t length, loff_t *offset){
	printk(KERN_ALERT "Status regs: This module does not support write operations yet");
	return -EINVAL;
}


static int statusregs_probe(struct platform_device *pdev)
{
	struct resource *r_mem; /* IO mem resources */
	struct device *dev_p = &pdev->dev;
	printk(KERN_INFO "StatusRegs: Loading");

	int rc = 0;
	/* Get iospace for the device */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(dev_p, "invalid address\n");
		return -ENODEV;
	}
	lp = (struct statusregs_local *) kmalloc(sizeof(struct statusregs_local), GFP_KERNEL);
	if (!lp) {
		dev_err(dev_p, "Cound not allocate statusregs device\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev_p, lp);
	lp->mem_start = r_mem->start;
	lp->mem_end = r_mem->end;

	if (!request_mem_region(lp->mem_start,
				lp->mem_end - lp->mem_start + 1,
				DRIVER_NAME)) {
		dev_err(dev_p, "Couldn't lock memory region at %p\n",
			(void *)lp->mem_start);
		rc = -EBUSY;
		goto error1;
	}

	lp->base_addr = ioremap(lp->mem_start, lp->mem_end - lp->mem_start + 1);
	if (!lp->base_addr) {
		dev_err(dev_p, "statusregs: Could not allocate iomem\n");
		rc = -EIO;
		goto error2;
	}

	init_vars();
	update_all();

	return 0;
error2:
	release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
error1:
	kfree(lp);
	dev_set_drvdata(dev_p, NULL);
	return rc;
}

static int statusregs_remove(struct platform_device *pdev)
{
	struct device *dev_p = &pdev->dev;
	struct statusregs_local *lp = dev_get_drvdata(dev_p);
	iounmap(lp->base_addr);
	release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
	kfree(lp);
	dev_set_drvdata(dev_p, NULL);
	return 0;
}


static struct of_device_id statusregs_of_match[] = {
	{ .compatible = "m0wut,statusregs", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, statusregs_of_match);



static struct platform_driver statusregs_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= statusregs_of_match,
	},
	.probe		= statusregs_probe,
	.remove		= statusregs_remove,
};

static int __init statusregs_init(void)
{
	
	major = register_chrdev(0, DRIVER_NAME, &fops);
	printk(KERN_INFO "Status registered as device major number : %d", major);


	return platform_driver_register(&statusregs_driver);
}


static void __exit statusregs_exit(void)
{
	platform_driver_unregister(&statusregs_driver);
	printk(KERN_ALERT "Status regs module unloaded\n");
}

module_init(statusregs_init);
module_exit(statusregs_exit);
