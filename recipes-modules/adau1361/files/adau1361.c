/*  adau1361.c - The simplest kernel module.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 3 of the License, or
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

/* Standard module information, edit as appropriate */
MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("Dan McGraw M0WUT");
MODULE_DESCRIPTION
    ("Kernel driver for ADAU1361 audio codec");

#define DRIVER_NAME "adau1361"


struct adau1361_local {
	int irq;
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
};

static int adau1361_probe(struct platform_device *pdev)
{
	struct resource *r_mem; /* IO mem resources */
	struct device *dev = &pdev->dev;
	struct adau1361_local *lp = NULL;
	int rc = 0;
	dev_info(dev, "Device Tree Probing\n");
	/* Get iospace for the device */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(dev, "Invalid address passed in reg parameter\n");
		return -ENODEV;
	}
	lp = (struct adau1361_local *) kmalloc(sizeof(struct adau1361_local), GFP_KERNEL);
	if (!lp) {
		dev_err(dev, "Cound not allocate adau1361 device\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, lp);
	lp->mem_start = r_mem->start;
	lp->mem_end = r_mem->end;

	if (!request_mem_region(lp->mem_start,
				lp->mem_end - lp->mem_start + 1,
				DRIVER_NAME)) {
		dev_err(dev, "Couldn't lock memory region at %p\n",
			(void *)lp->mem_start);
		rc = -EBUSY;
		goto error1;
	}

	lp->base_addr = ioremap(lp->mem_start, lp->mem_end - lp->mem_start + 1);
	if (!lp->base_addr) {
		dev_err(dev, "adau1361: Could not allocate iomem\n");
		rc = -EIO;
		goto error2;
	}

	dev_info(dev,"adau1361 at 0x%08x mapped to 0x%08x, irq=%d\n",
		(unsigned int __force)lp->mem_start,
		(unsigned int __force)lp->base_addr,
		lp->irq);
	return 0;

error2:
	release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
error1:
	kfree(lp);
	dev_set_drvdata(dev, NULL);
	return rc;
}

static int adau1361_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adau1361_local *lp = dev_get_drvdata(dev);
	iounmap(lp->base_addr);
	release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
	kfree(lp);
	dev_set_drvdata(dev, NULL);
	return 0;
}


static struct of_device_id adau1361_of_match[] = {
	{ .compatible = "m0wut,adau1361", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adau1361_of_match);

static struct platform_driver adau1361_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= adau1361_of_match,
	},
	.probe		= adau1361_probe,
	.remove		= adau1361_remove,
};

static int __init adau1361_init(void)
{
	printk("<1>Hello module world.\n");
	printk("<1>Module parameters were ");

	return platform_driver_register(&adau1361_driver);
}


static void __exit adau1361_exit(void)
{
	platform_driver_unregister(&adau1361_driver);
	printk(KERN_ALERT "Goodbye module world.\n");
}

module_init(adau1361_init);
module_exit(adau1361_exit);
