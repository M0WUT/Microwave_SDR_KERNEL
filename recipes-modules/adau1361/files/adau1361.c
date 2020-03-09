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
#include "axi_iic.h"

/* Standard module information, edit as appropriate */
MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("Dan McGraw M0WUT");
MODULE_DESCRIPTION
    ("Kernel driver for ADAU1361 audio codec");

#define DRIVER_NAME "adau1361"


struct adau1361_local {
	struct iic_local iic;
};

static int adau1361_probe(struct platform_device *pdev)
{
	int r_irq; /* Interrupt resources */
	struct resource *r_mem; /* IO mem resources */
	struct device *dev = &pdev->dev;
	struct device_node *node_p = pdev->dev.of_node;
	struct adau1361_local *adau1361_dev = NULL;
	int rc = 0;

	printk(KERN_INFO "Probing ADAU1361 module");
	/* Get iospace for the device */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(dev, "Invalid address passed in reg parameter\n");
		return -ENODEV;
	}
	adau1361_dev = (struct adau1361_local *) kmalloc(sizeof(struct adau1361_local), GFP_KERNEL);
	if (!adau1361_dev) {
		dev_err(dev, "Cound not allocate adau1361 device\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, adau1361_dev);
	adau1361_dev->iic.mem_start = r_mem->start;
	adau1361_dev->iic.mem_end = r_mem->end;

	if (!request_mem_region(adau1361_dev->iic.mem_start,
				adau1361_dev->iic.mem_end - adau1361_dev->iic.mem_start + 1,
				DRIVER_NAME)) {
		dev_err(dev, "Couldn't lock memory region at %p\n",
			(void *)adau1361_dev->iic.mem_start);
		rc = -EBUSY;
		goto error1;
	}

	adau1361_dev->iic.base_addr = ioremap(adau1361_dev->iic.mem_start, adau1361_dev->iic.mem_end - adau1361_dev->iic.mem_start + 1);
	if (!adau1361_dev->iic.base_addr) {
		dev_err(dev, "adau1361: Could not allocate iomem\n");
		rc = -EIO;
		goto error2;
	}

	// Request Mutex for the I2C TX Lock
	mutex_init(&adau1361_dev->iic.tx_lock);

	/* Get IRQ for the device */
	//r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	r_irq = platform_get_irq(pdev, 0);
	if (r_irq < 0) {
		dev_info(dev, "no IRQ found\n");
		dev_info(dev, "skeleton at 0x%08x mapped to 0x%08x\n",
			(unsigned int __force)adau1361_dev->iic.mem_start,
			(unsigned int __force)adau1361_dev->iic.base_addr);
		return 0;
	}

	adau1361_dev->iic.irq = r_irq;
	// rc = request_irq(adau1361_dev->iic.irq, &iic_irq, 0, DRIVER_NAME, &adau1361_dev->iic);
	rc = devm_request_threaded_irq(&pdev->dev, r_irq, iic_irq, iic_irq_process, IRQF_ONESHOT, DRIVER_NAME, &adau1361_dev->iic);
	if (rc) {
		dev_err(dev, "testmodule: Could not allocate interrupt %d.\n",
			adau1361_dev->iic.irq);
		goto error3;
	}

	// Read device tree parameters and sanitise
	rc = of_property_read_u32(node_p, "slave_address", &adau1361_dev->iic.slave_address);
	if (rc) {
		dev_err(&pdev->dev, "Can't parse I2C slave address\n");
	}
	#ifdef DEBUG
		printk(KERN_INFO "ADAU1361: slave I2C address loaded as %04X", adau1361_dev->iic.slave_address);
	#endif

	// Initialise I2C peripheral
	iic_init(&adau1361_dev->iic);
	iic_write(&adau1361_dev->iic, 0x4019, 0xAA);

	#ifdef DEBUG
		printk(KERN_INFO "ADAU1361: All done, returning 0 now");
	#endif
	return 0;


error3:
	free_irq(adau1361_dev->iic.irq, adau1361_dev);
error2:
	release_mem_region(adau1361_dev->iic.mem_start, adau1361_dev->iic.mem_end - adau1361_dev->iic.mem_start + 1);
error1:
	kfree(adau1361_dev);
	dev_set_drvdata(dev, NULL);
	return rc;
}

static int adau1361_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adau1361_local *adau1361_dev = dev_get_drvdata(dev);
	iounmap(adau1361_dev->iic.base_addr);
	release_mem_region(adau1361_dev->iic.mem_start, adau1361_dev->iic.mem_end - adau1361_dev->iic.mem_start + 1);
	kfree(adau1361_dev);
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
	return platform_driver_register(&adau1361_driver);
}


static void __exit adau1361_exit(void)
{
	platform_driver_unregister(&adau1361_driver);
	printk(KERN_ALERT "Goodbye module world.\n");
}

module_init(adau1361_init);
module_exit(adau1361_exit);
