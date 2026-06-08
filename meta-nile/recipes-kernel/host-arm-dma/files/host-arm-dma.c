/******************************************************************************
 *
 *   Copyright (C) 2026 National Instruments Corporation.
 *
 *   SPDX-License-Identifier: GPL-2.0
 *
 *****************************************************************************/

/*
 * This kernel module supports a proof-of-concept data path where the host DMA
 * engine transfers data for consumption by the PS userspace demo application
 * host-arm-dma-wait. The module receives PL interrupts when a DMA transaction
 * is ready to process, then notifies userspace through an eventfd registered
 * via a custom ioctl for lower-latency signaling. After processing, userspace
 * writes back to this device node to signal to the host that processing is
 * complete.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/eventfd.h>
#include <linux/of_address.h>

#define DRIVER_NAME "host-arm-dma"
#define DRIVER_VERSION "0.1.0"

// Use the AXI GPIO block to receive interrupts from the host and interrupt completions back to the host
// https://docs.amd.com/r/en-US/pg144-axi-gpio/Register-Space
#define AXI_GPIO_GIER_OFFSET 0x011C
#define AXI_GPIO_GIER_ENABLE 0x80000000
#define AXI_GPIO2_DATA_OFFSET 0x0008
#define AXI_GPIO_IP_ISR_OFFSET 0x0120
#define AXI_GPIO_IP_IER_OFFSET 0x0128
#define AXI_GPIO_IP_IER_ENABLE 0x00000001

#define HOST_ARM_DMA_IOC_MAGIC 'H'
#define HOST_ARM_DMA_IOC_SET_EVENTFD \
	_IOW(HOST_ARM_DMA_IOC_MAGIC, 0x01, int)
#define HOST_ARM_DMA_IOC_CLR_EVENTFD \
	_IO(HOST_ARM_DMA_IOC_MAGIC, 0x02)

struct host_arm_dma_dev {
	struct device *dev;
	resource_size_t axi_gpio_0_phys;
	resource_size_t axi_gpio_1_phys;
	void __iomem *axi_gpio_0_base;
	void __iomem *axi_gpio_1_base;
	int irq;
	atomic_long_t irq_count;
	u64 last_isr_ns;
	spinlock_t eventfd_lock;
	struct eventfd_ctx *eventfd;
	struct miscdevice miscdev;
};

static int host_arm_dma_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct host_arm_dma_dev *dma =
		container_of(miscdev, struct host_arm_dma_dev, miscdev);

	file->private_data = dma;

	return 0;
}

static int host_arm_dma_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static ssize_t host_arm_dma_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct host_arm_dma_dev *dma = file->private_data;
	u32 value;
	u64 last_isr_ns;
	u64 write_trigger_ns;
	u64 latency_ns;

	if (!dma)
		return -EINVAL;

	if (count < sizeof(value))
		return -EINVAL;

	if (copy_from_user(&value, buf, sizeof(value)))
		return -EFAULT;

	if (!dma->axi_gpio_1_base)
		return -EINVAL;

	writel(value, dma->axi_gpio_1_base + AXI_GPIO2_DATA_OFFSET);
	write_trigger_ns = ktime_get_ns();
	last_isr_ns = READ_ONCE(dma->last_isr_ns);

	if (last_isr_ns && write_trigger_ns >= last_isr_ns) {
		latency_ns = write_trigger_ns - last_isr_ns;
		dev_info(dma->dev,
			 "Interrupt sent (value: %u, dma->irq_count=%ld), ISR->write latency: %llu ns\n",
			 value, atomic_long_read(&dma->irq_count), latency_ns);
	} else {
		dev_info(dma->dev,
			 "Interrupt sent (value: %u, dma->irq_count=%ld), ISR->write latency unavailable\n",
			 value, atomic_long_read(&dma->irq_count));
	}

	return sizeof(value);
}

static long host_arm_dma_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	struct host_arm_dma_dev *dma = file->private_data;
	struct eventfd_ctx *new_eventfd;
	struct eventfd_ctx *old_eventfd;
	unsigned long flags;
	int eventfd_fd;

	if (!dma)
		return -EINVAL;

	switch (cmd) {
	case HOST_ARM_DMA_IOC_SET_EVENTFD:
		if (copy_from_user(&eventfd_fd, (int __user *)arg,
				   sizeof(eventfd_fd)))
			return -EFAULT;

		new_eventfd = eventfd_ctx_fdget(eventfd_fd);
		if (IS_ERR(new_eventfd))
			return PTR_ERR(new_eventfd);

		spin_lock_irqsave(&dma->eventfd_lock, flags);
		old_eventfd = dma->eventfd;
		dma->eventfd = new_eventfd;
		spin_unlock_irqrestore(&dma->eventfd_lock, flags);

		if (old_eventfd)
			eventfd_ctx_put(old_eventfd);

		return 0;

	case HOST_ARM_DMA_IOC_CLR_EVENTFD:
		spin_lock_irqsave(&dma->eventfd_lock, flags);
		old_eventfd = dma->eventfd;
		dma->eventfd = NULL;
		spin_unlock_irqrestore(&dma->eventfd_lock, flags);

		if (old_eventfd)
			eventfd_ctx_put(old_eventfd);

		return 0;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations host_arm_dma_fops = {
	.owner = THIS_MODULE,
	.open = host_arm_dma_open,
	.release = host_arm_dma_release,
	.write = host_arm_dma_write,
	.unlocked_ioctl = host_arm_dma_ioctl,
	.llseek = default_llseek,
};

static irqreturn_t host_arm_dma_isr(int irq, void *data)
{
	struct host_arm_dma_dev *dma = data;
	struct eventfd_ctx *eventfd;
	unsigned long flags;

	WRITE_ONCE(dma->last_isr_ns, ktime_get_ns());
	atomic_long_inc(&dma->irq_count);

	spin_lock_irqsave(&dma->eventfd_lock, flags);
	eventfd = dma->eventfd;
	if (eventfd)
		eventfd_signal(eventfd);
	spin_unlock_irqrestore(&dma->eventfd_lock, flags);

	// Re-arm the interrupt
	writel(0x1, dma->axi_gpio_0_base + AXI_GPIO_IP_ISR_OFFSET);

	return IRQ_HANDLED;
}

static int host_arm_dma_probe(struct platform_device *pdev)
{
	struct host_arm_dma_dev *dma;
	struct resource res;
	struct device_node *gpio_np;
	int ret;

	dma = devm_kzalloc(&pdev->dev, sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;

	dma->dev = &pdev->dev;
	platform_set_drvdata(pdev, dma);
	atomic_long_set(&dma->irq_count, 0);
	spin_lock_init(&dma->eventfd_lock);
	dma->eventfd = NULL;
	WRITE_ONCE(dma->last_isr_ns, 0);

	/* Get IRQ declared on this device node. With gpio-xilinx removed, this
	 * driver owns the AXI GPIO interrupt line directly (GIC SPI 93). */
	dma->irq = platform_get_irq(pdev, 0);

	if (dma->irq <= 0) {
		dev_err(&pdev->dev, "Failed to get IRQ: %d\n", dma->irq);
		return dma->irq ? dma->irq : -ENXIO;
	}

	/* Register interrupt handler */
	ret = devm_request_irq(&pdev->dev, dma->irq, host_arm_dma_isr,
			       0, DRIVER_NAME, dma);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n",
			dma->irq, ret);
		return ret;
	}
	dev_info(&pdev->dev, "Registered DMA driver, owned IRQ %d\n", dma->irq);

	/* Map AXI GPIO 0 from the DT phandle (ni,axi-gpio-0). */
	gpio_np = of_parse_phandle(pdev->dev.of_node, "ni,axi-gpio-0", 0);
	if (!gpio_np) {
		dev_err(&pdev->dev, "Missing ni,axi-gpio-0 phandle\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(gpio_np, 0, &res);
	of_node_put(gpio_np);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get AXI GPIO 0 resource: %d\n", ret);
		return ret;
	}

	dma->axi_gpio_0_phys = res.start;
	dma->axi_gpio_0_base = ioremap(res.start, resource_size(&res));
	if (!dma->axi_gpio_0_base) {
		dev_err(&pdev->dev, "Failed to map axi_gpio_0 memory\n");
		return -ENOMEM;
	}

	/* Map AXI GPIO 1 from the DT phandle (ni,axi-gpio-1). */
	gpio_np = of_parse_phandle(pdev->dev.of_node, "ni,axi-gpio-1", 0);
	if (!gpio_np) {
		dev_err(&pdev->dev, "Missing ni,axi-gpio-1 phandle\n");
		iounmap(dma->axi_gpio_0_base);
		dma->axi_gpio_0_base = NULL;
		return -EINVAL;
	}

	ret = of_address_to_resource(gpio_np, 0, &res);
	of_node_put(gpio_np);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get AXI GPIO 1 resource: %d\n", ret);
		iounmap(dma->axi_gpio_0_base);
		dma->axi_gpio_0_base = NULL;
		return ret;
	}

	dma->axi_gpio_1_phys = res.start;
	dma->axi_gpio_1_base = ioremap(res.start, resource_size(&res));
	if (!dma->axi_gpio_1_base) {
		dev_err(&pdev->dev, "Failed to map axi_gpio_1 memory\n");
		iounmap(dma->axi_gpio_0_base);
		dma->axi_gpio_0_base = NULL;
		return -ENOMEM;
	}

	/* Enable AXI GPIO global interrupt (GIER, bit 31). */
	writel(AXI_GPIO_GIER_ENABLE, dma->axi_gpio_0_base + AXI_GPIO_GIER_OFFSET);
	/* Enable AXI GPIO IP interrupt source (IP IER, bit 0). */
	writel(AXI_GPIO_IP_IER_ENABLE,
	       dma->axi_gpio_0_base + AXI_GPIO_IP_IER_OFFSET);
	/* Enable AXI GPIO 1 global interrupt (GIER, bit 31). */
	writel(AXI_GPIO_GIER_ENABLE, dma->axi_gpio_1_base + AXI_GPIO_GIER_OFFSET);
	/* Enable AXI GPIO 1 IP interrupt source (IP IER, bit 0). */
	writel(AXI_GPIO_IP_IER_ENABLE,
	       dma->axi_gpio_1_base + AXI_GPIO_IP_IER_OFFSET);

	dev_info(&pdev->dev, "axi_gpio_0 mapped at 0x%llx (virt: %p)\n",
			 (unsigned long long)dma->axi_gpio_0_phys, dma->axi_gpio_0_base);
	dev_info(&pdev->dev, "axi_gpio_1 mapped at 0x%llx (virt: %p)\n",
			 (unsigned long long)dma->axi_gpio_1_phys, dma->axi_gpio_1_base);

	dma->miscdev.minor = MISC_DYNAMIC_MINOR;
	dma->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
		"%s", DRIVER_NAME);
	if (!dma->miscdev.name)
		return -ENOMEM;
	dma->miscdev.fops = &host_arm_dma_fops;
	dma->miscdev.parent = &pdev->dev;

	ret = misc_register(&dma->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register misc device: %d\n", ret);
		iounmap(dma->axi_gpio_1_base);
		dma->axi_gpio_1_base = NULL;
		iounmap(dma->axi_gpio_0_base);
		dma->axi_gpio_0_base = NULL;
		return ret;
	}

	dev_info(&pdev->dev, "Eventfd control interface at /dev/%s\n", dma->miscdev.name);

	return 0;
}

static void host_arm_dma_remove(struct platform_device *pdev)
{
	struct host_arm_dma_dev *dma = platform_get_drvdata(pdev);
	struct eventfd_ctx *eventfd;
	unsigned long flags;

	if (!dma)
		return;

	misc_deregister(&dma->miscdev);

	spin_lock_irqsave(&dma->eventfd_lock, flags);
	eventfd = dma->eventfd;
	dma->eventfd = NULL;
	spin_unlock_irqrestore(&dma->eventfd_lock, flags);

	if (eventfd)
		eventfd_ctx_put(eventfd);

	iounmap(dma->axi_gpio_1_base);
	iounmap(dma->axi_gpio_0_base);

	dev_info(dma->dev, "Unregistered DMA driver (received %ld interrupts)\n",
		 atomic_long_read(&dma->irq_count));
}

static const struct of_device_id host_arm_dma_of_match[] = {
	{ .compatible = "ni,host-arm-dma" },
	{ }
};
MODULE_DEVICE_TABLE(of, host_arm_dma_of_match);

static struct platform_driver host_arm_dma_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = host_arm_dma_of_match,
	},
	.probe = host_arm_dma_probe,
	.remove = host_arm_dma_remove,
};

static int __init host_arm_dma_init(void)
{
	pr_info("Host ARM DMA interrupt driver v%s\n", DRIVER_VERSION);
	return platform_driver_register(&host_arm_dma_driver);
}

static void __exit host_arm_dma_exit(void)
{
	platform_driver_unregister(&host_arm_dma_driver);
}

module_init(host_arm_dma_init);
module_exit(host_arm_dma_exit);

MODULE_AUTHOR("National Instruments");
MODULE_DESCRIPTION("Host ARM DMA interrupt-driven device driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
