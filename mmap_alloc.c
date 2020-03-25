
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#ifdef MODVERSIONS
#  include <linux/modversions.h>
#endif
#include <asm/io.h>

/*
 * Example of driver that allows a user-space program to mmap a buffer of
 * contiguous non-cached physical memory.
 * Note that pages are contiguous in physical memory but not in virtual memory.
 * Based on the original work made by Martin Frey <frey@scs.ch>.
 *
 * Authors: Claudio Scordino, Bruno Morelli
 */

static int dma_mask_bit = 32;

/* character device structures */
static dev_t mmap_dev;
static struct cdev mmap_cdev;
static struct class *cl; // Global variable for the device class
struct device *mydev;

/* methods of the character device */
static int mmap_open(struct inode *inode, struct file *filp);
static int mmap_release(struct inode *inode, struct file *filp);
static int mmap_mmap(struct file *filp, struct vm_area_struct *vma);

/* the file operations, i.e. all character device methods */
static struct file_operations mmap_fops = {
    .open = mmap_open,
    .release = mmap_release,
    .mmap = mmap_mmap,
    .owner = THIS_MODULE,
};

// length of the two memory areas
#define NPAGES 16
// pointer to the allocated area, rounded up to a page boundary
static int *alloc_area;
// original pointer for allocated area
static void *alloc_ptr;
dma_addr_t dma_handle;

/* character device open method */
static int mmap_open(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "mmap_alloc: device open\n");
    return 0;
}
/* character device last close method */
static int mmap_release(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "mmap_alloc: device is being released\n");
    return 0;
}

// helper function, mmap's the allocated area which is physically contiguous
int mmap_kmem(struct file *filp, struct vm_area_struct *vma)
{
    int ret;
    long length = vma->vm_end - vma->vm_start;

    /* check length - do not allow larger mappings than the number of
        pages allocated */
    if (length > NPAGES * PAGE_SIZE)
        return -EIO;

    printk(KERN_INFO "Using dma_mmap_coherent\n");
    ret = dma_mmap_coherent(mydev, vma, alloc_ptr, dma_handle, length);

    if (ret < 0) {
        printk(KERN_ERR "mmap_alloc: remap failed (%d)\n", ret);
        return ret;
    }

    return 0;
}

/* character device mmap method */
static int mmap_mmap(struct file *filp, struct vm_area_struct *vma)
{
    printk(KERN_INFO "mmap_alloc: device is being mapped\n");
    return mmap_kmem(filp, vma);
}

/* module initialization - called at module load time */
static int __init mmap_alloc_init(void)
{
    int ret = 0;
    int i;

    /* get the major number of the character device */
    if ((ret = alloc_chrdev_region(&mmap_dev, 0, 1, "mmap_alloc")) < 0) {
        printk(KERN_ERR "mmap_alloc: could not allocate major number for mmap\n");
        goto out;
    }

    if (IS_ERR(cl = class_create(THIS_MODULE, "mmap_alloc")))
    {
        ret = PTR_ERR(cl);
        goto out_unalloc_region;
    }

    if (IS_ERR(mydev = device_create(cl, NULL, mmap_dev, NULL, "mmap_alloc")))
    {
        ret = PTR_ERR(mydev);
        goto out_class_destroy;
    }

    /* initialize the device structure and register the device with the
    * kernel */
    cdev_init(&mmap_cdev, &mmap_fops);
    if ((ret = cdev_add(&mmap_cdev, mmap_dev, 1)) < 0) {
        printk(KERN_ERR "mmap_alloc: could not allocate chrdev for mmap\n");
        goto out_device_destroy;
    }

    if (mydev->dma_mask == NULL) {
        mydev->dma_mask = &mydev->coherent_dma_mask;
    }
    /*
    * set this->dma_mask
    */
    if (*mydev->dma_mask == 0) {
        if (dma_set_mask(mydev, DMA_BIT_MASK(dma_mask_bit)) == 0) {
            dma_set_coherent_mask(mydev, DMA_BIT_MASK(dma_mask_bit));
        } else {
            printk(KERN_WARNING "dma_set_mask(DMA_BIT_MASK(%d)) failed\n", dma_mask_bit);
            dma_set_mask(mydev, DMA_BIT_MASK(32));
            dma_set_coherent_mask(mydev, DMA_BIT_MASK(32));
            ret = -ENOMEM;
            goto out_device_destroy;
        }
    }

    printk(KERN_INFO "DMA Mask %llx\n", *mydev->dma_mask);

    /* Allocate not-cached memory area with dma_map_coherent. */
    printk(KERN_INFO "Use dma_alloc_coherent\n");
    alloc_ptr = dma_alloc_coherent (mydev, (NPAGES + 2) * PAGE_SIZE, &dma_handle, GFP_KERNEL);

    if (!alloc_ptr) {
        printk(KERN_ERR "mmap_alloc: dma_alloc_coherent error\n");
        ret = -ENOMEM;
        goto out_device_destroy;
    }
    printk(KERN_INFO "mmap_alloc: physical address is %u\n", dma_handle);

    alloc_area = alloc_ptr;

    /* store a pattern in the memory.
    * the test application will check for it */
    for (i = 0; i < (NPAGES * PAGE_SIZE / sizeof(int)); i += 2) {
        alloc_area[i] = (0xdead << 16) + i;
        alloc_area[i + 1] = (0xbeef << 16) + i;
    }

    return ret;

    out_device_destroy:
        device_destroy(cl, mmap_dev);
    out_class_destroy:
        class_destroy(cl);
    out_unalloc_region:
        unregister_chrdev_region(mmap_dev, 1);
    out:
        return ret;
}

/* module unload */
static void __exit mmap_alloc_exit(void)
{
    /* remove the character deivce */
    cdev_del(&mmap_cdev);
    device_destroy(cl, mmap_dev);
    class_destroy(cl);
    unregister_chrdev_region(mmap_dev, 1);

    /* free the memory areas */
    dma_free_coherent (mydev, (NPAGES + 2) * PAGE_SIZE, alloc_ptr, dma_handle);
    printk(KERN_INFO "mmap_alloc unregistered\n");
}

module_init(mmap_alloc_init);
module_exit(mmap_alloc_exit);
MODULE_DESCRIPTION("mmap_alloc driver");
MODULE_AUTHOR("Claudio Scordino and Bruno Morelli");
MODULE_LICENSE("GPL");