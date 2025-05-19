/*
 * Copyright 2025 ALFA Project. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <asm/page.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define DEVICE_NAME "alfa_ext_mem"
#define CLASS_NAME "alfa_ext_mem_class"
#define DMA_BUF_SIZE 0xA00000  // 10MB buffer size

#define ALFA_EXT_MEM_GET_PHYS_ADDR _IOR('a', 1, unsigned long)

static struct class *alfa_ext_mem_class;
static struct platform_device *g_pdev = NULL;
static void *dma_buf;
static dma_addr_t dma_handle;
static bool file_open = false;

static int alfa_ext_mem_open(struct inode *inode, struct file *filp) {
  if (file_open) {
    pr_err(
        "alfa_ext_mem: Failed to open /dev/alfa_ext_mem because it is already "
        "open\n");
    return -EBUSY;
  }

  printk("alfa_ext_mem: Opening /dev/alfa_ext_mem\n");
  file_open = true;
  return 0;
}

static int alfa_ext_mem_release(struct inode *inode, struct file *filp) {
  if (!file_open) {
    pr_err(
        "alfa_ext_mem: Failed to release /dev/alfa_ext_mem because it is not "
        "open\n");
    return -EBUSY;
  }
  printk("alfa_ext_mem: Closing /dev/alfa_ext_mem\n");
  file_open = false;
  return 0;
}

static long alfa_ext_mem_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
  printk("alfa_ext_mem: ioctl cmd=%d\n", cmd);
  switch (cmd) {
    case ALFA_EXT_MEM_GET_PHYS_ADDR:
      if (copy_to_user((void __user *)arg, &dma_handle, sizeof(dma_handle))) {
        return -EFAULT;
      }
      break;
    default:
      return -EINVAL;
  }
  return 0;
}

static int alfa_ext_mem_mmap(struct file *filp, struct vm_area_struct *vma) {
  unsigned long size = vma->vm_end - vma->vm_start;
  unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

  printk("alfa_ext_mem: mmap size=%lx, off=%lx\n", size, off);

  if (size > DMA_BUF_SIZE) {
    pr_err("alfa_ext_mem: Invalid mmap size: %lx (max: %lx)\n", size, (unsigned long)DMA_BUF_SIZE);
    return -EINVAL;
  }

  if (off != 0) {
    pr_err("alfa_ext_mem: Invalid mmap offset: %lx\n", off);
    return -EINVAL;
  }

  unsigned long pfn = dma_handle >> PAGE_SHIFT;
  if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
    pr_err("alfa_ext_mem: Failed to remap DMA buffer\n");
    return -EAGAIN;
  }

  return 0;
}

static const struct file_operations alfa_ext_mem_fops = {
    .owner = THIS_MODULE,
    .open = alfa_ext_mem_open,
    .release = alfa_ext_mem_release,
    .unlocked_ioctl = alfa_ext_mem_ioctl,
    .mmap = alfa_ext_mem_mmap,
};

static int alfa_ext_mem_probe(struct platform_device *pdev) {
  int ret;

  g_pdev = pdev;

  // Allocate a single DMA buffer
  dma_buf = dma_alloc_coherent(&pdev->dev, DMA_BUF_SIZE, &dma_handle, GFP_KERNEL);
  if (!dma_buf) {
    dev_err(&pdev->dev, "Failed to allocate DMA buffer\n");
    return -ENOMEM;
  }

  dev_info(&pdev->dev, "Allocated DMA buffer at virtual address %p, physical address %pad\n",
           dma_buf, &dma_handle);

  // Create device class
  if (!alfa_ext_mem_class) {
    alfa_ext_mem_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(alfa_ext_mem_class)) {
      pr_err("alfa_ext_mem: Failed to create device class");
      ret = PTR_ERR(alfa_ext_mem_class);
      dma_free_coherent(&pdev->dev, DMA_BUF_SIZE, dma_buf, dma_handle);
      return ret;
    }
  }

  // Create a character device to access the mapped memory
  ret = register_chrdev(0, DEVICE_NAME, &alfa_ext_mem_fops);
  if (ret < 0) {
    pr_err("alfa_ext_mem: Failed to register character device\n");
    if (alfa_ext_mem_class) {
      class_destroy(alfa_ext_mem_class);
    }
    dma_free_coherent(&pdev->dev, DMA_BUF_SIZE, dma_buf, dma_handle);
    return ret;
  }

  device_create(alfa_ext_mem_class, NULL, MKDEV(ret, 0), NULL, DEVICE_NAME);
  pr_info("alfa_ext_mem: ALFA_EXT_MEM probed successfully\n");
  return 0;
}

static int alfa_ext_mem_remove(struct platform_device *pdev) {
  unregister_chrdev(0, DEVICE_NAME);
  pr_info("alfa_ext_mem: Unmapped physical memory\n");
  if (alfa_ext_mem_class) {
    class_destroy(alfa_ext_mem_class);
    alfa_ext_mem_class = NULL;
  }
  dma_free_coherent(&pdev->dev, DMA_BUF_SIZE, dma_buf, dma_handle);
  g_pdev = NULL;
  return 0;
}

static const struct of_device_id alfa_ext_mem_of_ids[] = {{.compatible = "alfa_ext_mem"}, {}};
MODULE_DEVICE_TABLE(of, alfa_ext_mem_of_ids);

static struct platform_driver alfa_ext_mem_driver = {
    .probe = alfa_ext_mem_probe,
    .remove = alfa_ext_mem_remove,
    .driver =
        {
            .name = "alfa_ext_mem",
            .owner = THIS_MODULE,
            .of_match_table = alfa_ext_mem_of_ids,
        },
};

static int __init alfa_ext_mem_init(void) { return platform_driver_register(&alfa_ext_mem_driver); }

static void __exit alfa_ext_mem_exit(void) { platform_driver_unregister(&alfa_ext_mem_driver); }

module_init(alfa_ext_mem_init);
module_exit(alfa_ext_mem_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ricardo Roriz");
MODULE_DESCRIPTION("ALFA Device driver to map ext mem to user space");
