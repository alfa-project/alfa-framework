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

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#define DEVICE_NAME "alfa_mem"
#define CLASS_NAME "alfa_mem_class"
#define NUM_BUFFERS 8
#define BUFFER_SIZE (0x200000)  // 2MB per buffer

#define ALFA_MEM_IOC_MAGIC 'k'
#define ALFA_MEM_IOC_GET_PHYS_ADDR _IOWR(ALFA_MEM_IOC_MAGIC, 1, unsigned int)
#define ALFA_MEM_IOC_FLUSH_CACHE _IO(ALFA_MEM_IOC_MAGIC, 2)

static struct class *alfa_mem_class;
static struct device *alfa_mem_device;
static dma_addr_t buffer_phys[NUM_BUFFERS];
static void *buffer_virt[NUM_BUFFERS];
static bool file_open = false;

static int alfa_mem_open(struct inode *inode, struct file *file) {
  if (file_open) {
    pr_err("alfa_mem: Device already open\n");
    return -EBUSY;
  }

  printk("alfa_mem: Opening device\n");
  file_open = true;
  return 0;
}

static int alfa_mem_release(struct inode *inode, struct file *file) {
  printk("alfa_mem: Releasing device\n");
  file_open = false;
  return 0;
}

static long alfa_mem_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
  unsigned int buffer_id;
  uint32_t phys_addr;

  pr_info("alfa_mem: IOCTL called with cmd: %u, arg: %lu\n", cmd, arg);

  switch (cmd) {
    case ALFA_MEM_IOC_GET_PHYS_ADDR:
      pr_info("alfa_mem: ALFA_MEM_IOC_GET_PHYS_ADDR command received\n");
      if (copy_from_user(&buffer_id, (unsigned int *)arg, sizeof(unsigned int))) {
        pr_err("alfa_mem: Failed to copy buffer_id from user space\n");
        return -EFAULT;
      }

      pr_info("alfa_mem: Received buffer_id: %u\n", buffer_id);

      if (buffer_id >= NUM_BUFFERS) {
        pr_err("alfa_mem: Invalid buffer_id: %u\n", buffer_id);
        return -EINVAL;
      }

      phys_addr = (uint32_t)buffer_phys[buffer_id];
      pr_info("alfa_mem: Physical address of buffer %u is 0x%x\n", buffer_id, phys_addr);

      if (copy_to_user((unsigned int *)arg, &phys_addr, sizeof(uint32_t))) {
        pr_err("alfa_mem: Failed to copy phys_addr to user space\n");
        return -EFAULT;
      }
      break;

    case ALFA_MEM_IOC_FLUSH_CACHE:
      pr_info("alfa_mem: ALFA_MEM_IOC_FLUSH_CACHE command received\n");
      if (copy_from_user(&buffer_id, (unsigned int *)arg, sizeof(unsigned int))) {
        pr_err("alfa_mem: Failed to copy buffer_id from user space\n");
        return -EFAULT;
      }

      pr_info("alfa_mem: Received buffer_id: %u\n", buffer_id);

      if (buffer_id >= NUM_BUFFERS) {
        pr_err("alfa_mem: Invalid buffer_id: %u\n", buffer_id);
        return -EINVAL;
      }

      if (buffer_virt[buffer_id]) {
        pr_info("alfa_mem: Flushing cache for buffer %u\n", buffer_id);
        mb();
        dma_sync_single_for_cpu(alfa_mem_device, buffer_phys[buffer_id], BUFFER_SIZE,
                                DMA_TO_DEVICE);
        dma_sync_single_for_device(alfa_mem_device, buffer_phys[buffer_id], BUFFER_SIZE,
                                   DMA_FROM_DEVICE);
      }
      break;

    default:
      pr_err("alfa_mem: Invalid IOCTL command\n");
      return -ENOTTY;
  }

  pr_info("alfa_mem: IOCTL command completed successfully\n");
  return 0;
}

static int alfa_mem_mmap(struct file *filp, struct vm_area_struct *vma) {
  unsigned long size = vma->vm_end - vma->vm_start;
  unsigned int buffer_id = vma->vm_pgoff;

  if (buffer_id >= NUM_BUFFERS || size > BUFFER_SIZE) {
    pr_err("alfa_mem: Invalid buffer_id %d or size %lu\n", buffer_id, size);
    return -EINVAL;
  }

  dma_addr_t dma_handle = buffer_phys[buffer_id];
  unsigned long pfn = dma_handle >> PAGE_SHIFT;

  pr_info("alfa_mem: Mapping buffer %d - dma_handle: 0x%lx, pfn: %lx\n", buffer_id,
          (unsigned long)dma_handle, pfn);

  if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
    pr_err("alfa_mem: Failed to remap memory for buffer %d\n", buffer_id);
    return -EAGAIN;
  }

  return 0;
}

static const struct file_operations alfa_fops = {
    .owner = THIS_MODULE,
    .open = alfa_mem_open,
    .release = alfa_mem_release,
    .unlocked_ioctl = alfa_mem_ioctl,
    .mmap = alfa_mem_mmap,
};

static int alfa_mem_probe(struct platform_device *pdev) {
  int ret, i;

  // Allocate DMA buffers
  for (i = 0; i < NUM_BUFFERS; i++) {
    buffer_virt[i] = dma_alloc_coherent(&pdev->dev, BUFFER_SIZE, &buffer_phys[i], GFP_KERNEL);
    if (!buffer_virt[i]) {
      pr_err("alfa_mem: Failed to allocate DMA buffer %d\n", i);
      ret = -ENOMEM;
      goto err_alloc;
    }
    pr_info("alfa_mem: Allocated buffer %d - phys: 0x%x\n", i, (uint32_t)buffer_phys[i]);
  }

  // Create device class
  if (!alfa_mem_class) {
    alfa_mem_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(alfa_mem_class)) {
      pr_err("alfa_mem: Failed to create device class");
      ret = PTR_ERR(alfa_mem_class);
      goto err_alloc;
    }
  }

  // Register character device
  ret = register_chrdev(0, DEVICE_NAME, &alfa_fops);
  if (ret < 0) {
    pr_err("alfa_mem: Failed to register character device\n");
    class_destroy(alfa_mem_class);
    goto err_alloc;
  }

  alfa_mem_device = device_create(alfa_mem_class, NULL, MKDEV(ret, 0), NULL, DEVICE_NAME);
  if (IS_ERR(alfa_mem_device)) {
    unregister_chrdev(ret, DEVICE_NAME);
    class_destroy(alfa_mem_class);
    ret = PTR_ERR(alfa_mem_device);
    goto err_alloc;
  }

  pr_info("alfa_mem: Device probed successfully\n");
  return 0;

err_alloc:
  for (i = 0; i < NUM_BUFFERS; i++) {
    if (buffer_virt[i]) {
      dma_free_coherent(&pdev->dev, BUFFER_SIZE, buffer_virt[i], buffer_phys[i]);
    }
  }
  return ret;
}

static int alfa_mem_remove(struct platform_device *pdev) {
  int i;

  unregister_chrdev(0, DEVICE_NAME);
  if (alfa_mem_class) {
    device_destroy(alfa_mem_class, MKDEV(0, 0));
    class_destroy(alfa_mem_class);
    alfa_mem_class = NULL;
  }

  // Free DMA buffers
  for (i = 0; i < NUM_BUFFERS; i++) {
    if (buffer_virt[i]) {
      dma_free_coherent(&pdev->dev, BUFFER_SIZE, buffer_virt[i], buffer_phys[i]);
    }
  }

  pr_info("alfa_mem: Device removed\n");
  return 0;
}

static const struct of_device_id alfa_mem_of_ids[] = {{.compatible = "alfa_mem"}, {}};
MODULE_DEVICE_TABLE(of, alfa_mem_of_ids);

static struct platform_driver alfa_mem_driver = {
    .probe = alfa_mem_probe,
    .remove = alfa_mem_remove,
    .driver =
        {
            .name = "alfa_mem",
            .owner = THIS_MODULE,
            .of_match_table = alfa_mem_of_ids,
        },
};

static int __init alfa_mem_init(void) { return platform_driver_register(&alfa_mem_driver); }

static void __exit alfa_mem_exit(void) { platform_driver_unregister(&alfa_mem_driver); }

module_init(alfa_mem_init);
module_exit(alfa_mem_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ricardo Roriz");
MODULE_DESCRIPTION("ALFA Device driver to map pc physical memory to user space");
