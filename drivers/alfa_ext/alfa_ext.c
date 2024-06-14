/*
 * Copyright 2023 ALFA Project. All rights reserved.
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#define DEVICE_NAME "alfa_ext"
#define CLASS_NAME "alfa_ext_class"
#define NUM_REGIONS 1

#define ALFA_EXT_IOC_MAGIC 'k'

// IOCTL commands
#define ALFA_EXT_IOC_WRITE_REGISTER \
  _IOW(ALFA_EXT_IOC_MAGIC, 1, struct alfa_ext_ioctl_data)
#define ALFA_EXT_IOC_READ_REGISTER \
  _IOR(ALFA_EXT_IOC_MAGIC, 2, struct alfa_ext_ioctl_data)
#define ALFA_EXT_IOC_WAIT_FOR_VALUE \
  _IOW(ALFA_EXT_IOC_MAGIC, 3, struct alfa_ext_ioctl_data)

struct alfa_ext_ioctl_data {
  unsigned int extension_id;  // ID of the extension region
  unsigned int offset;        // Offset of the register
  unsigned int value;         // Value to write or the read value
};

static struct class *alfa_ext_class;
static struct device *alfa_ext_device;
static void __iomem *alfa_base_addr[NUM_REGIONS];
static resource_size_t alfa_base_phys[NUM_REGIONS];
static resource_size_t alfa_size[NUM_REGIONS];
static bool file_open = false;

// Define DEBUG_FLAG if not defined
#ifndef DEBUG_FLAG
// #define DEBUG_FLAG
#endif

// Macro for conditional pr_info logging
#ifdef DEBUG_FLAG
#define LOG_INFO(fmt, ...) pr_info(fmt, ##__VA_ARGS__)
#else
#define LOG_INFO(fmt, ...) \
  do {                     \
  } while (0)
#endif

static int alfa_ext_open(struct inode *inode, struct file *file) {
  if (file_open) {
    pr_err("alfa_ext: Device already open\n");
    return -EBUSY;
  }

  printk("alfa_ext: Opening device\n");
  file_open = true;
  return 0;
}

static int alfa_ext_release(struct inode *inode, struct file *file) {
  printk("alfa_ext: Releasing device\n");
  file_open = false;
  return 0;
}

static long alfa_ext_ioctl(struct file *file, unsigned int cmd,
                           unsigned long arg) {
  struct alfa_ext_ioctl_data data;
  void __iomem *addr;
  unsigned int max_retries = 500;  // Maximum of 500 entries
  unsigned int delay_us = 100;     // Delay of 0.1 ms (100 microseconds)
  unsigned int value;

  if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
    pr_err("alfa_ext: Failed to copy data from user space\n");
    return -EFAULT;
  }

  LOG_INFO(
      "alfa_ext: IOCTL called with cmd: %u, extension_id: %u, offset: 0x%x, "
      "value: %u\n",
      cmd, data.extension_id, data.offset, data.value);

  if (data.extension_id >= NUM_REGIONS ||
      data.offset >= alfa_size[data.extension_id]) {
    pr_err("alfa_ext: Invalid extension ID or offset\n");
    return -EINVAL;
  }

  addr = alfa_base_addr[data.extension_id] + data.offset;

  switch (cmd) {
    case ALFA_EXT_IOC_WRITE_REGISTER:
      LOG_INFO("alfa_ext: Write register IOCTL\n");
      iowrite32(data.value, addr);
      wmb();  // Ensure the write is completed
      break;

    case ALFA_EXT_IOC_READ_REGISTER:
      LOG_INFO("alfa_ext: Read register IOCTL\n");
      data.value = ioread32(addr);
      rmb();  // Ensure the read is completed

      if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
        pr_err("alfa_ext: Failed to copy data to user space\n");
        return -EFAULT;
      }
      break;

    case ALFA_EXT_IOC_WAIT_FOR_VALUE:
      LOG_INFO("alfa_ext: Wait for value IOCTL\n");
      unsigned int retries;
      for (retries = 0; retries < max_retries; retries++) {
        value = ioread32(addr);
        rmb();  // Ensure the read is completed

        if (value == data.value) {
          LOG_INFO("alfa_ext: Value changed to %u at offset 0x%x\n", data.value,
                   data.offset);
          return 0;
        }
        // Sleep for 0.01 ms
        usleep_range(delay_us, delay_us + 10);
      }

      // pr_err("alfa_ext: Timeout waiting for value change at offset 0x%x\n",
      //        data.offset);
      return -ETIMEDOUT;

    default:
      pr_err("alfa_ext: Invalid IOCTL command\n");
      return -ENOTTY;
  }

  return 0;
}

static const struct file_operations alfa_fops = {
    .owner = THIS_MODULE,
    .open = alfa_ext_open,
    .release = alfa_ext_release,
    .unlocked_ioctl = alfa_ext_ioctl,
};

static int alfa_ext_probe(struct platform_device *pdev) {
  struct resource *res;
  int ret, i;

  for (i = 0; i < NUM_REGIONS; i++) {
    res = platform_get_resource(pdev, IORESOURCE_MEM, i);
    if (!res) {
      pr_err("alfa_ext: Failed to get resource %d\n", i);
      return -ENODEV;
    }

    alfa_base_phys[i] = res->start;
    alfa_size[i] = resource_size(res);
    alfa_base_addr[i] = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(alfa_base_addr[i])) {
      pr_err("alfa_ext: Failed to remap IO for region %d\n", i);
      return PTR_ERR(alfa_base_addr[i]);
    }

    LOG_INFO("alfa_ext: Region %d - phys: %pa, size: %pa\n", i,
             &alfa_base_phys[i], &alfa_size[i]);
  }

  // Create device class
  if (!alfa_ext_class) {
    alfa_ext_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(alfa_ext_class)) {
      pr_err("alfa_ext: Failed to create device class");
      return PTR_ERR(alfa_ext_class);
    }
  }

  // Register character device
  ret = register_chrdev(0, DEVICE_NAME, &alfa_fops);
  if (ret < 0) {
    pr_err("alfa_ext: Failed to register character device\n");
    class_destroy(alfa_ext_class);
    return ret;
  }

  alfa_ext_device =
      device_create(alfa_ext_class, NULL, MKDEV(ret, 0), NULL, DEVICE_NAME);
  if (IS_ERR(alfa_ext_device)) {
    unregister_chrdev(ret, DEVICE_NAME);
    class_destroy(alfa_ext_class);
    return PTR_ERR(alfa_ext_device);
  }

  LOG_INFO("alfa_ext: Device probed successfully\n");
  return 0;
}

static int alfa_ext_remove(struct platform_device *pdev) {
  unregister_chrdev(0, DEVICE_NAME);
  LOG_INFO("alfa_ext: Device removed\n");
  if (alfa_ext_class) {
    device_destroy(alfa_ext_class, MKDEV(0, 0));
    class_destroy(alfa_ext_class);
    alfa_ext_class = NULL;
  }
  return 0;
}

static const struct of_device_id alfa_ext_of_ids[] = {
    {.compatible = "alfa_ext"}, {}};
MODULE_DEVICE_TABLE(of, alfa_ext_of_ids);

static struct platform_driver alfa_ext_driver = {
    .probe = alfa_ext_probe,
    .remove = alfa_ext_remove,
    .driver =
        {
            .name = "alfa_ext",
            .owner = THIS_MODULE,
            .of_match_table = alfa_ext_of_ids,
        },
};

static int __init alfa_ext_init(void) {
  return platform_driver_register(&alfa_ext_driver);
}

static void __exit alfa_ext_exit(void) {
  platform_driver_unregister(&alfa_ext_driver);
}

module_init(alfa_ext_init);
module_exit(alfa_ext_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ricardo Roriz");
MODULE_DESCRIPTION("ALFA Device driver to map ext registers to user space");
