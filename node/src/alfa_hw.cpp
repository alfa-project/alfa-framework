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

#include "alfa_node.hpp"

extern "C" {
#include <sys/ioctl.h>
}

// Define custom ioctl commands
#define ALFA_IOCTL_TRIGGER_READ _IO('k', 1)
#define ALFA_IOCTL_TRIGGER_WRITE _IO('k', 2)
#define ALFA_IOCTL_CHECK_MEM _IO('k', 3)

bool AlfaNode::hardware_setup() {
  bool return_value = 0;

  if (((ext_fd = open("/dev/alfa_ext", O_RDWR | O_SYNC)) != -1) &&
      ((mem_fd = open("/dev/alfa_mem", O_RDWR | O_SYNC)) != -1) &&
      ((ext_mem_fd = open("/dev/alfa_ext_mem", O_RDWR | O_SYNC)) != -1)) {
    pointcloud_ptr_address = configuration.pointcloud_id;

    this->pointcloud.ptr =
        (std::uint64_t *)mmap(0x0, POINTCLOUD_BASE_PER_ID, PROT_READ | PROT_WRITE, MAP_SHARED,
                              mem_fd, pointcloud_ptr_address);

    if (this->pointcloud.ptr == MAP_FAILED) {
      return_value = 1;
      cout << "Failed to map memory" << endl;
    } else {
      //  Create parameter's callbacks
      this->parameters_callback_handle = this->add_on_set_parameters_callback(
          std::bind(&AlfaNode::parameters_callback, this, std::placeholders::_1));
      this->pointcloud.size = 0;

      // Get the physical address of the pointcloud buffer
      unsigned int buffer_id = configuration.pointcloud_id;
      if (ioctl(mem_fd, ALFA_MEM_IOC_GET_PHYS_ADDR, &buffer_id) == -1) {
        perror("Failed to get physical address");
        close(mem_fd);
        return_value = 1;
      }
      unit_write_register(UNIT_PARAMETERS_MEMMU, buffer_id);

      // Get the physical address of the external memory buffer
      if (ioctl(ext_mem_fd, ALFA_EXT_MEM_GET_PHYS_ADDR, &buffer_id) == -1) {
        perror("Failed to get physical address");
        close(ext_mem_fd);
        return_value = 1;
      }

      // Map external memory buffer
      ext_mem = (std::uint64_t *)mmap(0x0, ALFA_EXT_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                                      ext_mem_fd, 0);

      if (ext_mem == MAP_FAILED) {
        return_value = 1;
        cout << "Failed to map external memory" << endl;
      } else {
        // Set the external memory pointer
        unit_write_register(UNIT_PARAMETERS_EXMU, buffer_id);
      }
    }
  } else
    return_value = 1;

#ifdef ALFA_VERBOSE
  if (return_value)
    verbose_fail("constructor", "mem pointers");
  else
    verbose_ok("constructor", "mem pointers");
#endif
  return return_value;
}

void AlfaNode::store_pointcloud(int type, pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (pointcloud == nullptr) pointcloud = this->input_pointcloud;
  switch (type) {
    case LOAD_STORE_CARTESIAN:
      store_pointcloud_cartesian(pointcloud);
      break;
    case LOAD_STORE_SPHERICAL:
      // store_pointcloud_spherical(pointcloud);
      break;

    default:
      store_pointcloud_cartesian(pointcloud);
      break;
  }
}

void AlfaNode::store_pointcloud_cartesian(pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (configuration.hardware_support.hardware_extension) {
    // Pointer to the memory where point cloud data will be stored
    std::uint64_t *mem = this->pointcloud.ptr;

    // Calculate the size of the point cloud
    const size_t num_points = pointcloud->size();

    // Reserve memory for the point cloud data
    std::vector<std::int16_t> a16_points(num_points * 4);  // Assuming 4 elements per point

    // Convert and pack points efficiently
    for (size_t i = 0; i < num_points; ++i) {
      const AlfaPoint &point = (*pointcloud)[i];
      std::int16_t *a16_point = &a16_points[i * 4];

      // Perform all conversions at once to minimize cache misses
      a16_point[0] = static_cast<std::int16_t>(std::round(point.x * FIXED_POINT_MULTIPLIER));
      a16_point[1] = static_cast<std::int16_t>(std::round(point.y * FIXED_POINT_MULTIPLIER));
      a16_point[2] = static_cast<std::int16_t>(std::round(point.z * FIXED_POINT_MULTIPLIER));
      a16_point[3] = static_cast<std::int16_t>(point.custom_field);
    }

    // Copy the packed data to the memory
    memcpy(mem, a16_points.data(), sizeof(std::int16_t) * 4 * num_points);

    // Update the point cloud size
    unit_write_register(UNIT_SIGNALS_SOFTWARE_POINTCLOUD_SIZE, num_points);

    // Synchronize memory with hardware
    __sync_synchronize();

    unsigned int buffer_id = configuration.pointcloud_id;
    if (ioctl(mem_fd, ALFA_MEM_IOC_FLUSH_CACHE, &buffer_id) == -1) {
      perror("Failed to sync memory with hardware");
      exit(EXIT_FAILURE);
    }

#ifdef ALFA_VERBOSE
    verbose_info("store_pointcloud_cartesian", "stored " + std::to_string(num_points) + " points");
    verbose_info("store_pointcloud_cartesian", "last point");
    verbose_info("store_pointcloud_cartesian",
                 "fixed point multiplier: " + std::to_string(FIXED_POINT_MULTIPLIER));
    verbose_info("store_pointcloud_cartesian",
                 "x: " + std::to_string(a16_points[(num_points - 1) * 4]));
    verbose_info("store_pointcloud_cartesian",
                 "y: " + std::to_string(a16_points[(num_points - 1) * 4 + 1]));
    verbose_info("store_pointcloud_cartesian",
                 "z: " + std::to_string(a16_points[(num_points - 1) * 4 + 2]));
#endif
  } else
    verbose_not_defined("store_pointcloud_cartesian");
}

void AlfaNode::load_pointcloud(int type, pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (pointcloud == nullptr) pointcloud = this->output_pointcloud;
  switch (type) {
    case LOAD_STORE_CARTESIAN:
      load_pointcloud_cartesian(pointcloud);
      break;
    case LOAD_STORE_SPHERICAL:
      // load_pointcloud_spherical(pointcloud);
      break;

    default:
      load_pointcloud_cartesian(pointcloud);
      break;
  }
}
#pragma GCC optimize("prefetch-loop-arrays")
void AlfaNode::load_pointcloud_cartesian(pcl::PointCloud<AlfaPoint>::Ptr pointcloud) {
  if (configuration.hardware_support.hardware_extension) {
    auto custom_type = this->configuration.custom_field_conversion_type;

    // Define constants
    const std::uint32_t POINT_STRIDE = 4;  // Assuming 4 elements per point

    // Calculate the number of points
    const std::uint32_t num_points = unit_read_register(UNIT_SIGNALS_SOFTWARE_POINTCLOUD_SIZE);

    // Reserve memory for the point cloud to avoid reallocations
    pointcloud->reserve(num_points);

    // Pointer to the custom data (skip the first 3 elements)
    std::int16_t *custom_ptr = (std::int16_t *)this->pointcloud.ptr + 3;

    // Index variables
    std::uint32_t i = 0;

    // Clear the point cloud
    pointcloud->clear();

    switch (custom_type) {
      case CUSTOM_FIELD_FILTER:
        for (const auto &point : *input_pointcloud) {
          if (custom_ptr[i * POINT_STRIDE] == FILTER_VALUE) {
            pointcloud->emplace_back(point);  // Use emplace_back for efficiency
          }
          i++;
        }
        break;

      default:
        for (const auto &point : *input_pointcloud) {
          pointcloud->emplace_back(point);  // Use emplace_back for efficiency
        }
        break;
    }

#ifdef ALFA_VERBOSE
    verbose_info("load_pointcloud_cartesian", "loaded " + std::to_string(num_points) + " points");
    verbose_info("load_pointcloud_cartesian", "last point");
    verbose_info("load_pointcloud_cartesian",
                 "fixed point multiplier: " + std::to_string(FIXED_POINT_MULTIPLIER));
    if (!pointcloud->empty()) {
      const AlfaPoint &lastPoint = pointcloud->back();
      verbose_info("load_pointcloud_cartesian", "x: " + std::to_string(lastPoint.x));
      verbose_info("load_pointcloud_cartesian", "y: " + std::to_string(lastPoint.y));
      verbose_info("load_pointcloud_cartesian", "z: " + std::to_string(lastPoint.z));
    }
#endif
  } else
    verbose_not_defined("load_pointcloud_cartesian");
}

// Calls the IOCTL to write a register from the hardware
void AlfaNode::unit_write_register(unsigned int offset, unsigned int value) {
  if (configuration.hardware_support.hardware_extension) {
    struct alfa_ext_ioctl_data data;
    data.offset = offset;
    data.value = value;
    data.extension_id = configuration.extension_id;

    if (ioctl(ext_fd, ALFA_EXT_IOC_WRITE_REGISTER, &data) == -1) {
      perror("ioctl write_register");
      exit(EXIT_FAILURE);
    }
  } else
    verbose_not_defined("unit_write_register");
}

// Calls the IOCTL to read a register from the hardware
unsigned int AlfaNode::unit_read_register(unsigned int offset) {
  if (configuration.hardware_support.hardware_extension) {
    struct alfa_ext_ioctl_data data;
    data.offset = offset;
    data.extension_id = configuration.extension_id;

    if (ioctl(ext_fd, ALFA_EXT_IOC_READ_REGISTER, &data) == -1) {
      perror("ioctl read_register");
      exit(EXIT_FAILURE);
    }

    return data.value;
  } else
    verbose_not_defined("unit_write_register");
  return 0;
}

// Calls the IOCTL to waits for a value in a register from the hardware
void AlfaNode::unit_wait_for_value(unsigned int offset, unsigned int value) {
  if (configuration.hardware_support.hardware_extension) {
    struct alfa_ext_ioctl_data data;
    data.offset = offset;
    data.value = value;
    data.extension_id = configuration.extension_id;

    if (ioctl(ext_fd, ALFA_EXT_IOC_WAIT_FOR_VALUE, &data) == -1) {
      if (errno == ETIMEDOUT)
        timeout_counter++;
      else {
        perror("ioctl wait_for_value");
        exit(EXIT_FAILURE);
      }
    }

  } else
    verbose_not_defined("unit_wait_for_value");
}

void AlfaNode::read_ext_memory(uint32_t offset, size_t size, void *buffer) {
  if (configuration.hardware_support.hardware_extension) {
    if (size > ALFA_EXT_MEM_SIZE) {
      verbose_fail("read_ext_memory", "Size is bigger than the external memory");
      return;
    }

    if (offset + size > ALFA_EXT_MEM_SIZE) {
      verbose_fail("read_ext_memory", "Offset is bigger than the external memory");
      return;
    }

    std::memcpy(buffer, ext_mem + offset, size);
  } else {
    verbose_not_defined("read_ext_memory");
  }
}

float AlfaNode::get_debug_point(std::uint16_t number) {
  if (number <= configuration.number_of_debug_points)
    if (configuration.hardware_support.hardware_extension)
      return static_cast<float>(unit_read_register(UNIT_DEBUG_POINT_0 + number * 4));
    else
      return debug_points_message[number].metric;
  else {
    verbose_not_defined("get_debug_point");
    return 1;
  }
}

void AlfaNode::set_debug_point(std::uint16_t number, float value, string tag = "") {
  if (number <= configuration.number_of_debug_points &&
      !configuration.hardware_support.hardware_extension) {
    debug_points_message[number].metric = value;
    if (tag != "") debug_points_message[number].metric_name = tag;
  } else
    verbose_info("set_debug_point", "Debug point number not defined");
}