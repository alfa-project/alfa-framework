# ALFA Drivers

This directory provides the Linux kernel drivers used by the ALFA-Node when hardware acceleration is enabled in the ALFA Node. These drivers are not intended to be accessed directly by ALFA-Extensions. All interaction is managed internally by the ALFA-Node.

## Overview

The ALFA drivers serve as the interface between user-space software, memory buffers, and hardware accelerators. They enable efficient communication and data transfer for and from the FPGA.

### Drivers

- **`alfa_ext`**  
  Provides access to memory-mapped hardware registers in the ALFA-Unit.  
  - Supports register read and write operations  
  - Allows waiting for a register to reach a target value with timeout  
  - Ensures memory access ordering through memory barriers  
  - Includes a watchdog for detecting delayed or missing hardware responses

- **`alfa_ext_mem`**  
  Manages a DMA buffer shared between software and hardware ALFA-Extensions.  
  - Allocates a contiguous memory region at runtime  
  - Exposes physical address for hardware access  
  - Maps the buffer into user space for software-based Extensions

- **`alfa_mem`**  
  Provides memory management for point cloud frames.  
  - Allocates multiple independent DMA buffers  
  - Buffers are mapped into user space for software access  
  - Designed to support parallel access to multiple frames for applications like multi-LiDAR fusion or motion estimation