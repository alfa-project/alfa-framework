# ALFA-Unit

<p align="center">
<img src="docs/figures/overview.png" alt="overview" width="700" />
</p>

ALFA unit is the hardware counterpart of ALFA-Node in the Core. It is responsible to create the base interface between hardware extensions and the remaining components of ALFA framework. This component is composed by four units, all integrated into one hardware IP core:

- The **Memory Management Unit (MemMU)** - responsible for managing ALFA's hardware memory. ALFA take advantage of different memory chips (on-chip RAM and FPGA-based memories, like BRAM) to implement ALFA's memory architecture. This architecture divides the memory into three sections: 1) core memory, which is used mainly for storing point clouds representations and Core registers;  2) operating system memory, to allow a smooth system operation, and 3) extensions memory to provide plenty of space for the deployed algorithms. Regarding point clouds representations, developers can configure MemMU to store data in two representations:  sorted *Spherical-grid* coordinates and unsorted point-by-point *Cartesian coordinates*.
- The **Extension Management Unit (ExMU)** - the primary connection between hardware algorithms implementation and the Core. It offers a dedicated cache and point cloud interface, allowing developers to change how the point cloud data flows into their algorithms. 
- The **Monitor Unit (MonU)** - responsible for marking and collecting all the information from the hardware core units and extensions, transferring it to the software layer to be later shared with the GUI tool.
- The **Sensor Interface Unit (SIU)** - the hardware-accelerated ALFA-Unit that decodes and reconstructs LiDAR data from sensors with Ethernet interfaces. It allows interfacing the most advanced LiDAR sensors in the market with FPGA-enable embedded platforms, reducing latency between the input data and the algorithms that rely on it. Due to SIU's parallelism, it can interface multiple sensors simultaneously while maintaining the maximum sensor throughput. 

# Interfaces and Mapped Registers

The ALFA-Unit IP has 8 interfaces:

**SYSTEM** - System signals composed by the a clock (system_clk) and a negedge reset (system_rst).

**S_AXI** - Slave lite AXI interface responsible to handle internal registers and signals.

**M_AXI** - Master full AXI interface responsible to handle memory transactions (BRAM, DRAM, etc).

**Spherical_Representation** - Interface that outputs the point cloud points in spherical representation (θ,φ,r):

- *pointAngleH* - Point's azimuthal angle (θ).
- *pointAngleV* - Point's polar angle (φ).
- *pointDistR0* - Point's radial distance.

**Cartesian_Representation** - Interface that outputs the point cloud points in cartesian representation (x,y,z):

- *pointX* - Point's X value.
- *pointY* - Point's Y value.
- *pointZ* - Point's Z value.

**USER_Define** - Interface that allows developers to send configuration values into their hardware extensions.

- *user_define_0* up to *user_define_9* - 10 x 32 bits mapped registers.

**Debug_Points** - Interface that allows developers to receive data from their hardware extensions.

- *debug_point_0* up to *debug_point_19* - 20 x 32 bits mapped registers.

**Extension_Interface** - Dedicated interface to connect an hardware extension. It is composed by 12 signals:

- *writeValid* - The extension has valid labels to write.
- *writeReady* - The node is ready to write new labels.
- *readValid* - The point information on the **Spherical_Representation** / **Cartesian_Representation** interface ouput is valid.
- *readReady* -The extension is ready to read new points.
- *PCSize* - The pointcloud size in points.
- *done_processing* - The extension is done processing.
- *writeCustomField* - New *custom field* by the extension.
- *writeID* - The point ID that the extension wants to write.
- *readID* - The point ID that the extension want to read.
- *readCustomField* - The original *custom field* of the point read.
- *status* - The extension status.
- *enable* - The extension enable.

## Interface signals exemplification

### Read Point

### Write Point

### Start processing

### Done processing

## Mapped Registers (32bit registers)

| Address |          Register          |
|:-------:|:--------------------------:|
|  0x00   |         CU status          |
|  0x04   |        ExMU status         |
|  0x08   |        MemMU status        |
|  0x0C   |        MonU status         |
|  0x10   |         SIU status         |
|  0x14   |         EXT status         |
|  0x18   |        Not defined         |
|  0x1C   |        Not defined         |
|  0x20   | Hardware point cloud ready |
|  0x24   |  Hardware processing done  |
|  0x28   |       Debug Point 0        |
|  0x2C   |       Debug Point 1        |
|  0x30   |       Debug Point 2        |
|  0x34   |       Debug Point 3        |
|  0x38   |       Debug Point 4        |
|  0x3C   |       Debug Point 5        |
|  0x40   |       Debug Point 6        |
|  0x44   |       Debug Point 7        |
|  0x48   |       Debug Point 8        |
|  0x4C   |       Debug Point 9        |
|  0x50   |       Debug Point 10       |
|  0x54   |       Debug Point 11       |
|  0x58   |       Debug Point 12       |
|  0x5C   |       Debug Point 13       |
|  0x60   |       Debug Point 14       |
|  0x64   |       Debug Point 15       |
|  0x68   |       Debug Point 16       |
|  0x6C   |       Debug Point 17       |
|  0x70   |       Debug Point 18       |
|  0x74   |       Debug Point 19       |
|  0x78   |        CU parameter        |
|  0x7C   |      MemMU parameter       |
|  0x80   |       ExMU parameter       |
|  0x84   |       MonU parameter       |
|  0x88   |       SIU parameter        |
|  0x8C   | Software pointcloud ready  |
|  0x90   |  Software processing done  |
|  0x94   |  Software pointcloud size  |
|  0x98   |       User Define 0        |
|  0x9C   |       User Define 1        |
|  0xA0   |       User Define 2        |
|  0xA4   |       User Define 3        |
|  0xA8   |       User Define 4        |
|  0xAC   |       User Define 5        |
|  0xB0   |       User Define 6        |
|  0xB4   |       User Define 7        |
|  0xB8   |       User Define 8        |
|  0xBC   |       User Define 9        |

# Configuration

There are three main configuration sections:

**Memory** - Configure how the Unit will manage the memory access. 9 parameters compose this section:

- *Interface Type* - Memory Interface connection type. Correctly only AXI interface is supported.
- *Memory Type* - Memory type. Both DDR and BRAM are supported.
- *Offset* - Memory offset where the core memory starts.
- *Pointcloud Id* - Specific Pointcloud Id that this Unit IP will manage and handle. ALFA-UNIT support up to 4 different point clouds running simultaneously.
- *Representation Type* - Type of representation stored in the memory. ALFA-UNIT support Spherical Representations (SR) and Cartesian Representations (CR).

**Sensor Interface** - Configure if and how the SIU handles the sensor data. Currently it is not fully supported.

**Extension** - Configure if and how the Extension Management Unit handles the interface between the data and extension. 3 parameters compose this section:

- *ExMU Support* - Selects if ALFA-Unit will deploy hardware extension support. If *No* is selected, the ALFA-Core will just support SIU, software extensions, debug points and user_defines.
- *Number of Debug Points* - Select the number of Debug Points (DP) required.
- *Number of User Defines* - Select the number of User Defines (UD) required.