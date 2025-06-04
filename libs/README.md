# ALFA Libraries

This directory provides ready-to-use ALFA libraries. They can be used inside your extensions to reduce the engineering effort of developing new ALFA-compatible modules.

---

## Available Libraries (Summary)

| Library              | Description                                                                                      |
|----------------------|--------------------------------------------------------------------------------------------------|
| [Metrics](#metrics-library)                 | Computes segmentation performance metrics using SemanticKITTI-based ground truth.         |
| [RAM Instantiation](#ram-instantiation-library)     | Helps instantiate BRAM and URAM modules in hardware.                                     |
| [Circular FIFO](#circular-fifo-library)            | Implements a circular FIFO using RAM resources.                                          |
| [Accumulator FIFO](#accumulator-fifo-library)        | Bit-level FIFO with dynamic-length reads and writes for compression pipelines.            |
| [Ranked Frequency Table](#ranked-frequency-table-library) | Tracks symbol frequencies in parallel and calculates rank ordering by frequency.          |
| [Compression](#compression-library)                | Provides RLE, LZ4, LZ77, and Huffman encoding/decoding utilities for binary streams.     |
| [Octree](#octree-library)                  | Software implementation of a DFS-based octree optimized for LiDAR point cloud handling.  |

---

## Libraries Details

### Metrics Library

- **Source files**: [Software](https://github.com/alfa-project/alfa-framework/tree/main/libs/sw/alib_metrics)
- **Purpose**: Provides evaluation metrics for segmentation tasks by comparing predicted and ground truth labels.
- **Based on**: [SemanticKITTI](http://semantic-kitti.org/)
- **Computed metrics**:
  - Accuracy
  - Precision
  - Recall
  - F1-Score
  - IoU
  - mIoU

---

### RAM Instantiation Library

- **Source files**: [Hardware](https://github.com/alfa-project/alfa-framework/tree/main/libs/hw/alib_ram)
- **Purpose**: Provides reusable modules for memory instantiation in FPGA-based extensions.
- **Supported types**:
  - Block RAM (BRAM)
  - Ultra RAM (URAM)

---

### Circular FIFO Library

- **Source files**: [Hardware](https://github.com/alfa-project/alfa-framework/tree/main/libs/hw/alib_ram)
- **Purpose**: Implements a circular FIFO queue using internal RAMs. Useful for buffering data streams in pipelined hardware designs.

---

### Accumulator FIFO Library

- **Source files**: [Hardware](https://github.com/alfa-project/alfa-framework/tree/main/libs/hw/alib_accumulator_fifo)
- **Purpose**: Implements a bit-accurate FIFO buffer with dynamic write and read lengths. Designed to handle variable-length codes such as those used in entropy compression (e.g., Huffman, LZ).
- **Design features**:
  - Accepts input writes with `WIDTH_INPUT`-bit width and `i_wr_data_len` bit length
  - Supports output reads with `WIDTH_OUTPUT`-bit width and `i_rd_data_len` bit length
  - Bit-level circular buffer implementation with wrapping logic
  - Full/empty indicators and available bit count output (`o_bits_left`)
  - Simultaneous write/read handling with internal pointer management
- **Configurable parameters**:
  - `DEPTH` – Total number of storage elements (default: 16)
  - `WIDTH_INPUT` – Width of the write interface in bits (e.g., 16 bits)
  - `WIDTH_OUTPUT` – Width of the read interface in bits (e.g., 8 bits)
- **Interface**:
  - `i_wr_data`, `i_wr_data_len`, `i_wr_en`: Write input signals
  - `i_rd_en`, `i_rd_data_len`: Read control signals
  - `o_rd_data`: Output read data
  - `o_full`, `o_empty`: Status signals
  - `o_bits_left`: Tracks how many valid bits are in the FIFO
- **Use case**: Ideal for bit-accumulation and streaming data in compression/decompression engines (e.g., output buffer for variable-length encoded data).

### Ranked Frequency Table Library

- **Source files**: [Hardware](https://github.com/alfa-project/alfa-framework/tree/main/libs/hw/alib_ranked_frequency_table)
- **Purpose**: Implements a hardware module that tracks the frequency of incoming characters in parallel and calculates their rank based on occurrence count.
- **Use case**: Useful as a preprocessing step in hardware compression pipelines (e.g., Huffman or ranked encoding), especially when sorting symbols by frequency is needed.
- **Design features**:
  - Accepts up to `NUMBER_OF_PARALLEL_INPUTS` (default: 8) 8-bit inputs per cycle
  - Maintains per-input frequency counters for all 256 possible symbols
  - Calculates ranks using total frequency and tie-breaking rules
  - Outputs the rank of a query character after rank calculation is triggered
- **Configurable parameters**:
  - `COUNTER_BITS` – Bitwidth of frequency counters (e.g., 16 bits)
  - `NUMBER_OF_PARALLEL_INPUTS` – Degree of input parallelism (default: 8)
- **Interface**:
  - `i_char`: Concatenated stream of input characters
  - `i_valid`: Valid bits for each input
  - `i_query_char`: Character whose rank is being queried
  - `i_start_rank_calc`: Triggers rank calculation
  - `o_query_rank`: Output rank value for `i_query_char`
  - `o_rank_done`: Indicates that the rank is available
  - `o_ready`: Indicates module is ready to begin processing

---

### Compression Library

- **Source files**: [Software](https://github.com/alfa-project/alfa-framework/tree/main/libs/sw/alib_compression)
- **Purpose**: Provides software utilities to compress and decompress binary bitstreams, especially targeting octree encodings or point cloud data.
- **Supported algorithms**:
  - **Run-Length Encoding (RLE)**: `alib_rle_encode`, `alib_rle_decode`
  - **LZ4**: `alib_lz4_encode`, `alib_lz4_decode`
  - **LZ77**: `alib_lz77_encode`, `alib_lz77_decode`
  - **Huffman**: `alib_huffman_encode`, `alib_huffman_decode`
  - **Simplified Huffman**: `alib_huffman_s_encode`, `alib_huffman_s_decode`
- **Additional tools**:
  - Frequency analysis (`print_frequencies`)
  - Binary vector visualization (`print_vector_binary`)
  - I/O utilities (`write_binary_file`, `output_compression_code`, `output_decoded_compression_code`)
  - Converters for `std::vector<AlfaPoint>` to/from encoded binary
- **Integration**: Can be used in compression-focused extensions such as octree bitstream encoders.

---

### Octree Library

- **Purpose**: Provides both software and hardware implementations of an octree encoder optimized for LiDAR point cloud compression and reconstruction using DFS traversal.
- **Software Source files**: [Software](https://github.com/alfa-project/alfa-framework/tree/main/libs/sw/alib_octree)
  - **Key features**:
    - DFS encoding/decoding of octree occupation codes
    - Parallel traversal support
    - Fixed bounding box (`AlfaBB`) and user-defined resolution per axis
    - Native support for `AlfaPoint`-based `pcl::PointCloud`s
  - **Main methods**:
    - `insert_point()` and `insert_pointcloud()` – populate the octree
    - `get_occupation_code_DFS()` – generates DFS-encoded octree bitstream
    - `convert_to_pointcloud()` – reconstructs original point cloud
    - `init_octree_from_occupation_code_DFS()` – builds tree from DFS bitstream
  - **Multithreading**: Enabled via `convert_to_pointcloud_multi_thread` and thread affinity handling
  - **Internal structure**:
    - Nodes are stored as `AlfaOctreeNode` with 8 children and bounding box
    - Bounding box calculations and point filtering done using `AlfaBB`

- **Hardware Source files**: [Hardware](https://github.com/alfa-project/alfa-framework/tree/main/libs/hw/alib_octree)
  - **Configurable parameters**:
    - `BB_MIN_X`, `BB_MIN_Y`, `BB_MIN_Z` – minimum bounding box coordinates
    - `BB_MAX_X`, `BB_MAX_Y`, `BB_MAX_Z` – maximum bounding box coordinates
    - `MAX_DEPTH` – maximum tree depth (resolution control)
    - `NUMBER_NODES` – maximum nodes supported in memory
    - `NUMBER_TREES` – number of octree encoders working in parallel
    - `SIZE_ID` – bitwidth of tree/node identifiers
  - **Design characteristics**:
    - Uses fixed bounding box arithmetic (no floating point logic)
    - Supports parallel encoding of octrees for improved throughput
    - Easily integrable into FPGA pipelines via AXI or custom streaming interfaces


