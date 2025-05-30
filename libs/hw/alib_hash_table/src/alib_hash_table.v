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

module alib_hash_table #(
    parameter SIZE_PARAM = 2,  // Default size: 32KB (can be 0 -> 8KB, 1 -> 16KB, 2 -> 32KB, 3 -> 64KB)
    parameter DATA_WIDTH = 32,  // Width of the input data (default: 32 bits)
    parameter ADDR_WIDTH = 16  // Width of the output data (address/position) stored in the table
) (
    input wire clk,
    input wire reset,
    input wire [DATA_WIDTH-1:0] input_data,  // Generic input data for hash
    input wire [ADDR_WIDTH-1:0] write_data,   // Data to write into the hash table (could be an address, index, etc.)
    input wire write_enable,  // Control signal to enable/disable writing to the hash table
    output reg [ADDR_WIDTH-1:0] read_data  // Output from the hash table (retrieved data)
);

  // Local parameters to define the hash table size based on SIZE_PARAM
  localparam SIZE_8KB = 13;  // 2^13 entries for 8KB
  localparam SIZE_16KB = 14;  // 2^14 entries for 16KB
  localparam SIZE_32KB = 15;  // 2^15 entries for 32KB
  localparam SIZE_64KB = 16;  // 2^16 entries for 64KB

  // Determine the number of hash bits (table size) based on SIZE_PARAM
  localparam HASH_BITS = (SIZE_PARAM == 0) ? SIZE_8KB  :
                       (SIZE_PARAM == 1) ? SIZE_16KB :
                       (SIZE_PARAM == 2) ? SIZE_32KB : SIZE_64KB;

  // Hash table memory: dynamically sized based on HASH_BITS
  reg [ADDR_WIDTH-1:0] hash_table_mem[0:(2**HASH_BITS)-1];  // Hash table memory

  // Register to hold the hash value
  reg [ HASH_BITS-1:0] hash_value;

  // Simple hash function (multiply and shift) to generate hash index
  always @(*) begin
    // Hash function for the generic input data
    hash_value = (input_data * 2654435761) >> (32 - HASH_BITS);  // Simple hash based on data width
  end

  // Hash table read/write logic
  always @(posedge clk) begin
    if (reset) begin
      // On reset, clear the output
      read_data <= {ADDR_WIDTH{1'b0}};
    end else begin
      // Read from the hash table (previously stored data)
      read_data <= hash_table_mem[hash_value];

      // Write new data into the hash table only if write_enable is high
      if (write_enable) begin
        hash_table_mem[hash_value] <= write_data;
      end
    end
  end

endmodule
