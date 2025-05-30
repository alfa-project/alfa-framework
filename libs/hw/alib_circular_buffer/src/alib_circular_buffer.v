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

module alib_circular_buffer #(
    parameter DEPTH     = 16,  // Depth of the buffer
    parameter WIDTH     = 8,   // Width of each data entry
    parameter ADDR_TYPE = 1    // 1 = Relative addressing, 0 = Absolute addressing
) (
    input wire clk,  // Clock signal
    input wire rst,  // Active-low reset
    input wire [WIDTH-1:0] data_in,  // Data input
    input wire wr_en,  // Write enable signal
    input wire [$clog2(
DEPTH
)-1:0] read_index,  // Read index
    output reg [WIDTH-1:0] data_out,  // Data output
    output wire full,  // Buffer full flag
    output wire empty,  // Buffer empty flag
    output reg [$clog2(
DEPTH
)-1:0] last_write_index  // Last write index output
);

  // Buffer memory
  reg [        WIDTH-1:0] buffer                                      [0:DEPTH-1];  // Buffer memory
  reg [$clog2(DEPTH)-1:0] head;  // Write pointer
  reg [  $clog2(DEPTH):0] count;  // Count to track number of elements

  // Full and empty flags
  assign full  = (count == DEPTH);
  assign empty = (count == 0);

  // Calculated read position based on ADDR_TYPE
  wire [$clog2(DEPTH)-1:0] read_pos;

  // Generate block for addressing mode
  generate
    if (ADDR_TYPE == 1) begin : relative_addressing
      // Relative addressing: calculate read_pos as offset from head
      assign read_pos = (read_index <= head) ? 
                        (head - read_index) : 
                        (head + DEPTH - read_index) % DEPTH;
    end else begin : absolute_addressing
      // Absolute addressing: read_pos is the direct index
      assign read_pos = read_index;
    end
  endgenerate

  // Write and read operations
  always @(posedge clk or negedge rst) begin
    if (!rst) begin
      // Active-low reset: initialize head, count, data_out, last_write_index, and buffer
      head <= 0;
      count <= 0;
      data_out <= 0;
      last_write_index <= 0;
      integer i;
      for (i = 0; i < DEPTH; i = i + 1) begin
        buffer[i] <= 0;
      end
    end else begin
      // Write operation
      if (wr_en && !full) begin
        buffer[head] <= data_in;  // Write data to buffer at head position
        last_write_index <= head;  // Update last write index to current head position
        head <= (head == DEPTH - 1) ? 0 : head + 1;  // Wrap around head pointer if needed
        count <= count + 1;  // Increment count on write
      end

      // Read operation
      data_out <= buffer[read_pos];  // Read data from calculated read_pos
    end
  end

endmodule


