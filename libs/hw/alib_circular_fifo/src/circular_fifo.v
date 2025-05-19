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

module alib_circular_fifo #(
                                                                                                    parameter DEPTH = 16,  // Depth of the FIFO buffer, determines maximum entries it can hold
                                                                                                    parameter WIDTH = 8    // Width of each entry in the FIFO buffer
) (
                                                                                                    input  wire             clk,       // Clock signal
                                                                                                    input  wire             rst,       // Reset signal (active-high)
                                                                                                    input  wire [WIDTH-1:0] data_in,   // Data input to the FIFO buffer
                                                                                                    input  wire             wr_en,     // Write enable signal
                                                                                                    input  wire             rd_en,     // Read enable signal
                                                                                                    output reg  [WIDTH-1:0] data_out,  // Data output from the FIFO buffer
                                                                                                    output wire             full,      // Indicates if the FIFO is full
                                                                                                    output wire             empty      // Indicates if the FIFO is empty
);

  // FIFO buffer memory array, with DEPTH entries of WIDTH-bit width each
  reg [        WIDTH-1:0] fifo  [0:DEPTH-1];

  // Head pointer: points to the location for the next write operation
  reg [$clog2(DEPTH)-1:0] head;

  // Tail pointer: points to the location for the next read operation
  reg [$clog2(DEPTH)-1:0] tail;

  // Counter to track the number of entries in the FIFO buffer
  reg [$clog2(DEPTH)-1:0] count;

  // Full and empty flags based on the count
  assign full  = (count == DEPTH);  // FIFO is full when count reaches DEPTH
  assign empty = (count == 0);  // FIFO is empty when count is zero

  integer i;  // For loop variable (used for reset purposes)

  always @(posedge clk) begin
    if (rst) begin
      // Active-high reset logic to clear FIFO
      head     <= 0;  // Reset head pointer to the start
      tail     <= 0;  // Reset tail pointer to the start
      count    <= 0;  // Reset count to zero (FIFO is empty)
      data_out <= 0;  // Clear data_out
    end else begin
      // Write operation: If wr_en is high and FIFO is not full
      if (wr_en && !full) begin
        fifo[head] <= data_in;     // Write data to current head position
        head <= (head == DEPTH - 1) ? 0 : head + 1; // Increment head, wrap if needed
      end

      // Read operation: If rd_en is high and FIFO is not empty
      if (rd_en && !empty) begin
        data_out <= fifo[tail];  // Read data from current tail position
        tail <= (tail == DEPTH - 1) ? 0 : tail + 1;  // Increment tail, wrap if needed
      end

      // Update count based on read and write operations
      if (wr_en && !full && rd_en && !empty) begin
        // If both read and write occur, count stays the same
        count <= count;
      end else if (wr_en && !full) begin
        // Increment count if writing without reading
        count <= count + 1;
      end else if (rd_en && !empty) begin
        // Decrement count if reading without writing
        count <= count - 1;
      end
    end
  end

endmodule


