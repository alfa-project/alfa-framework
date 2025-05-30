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

`timescale 1ns / 1ps

module alib_accumulator_fifo #(
    parameter DEPTH = 16,
    parameter WIDTH_INPUT = 16,
    parameter WIDTH_OUTPUT = 8
) (
    input wire i_clk,
    input wire i_rst,
    // Input code to be written to FIFO
    input wire [WIDTH_INPUT-1:0] i_wr_data,
    input wire [$clog2(
WIDTH_INPUT
):0] i_wr_data_len,
    input wire i_wr_en,
    // Output code read from FIFO
    input wire i_rd_en,
    input wire [$clog2(
WIDTH_OUTPUT
):0] i_rd_data_len,
    output reg [WIDTH_OUTPUT-1:0] o_rd_data,
    // FIFO full and empty indicators
    output wire o_full,
    output wire o_empty,
    // Bits left in the FIFO
    output wire [$clog2(
WIDTH_OUTPUT
):0] o_bits_left

);

  // Local parameters
  localparam BIT_BUFFER_SIZE = WIDTH_INPUT>=WIDTH_OUTPUT? WIDTH_INPUT*DEPTH : WIDTH_OUTPUT*DEPTH;  // Circular bit buffer size

  // Internal FIFO memory
  reg [      0:BIT_BUFFER_SIZE-1] fifo_mem = 0;

  reg [$clog2(BIT_BUFFER_SIZE):0] fifo_write_pointer = 0;
  reg [$clog2(BIT_BUFFER_SIZE):0] fifo_read_pointer = 0;
  reg [$clog2(BIT_BUFFER_SIZE):0] fifo_count_bits = 0;

  // Full and empty indicators
  assign o_full  = (fifo_count_bits > (BIT_BUFFER_SIZE - WIDTH_INPUT));
  assign o_empty = (fifo_count_bits < WIDTH_OUTPUT);

  // Valid rd and wr transactions
  wire valid_rd, valid_wr;
  assign valid_rd = i_rd_en && !o_empty && (fifo_count_bits >= i_rd_data_len);
  assign valid_wr = i_wr_en && !o_full && ((fifo_count_bits + i_wr_data_len) <= BIT_BUFFER_SIZE);

  // Additional internal signals
  integer i, j;

  // Bits left logic
  assign o_bits_left = fifo_count_bits;

  // Update FIFO count bits
  always @(posedge i_clk) begin
    if (!i_rst) begin
      fifo_count_bits <= 0;
    end else begin
      case ({
        (valid_wr), (valid_rd)
      })
        // Write only
        2'b10:   fifo_count_bits <= fifo_count_bits + i_wr_data_len;
        // Read only
        2'b01:   fifo_count_bits <= fifo_count_bits - i_rd_data_len;
        // Write and read simultaneously
        2'b11:   fifo_count_bits <= fifo_count_bits - i_rd_data_len + i_wr_data_len;
        // No operation
        default: fifo_count_bits <= fifo_count_bits;
      endcase
    end
  end

  // FIFO read logic
  always @(posedge i_clk) begin
    if (!i_rst) begin
      o_rd_data <= 0;
      fifo_read_pointer <= 0;
    end else if (valid_rd) begin
      for (i = 0; i < i_rd_data_len; i = i + 1) begin
        o_rd_data[i_rd_data_len-1-i] <= fifo_mem[(fifo_read_pointer + i) & (BIT_BUFFER_SIZE - 1)]; // Wrap around
      end
      fifo_read_pointer <= (fifo_read_pointer + i_rd_data_len) & (BIT_BUFFER_SIZE - 1);
    end
  end

  // FIFO write logic
  always @(posedge i_clk) begin
    if (!i_rst) begin
      fifo_write_pointer <= 0;
    end else if (valid_wr) begin
      for (j = 0; j < i_wr_data_len; j = j + 1) begin
        fifo_mem[(fifo_write_pointer + j) & (BIT_BUFFER_SIZE - 1)] <= i_wr_data[i_wr_data_len-1-j]; // Wrap around
      end
      fifo_write_pointer <= (fifo_write_pointer + i_wr_data_len) & (BIT_BUFFER_SIZE - 1);
    end
  end

endmodule
