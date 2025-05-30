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

module alib_points_circular_fifo #(
    parameter DEPTH = 16  // Depth of the FIFO (number of 3D points it can store)
) (
    input  wire        clk,          // Clock signal
    input  wire        rst,          // Active-low reset signal
    input  wire [15:0] point_x_in,   // 16-bit input for x-coordinate
    input  wire [15:0] point_y_in,   // 16-bit input for y-coordinate
    input  wire [15:0] point_z_in,   // 16-bit input for z-coordinate
    input  wire        wr_en,        // Write enable signal
    input  wire        rd_en,        // Read enable signal
    output wire [15:0] point_x_out,  // 16-bit output for x-coordinate
    output wire [15:0] point_y_out,  // 16-bit output for y-coordinate
    output wire [15:0] point_z_out,  // 16-bit output for z-coordinate
    output wire        full,         // FIFO full flag
    output wire        empty         // FIFO empty flag
);

  // Concatenate x, y, and z inputs into a single 48-bit data entry
  // This combines the three 16-bit coordinates into a single word for storage in the FIFO
  wire [47:0] data_in_concat = {point_x_in, point_y_in, point_z_in};

  // 48-bit data output from the FIFO, which will contain concatenated x, y, and z coordinates
  wire [47:0] data_out_concat;

  // Instantiate a single FIFO for the concatenated x, y, and z coordinates
  alib_circular_fifo #(
      .DEPTH(DEPTH),  // Set FIFO depth as defined by the module parameter
      .WIDTH(48)      // Set FIFO data width to 48 bits (for three 16-bit coordinates)
  ) fifo_concat (
      .clk     (clk),              // Connect clock
      .rst     (rst),              // Connect reset
      .data_in (data_in_concat),   // 48-bit concatenated input data (x, y, z)
      .wr_en   (wr_en),            // Connect write enable
      .rd_en   (rd_en),            // Connect read enable
      .data_out(data_out_concat),  // 48-bit concatenated output data (x, y, z)
      .full    (full),             // Connect full flag
      .empty   (empty)             // Connect empty flag
  );

  // Split the concatenated 48-bit output into separate 16-bit x, y, and z outputs
  // point_x_out takes the upper 16 bits of data_out_concat
  // point_y_out takes the middle 16 bits of data_out_concat
  // point_z_out takes the lower 16 bits of data_out_concat
  assign point_x_out = data_out_concat[47:32];
  assign point_y_out = data_out_concat[31:16];
  assign point_z_out = data_out_concat[15:0];

endmodule

