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
    parameter DEPTH = 16,
    parameter WIDTH = 8
) (
    input wire clk,
    input wire rst,
    input wire [WIDTH-1:0] data_in,
    input wire wr_en,
    input wire rd_en,
    output reg [WIDTH-1:0] data_out,
    output wire full,
    output wire empty
);

  reg [WIDTH-1:0] fifo [0:DEPTH-1];
  reg [$clog2(DEPTH)-1:0] head;
  reg [$clog2(DEPTH)-1:0] tail;
  reg [$clog2(DEPTH)-1:0] count;

  assign full  = (count == DEPTH);
  assign empty = (count == 0);
  integer i;

  always @(posedge clk) begin
    if (rst) begin
      if (wr_en && !full) begin
        fifo[head] <= data_in;
        head <= (head + 1) % DEPTH;
        count <= count + 1;
      end
      if (rd_en && !empty) begin
        data_out <= fifo[tail];
        tail <= (tail + 1) % DEPTH;
        count <= count - 1;
      end
    end else begin
      head <= 0;
      tail <= 0;
      count <= 0;
      data_out <= 0;
      for (i = 0; i < DEPTH; i = i + 1) begin
        fifo[i] <= 0;
      end
    end
  end

endmodule

  module points_circular_fifo #(
    parameter DEPTH = 16
) (
    input wire clk,
    input wire rst,
    input wire [15:0] point_h_in,
    input wire [15:0] point_v_in,
    input wire [15:0] point_r_in,
    input wire wr_en,
    input wire rd_en,
    output reg [15:0] point_h_out,
    output reg [15:0] point_v_out,
    output reg [15:0] point_r_out,
    output wire full,
    output wire empty
);

  reg [15:0] fifo_h[0:DEPTH-1];
  reg [15:0] fifo_v[0:DEPTH-1];
  reg [15:0] fifo_r[0:DEPTH-1];
  reg [ 7:0] head;
  reg [ 7:0] tail;
  reg [ 7:0] count;

  assign full  = (count == DEPTH);
  assign empty = (count == 0);
  integer i;

  always @(posedge clk) begin
    if (rst) begin
      if (wr_en && !full) begin
        fifo_h[head] <= point_h_in;
        fifo_v[head] <= point_v_in;
        fifo_r[head] <= point_r_in;
        head <= (head + 1) % DEPTH;
        count <= count + 1;
      end
      if (rd_en && !empty) begin
        point_h_out <= fifo_h[tail];
        point_v_out <= fifo_v[tail];
        point_r_out <= fifo_r[tail];
        tail <= (tail + 1) % DEPTH;
        count <= count - 1;
      end
    end else begin
      head <= 0;
      tail <= 0;
      count <= 0;
      point_h_out <= 0;
      point_v_out <= 0;
      point_r_out <= 0;
      for (i = 0; i < DEPTH; i = i + 1) begin
        fifo_h[i] <= 0;
        fifo_v[i] <= 0;
        fifo_r[i] <= 0;
      end
    end
  end

endmodule

