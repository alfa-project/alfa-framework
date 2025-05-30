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

module tb_alib_circular_fifo;

  // Parameters
  parameter DEPTH = 16;
  parameter WIDTH = 8;

  // Inputs
  reg              clk;
  reg              rst;
  reg  [WIDTH-1:0] data_in;
  reg              wr_en;
  reg              rd_en;

  // Outputs
  wire [WIDTH-1:0] data_out;
  wire             full;
  wire             empty;

  // Instantiate the FIFO module
  alib_circular_fifo #(
      .DEPTH(DEPTH),
      .WIDTH(WIDTH)
  ) uut (
      .clk(clk),
      .rst(rst),
      .data_in(data_in),
      .wr_en(wr_en),
      .rd_en(rd_en),
      .data_out(data_out),
      .full(full),
      .empty(empty)
  );

  // Clock generation
  initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 100 MHz clock
  end

  // Test sequence
  initial begin
    // Initialize inputs
    rst = 1;
    data_in = 0;
    wr_en = 0;
    rd_en = 0;

    // Apply reset
    #10 rst = 0;
    #10 rst = 1;

    // Wait for reset to complete
    #20;

    // Write data to FIFO until full
    $display("Writing to FIFO...");
    repeat (DEPTH) begin
      @(posedge clk);
      data_in = $random % 256;  // Random data
      wr_en   = 1;
      #10 wr_en = 0;
    end

    // Wait a few cycles
    #20;

    // Read data from FIFO until empty
    $display("Reading from FIFO...");
    repeat (DEPTH) begin
      @(posedge clk);
      rd_en = 1;
      #10 rd_en = 0;
    end

    // Wait a few cycles
    #20;

    // Finish simulation
    $stop;
  end
endmodule
