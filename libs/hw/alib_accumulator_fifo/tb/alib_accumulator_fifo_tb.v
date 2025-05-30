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

module tb_alib_accumulator_fifo;
  // Parameters
  parameter DEPTH = 16;
  parameter WIDTH_INPUT = 16;
  parameter WIDTH_OUTPUT = 8;

  // Inputs
  reg                              i_clk;
  reg                              i_rst;
  reg     [       WIDTH_INPUT-1:0] i_code;
  reg     [ $clog2(WIDTH_INPUT):0] i_code_len;
  reg                              i_wr_enable;
  reg                              i_rd_en;

  // Outputs
  wire                             o_full;
  wire                             o_empty;
  wire    [$clog2(WIDTH_OUTPUT):0] o_bits_left;
  wire    [      WIDTH_OUTPUT-1:0] o_data_out;

  integer                          i;

  // Instantiate the FIFO module
  alib_accumulator_fifo #(
      .DEPTH(DEPTH),
      .WIDTH_INPUT(WIDTH_INPUT),
      .WIDTH_OUTPUT(WIDTH_OUTPUT)
  ) uut (
      .i_clk(i_clk),
      .i_rst(i_rst),
      .i_wr_data(i_code),
      .i_wr_data_len(i_code_len),
      .i_wr_en(i_wr_enable),
      .o_full(o_full),
      .o_empty(o_empty),
      .o_bits_left(o_bits_left),
      .i_rd_en(i_rd_en),
      .i_rd_data_len(WIDTH_OUTPUT),
      .o_rd_data(o_data_out)
  );

  // Clock generation
  initial begin
    i_clk = 1;
    forever #5 i_clk = ~i_clk;  // 100 MHz clock
  end

  // Write Logic
  initial begin
    // Initialize inputs
    i_rst = 0;
    i_code = 0;
    i_code_len = 0;
    i_wr_enable = 0;

    // Apply reset
    #20 i_rst = 1;

    // Write 20 codes to the FIFO
    for (i = 0; i < 50; i = i + 1) begin
      // Wait if FIFO is full
      if (o_full) begin
        $display("FIFO full at time %t. Waiting to write...", $time);
        wait (!o_full);
      end
      #10;
      // Write to FIFO
      i_code = $random % (1 << WIDTH_INPUT);  // Random input code
      i_code_len = ($random % WIDTH_INPUT) + 1;  // Random code length
      i_wr_enable = 1;
      $display("Writing code: %b with length %0d at time %t", i_code, i_code_len, $time);

      // Wait one clock cycle to simulate write enable
      #10;
      i_wr_enable = 0;
    end
  end

  // Read Logic
  initial begin
    // Initialize read enable
    i_rd_en = 0;

    // Wait for the reset and some initial writes
    #30;

    // Read from FIFO until empty
    forever begin
      if (!o_empty) begin
        i_rd_en = 1;  // Enable read
        #10;  // Wait for one clock cycle
        $display("Read data: %b at time %t", o_data_out, $time);
        i_rd_en = 0;  // Disable read
        #10;  // Wait for one clock cycle
      end else begin
        // Wait if FIFO is empty
        $display("FIFO empty at time %t. Waiting to read...", $time);
        wait (!o_empty);
      end
    end
  end

  // Finish simulation
  initial begin
    #1000;  // Run simulation for a limited time
    $stop;
  end
endmodule

