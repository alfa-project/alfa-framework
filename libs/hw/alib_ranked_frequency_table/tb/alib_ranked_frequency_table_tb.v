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

module tb_alib_ranked_frequency_table;

  // Parameters
  parameter COUNTER_BITS = 16;
  parameter NUMBER_PARALLEL_INPUTS = 8;

  // Inputs
  reg                                   i_clk;
  reg                                   i_rst;
  reg  [(8*NUMBER_PARALLEL_INPUTS)-1:0] i_char;  // Concatenated 8-bit characters
  reg  [    NUMBER_PARALLEL_INPUTS-1:0] i_valid;  // Concatenated valid signals
  reg  [                           7:0] i_query_char;
  reg                                   i_start_rank_calc;

  // Outputs
  wire                                  o_rank_done;
  wire [                           7:0] o_query_rank;

  // Instantiate the ranked_frequency_table module
  alib_ranked_frequency_table #(
      .COUNTER_BITS(COUNTER_BITS),
      .NUMBER_PARALLEL_INPUTS(NUMBER_PARALLEL_INPUTS)
  ) uut (
      .i_clk(i_clk),
      .i_rst(i_rst),
      .i_char(i_char),
      .i_valid(i_valid),
      .i_query_char(i_query_char),
      .i_start_rank_calc(i_start_rank_calc),
      .o_rank_done(o_rank_done),
      .o_query_rank(o_query_rank)
  );

  // Clock generation
  initial begin
    i_clk = 0;
    forever #5 i_clk = ~i_clk;  // 100 MHz clock
  end

  // Test sequence
  initial begin
    // Initialize inputs
    i_rst = 1;
    i_char = 0;
    i_valid = 0;
    i_query_char = 0;
    i_start_rank_calc = 0;

    // Apply synchronous reset and wait for reset completion
    #10 i_rst = 0;
    #10 i_rst = 1;

    // Wait for array reset cycles (256 cycles)
    repeat (256) @(posedge i_clk);

    // Complex test sequence with different character patterns
    // First phase: Sequential data entry
    $display("Sequential data entry phase");
    #10 i_char = {8'h41, 8'h42, 8'h43, 8'h44, 8'h00, 8'h00, 8'h00, 8'h00};
    i_valid = 8'b11110000;
    #10 i_valid = 0;

    // Second phase: Random data bursts
    $display("Random data bursts phase");
    #10 i_char = {8'h00, 8'h00, 8'h45, 8'h00, 8'h46, 8'h00, 8'h00, 8'h00};
    i_valid = 8'b00101000;
    #10 i_valid = 0;

    #10 i_char = {8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h47, 8'h00, 8'h00};
    i_valid = 8'b00000100;
    #10 i_valid = 0;

    #10 i_char = {8'h00, 8'h00, 8'h00, 8'h48, 8'h00, 8'h00, 8'h00, 8'h49};
    i_valid = 8'b10001000;
    #10 i_valid = 0;

    // Third phase: Multiple simultaneous inputs
    $display("Multiple simultaneous inputs phase");
    #10 i_char = {8'h4A, 8'h00, 8'h4B, 8'h00, 8'h4C, 8'h00, 8'h4D, 8'h00};
    i_valid = 8'b01010101;
    #10 i_valid = 0;

    // Fourth phase: Randomized signal toggling
    $display("Randomized signal toggling phase");
    repeat (10) begin
      @(posedge i_clk);
      i_char  = $random;
      i_valid = $random;
      #10 i_valid = 0;
    end

    // Start rank calculation
    $display("Starting rank calculation...");
    #20 i_start_rank_calc = 1;
    #10 i_start_rank_calc = 0;

    // Wait for rank calculation to complete
    wait (o_rank_done);

    // Query rank for various characters
    $display("Querying rank for different characters");
    #10 i_query_char = 8'h41;  // 'A'
    #10 $display("Rank of 'A': %d", o_query_rank);

    #10 i_query_char = 8'h42;  // 'B'
    #10 $display("Rank of 'B': %d", o_query_rank);

    #10 i_query_char = 8'h47;  // 'G'
    #10 $display("Rank of 'G': %d", o_query_rank);

    #10 i_query_char = 8'h4D;  // 'M'
    #10 $display("Rank of 'M': %d", o_query_rank);

    // Finish simulation
    #50 $stop;
  end

endmodule
