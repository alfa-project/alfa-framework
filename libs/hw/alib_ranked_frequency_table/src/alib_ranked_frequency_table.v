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

module alib_ranked_frequency_table #(
    parameter COUNTER_BITS = 16,  // Parameter to define the width of the counter
    parameter NUMBER_OF_PARALLEL_INPUTS = 8  // Number of parallel inputs, default is 8
) (
    input i_clk,
    input i_rst,  // Active-low, synchronous reset
    input [(8*NUMBER_OF_PARALLEL_INPUTS)-1:0] i_char,  // Concatenated 8-bit input characters
    input [NUMBER_OF_PARALLEL_INPUTS-1:0] i_valid,  // Enable signals for inputs
    input [7:0] i_query_char,  // 8-bit character whose rank is being queried
    input i_start_rank_calc,  // Signal to start rank calculation
    output reg o_rank_done,  // Signal to indicate rank calculation is done
    output reg o_ready,  // Signal to indicate module is ready for input
    output reg [7:0] o_query_rank  // Output for the rank of the query character
);

  // 3D frequency array
  reg [COUNTER_BITS-1:0] frequency[NUMBER_OF_PARALLEL_INPUTS-1:0][255:0];

  reg [7:0] rank_table[255:0];
  reg [COUNTER_BITS-1:0] sum_frequency[255:0];
  reg [COUNTER_BITS-1:0] sum_frequency_extra[255:0];

  reg [7:0] comparison_char;
  reg [7:0] increment_char;

  reg sum_calc_done;

  reg [COUNTER_BITS-1:0] sum_temp;

  reg [8:0] reset_index;  // Reset progress
  reg internal_start_rank;

  wire [7:0] selector_char[NUMBER_OF_PARALLEL_INPUTS-1:0];
  wire [NUMBER_OF_PARALLEL_INPUTS-1:0] overflow_char;
  wire start_rank_calc;

  // Assign input characters to selector_char
  genvar i;
  integer j, k;
  generate
    for (i = 0; i < NUMBER_OF_PARALLEL_INPUTS; i = i + 1) begin : char_selector
      assign selector_char[i] = i_char[(i*8)+:8];
    end
  endgenerate

  // Check for overflow
  generate
    for (i = 0; i < NUMBER_OF_PARALLEL_INPUTS; i = i + 1) begin : overflow_logic
      assign overflow_char[i] = frequency[i][selector_char[i]] == {COUNTER_BITS{1'b1}};
    end
  endgenerate

  assign start_rank_calc = (internal_start_rank || i_start_rank_calc) && o_ready && ~sum_calc_done;

  // Sequential logic
  always @(posedge i_clk) begin  // Reset logic
    if (!i_rst) begin
      reset_index <= 8'd0;
      o_ready <= 1'b0;
      comparison_char <= 8'd0;
      increment_char <= 8'd0;
      sum_calc_done <= 1'b0;
      o_rank_done <= 1'b0;
      sum_temp <= 0;
    end else if (~o_ready) begin  // Reset arrays

      for (j = 0; j < NUMBER_OF_PARALLEL_INPUTS; j = j + 1) begin
        frequency[j][reset_index] <= {COUNTER_BITS{1'b0}};
      end

      rank_table[reset_index] <= 8'd0;
      sum_frequency[reset_index] <= 0;
      sum_frequency_extra[reset_index] <= 0;

      if (reset_index < 255) reset_index <= reset_index + 1;
      else o_ready <= 1'b1;
    end else if (start_rank_calc) begin

      if (comparison_char == 255) begin
        sum_calc_done <= 1;
        internal_start_rank <= 0;
      end else begin
        comparison_char <= comparison_char + 1;
        internal_start_rank <= 1;
      end

      sum_temp = 0;
      for (j = 0; j < NUMBER_OF_PARALLEL_INPUTS; j = j + 1) begin
        sum_temp = sum_temp + frequency[j][comparison_char];
      end

      sum_frequency[comparison_char] <= sum_temp;
      sum_frequency_extra[comparison_char] <= sum_temp;

    end else if (sum_calc_done && ~o_rank_done) begin

      // Sequential comparison for rank calculation
      if (increment_char <= 8'd255) begin
        for (k = 0; k <= 255; k = k + 1) begin

          if(sum_frequency[increment_char] > sum_frequency_extra[k] || (sum_frequency[increment_char] == sum_frequency_extra[k] && increment_char< k )) begin
            rank_table[k] <= rank_table[k] + 1;
          end

        end
      end
      if (increment_char < 8'd255) begin
        increment_char <= increment_char + 1;
      end else begin
        o_rank_done <= 1;
      end
    end else begin  // Frequency increment logic
      for (j = 0; j < NUMBER_OF_PARALLEL_INPUTS; j = j + 1) begin
        if (i_valid[j] && !overflow_char[j]) begin
          frequency[j][selector_char[j]] <= frequency[j][selector_char[j]] + 1;
        end
      end
    end
  end

  // Output rank
  always @(posedge i_clk) begin
    if (o_rank_done) o_query_rank <= rank_table[i_query_char];
    else o_query_rank <= 8'd0;
  end

endmodule
