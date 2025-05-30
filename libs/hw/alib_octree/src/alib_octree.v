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

`define B_N_MODES 5

`define TURN_ALL_ENABLES_OFF \
  enable_parent_update <= 0; \
  enable_occupation_code_update <= 0; \
  enable_child_update <= 0; \

module alib_octree #(
    parameter [31:0] NUMBER_NODES = 100000,
    parameter [31:0] SIZE_ID = $clog2(NUMBER_NODES),
    parameter PARALLEL_LEVEL = 1,
    parameter COMPRESSION = 0,
    parameter signed [15:0] BB_MIN_X = 0,
    parameter signed [15:0] BB_MIN_Y = 0,
    parameter signed [15:0] BB_MIN_Z = 0,
    parameter signed [15:0] BB_MAX_X = 0,
    parameter signed [15:0] BB_MAX_Y = 0,
    parameter signed [15:0] BB_MAX_Z = 0,
    parameter [3:0] MAX_DEPTH = 8
) (
    input wire i_SYSTEM_clk,
    input wire i_SYSTEM_rst,
    input wire i_new_point,
    input wire signed [15:0] i_point_x,
    input wire signed [15:0] i_point_y,
    input wire signed [15:0] i_point_z,
    input wire i_occupation_code_rd,
    input wire i_reset_octree,
    input wire i_start_dfs,
    output wire o_dfs_ready,
    output wire o_dfs_done,
    output wire o_occupation_code_ready,
    output wire o_new_point_ready,
    output wire o_all_nodes_occupied,
    output wire [7:0] o_occupation_code,
    output reg [2:0] o_occupation_code_last_valid_bits,
    output wire o_occupation_code_last_byte
);

  // Parameters and Signals
  localparam integer NUMBER_TREES = PARALLEL_LEVEL == 0 ? 1 : 8;
  localparam integer NUMBER_NODES_PER_OCTREE = NUMBER_NODES / NUMBER_TREES;
  localparam signed [15:0] BB_SIZE_X = (BB_MAX_X - BB_MIN_X) / (2 ** PARALLEL_LEVEL);
  localparam signed [15:0] BB_SIZE_Y = (BB_MAX_Y - BB_MIN_Y) / (2 ** PARALLEL_LEVEL);
  localparam signed [15:0] BB_SIZE_Z = (BB_MAX_Z - BB_MIN_Z) / (2 ** PARALLEL_LEVEL);

  // Tree signals
  reg [$clog2(NUMBER_NODES_PER_OCTREE)-1:0] occupation_code_id[NUMBER_TREES-1:0];
  reg [                               31:0] size_code_written;
  reg root_setup, last_write_done;
  wire [ 7:0] occupation_root_code;  // Fix me latter for 64 trees
  wire        dfs_done                                            [NUMBER_TREES-1:0];
  wire        dfs_ready                                           [NUMBER_TREES-1:0];
  wire        all_nodes_occupied                                  [NUMBER_TREES-1:0];
  wire        new_point_ready                                     [NUMBER_TREES-1:0];
  wire [ 7:0] occupation_code                                     [NUMBER_TREES-1:0];
  wire [31:0] occupation_code_size_bytes                          [NUMBER_TREES-1:0];
  wire [31:0] original_occupation_code_size_bytes;
  wire        occupation_code_id_is_inside_range                  [NUMBER_TREES-1:0];
  genvar i;  // Intermediate variable to iterate over the trees
  reg [2:0] selected_tree;  // Selected tree for bitstream writing

  // Signals used only for compression but they are present no matter what (rankings)
  wire [7:0] occupation_code_insertion[NUMBER_TREES-1:0];  // Codes inserted during dfs
  wire occupation_code_insertion_enable[NUMBER_TREES-1:0];  // Enables

  // Intermediate signals to hold the combined and flatten values of the trees
  integer k, j;
  reg                      combined_dfs_ready;
  reg                      combined_dfs_done;
  reg                      combined_new_point_ready;
  reg                      combined_all_nodes_occupied;
  reg [  NUMBER_TREES-1:0] flat_occupation_code_id_is_inside_range;
  reg [8*NUMBER_TREES-1:0] flat_occupation_code_insertion;
  reg [  NUMBER_TREES-1:0] flat_occupation_code_insertion_enable;

  always @(*) begin
    combined_dfs_ready = 1;
    combined_dfs_done = 1;
    combined_new_point_ready = 1;
    combined_all_nodes_occupied = 0;

    flat_occupation_code_id_is_inside_range = 0;
    flat_occupation_code_insertion = 0;
    flat_occupation_code_insertion_enable = 0;

    for (k = 0; k < NUMBER_TREES; k = k + 1) begin
      combined_dfs_ready = combined_dfs_ready & dfs_ready[k];
      combined_dfs_done = combined_dfs_done & dfs_done[k];
      combined_new_point_ready = combined_new_point_ready & new_point_ready[k];
      combined_all_nodes_occupied = combined_all_nodes_occupied | all_nodes_occupied[k];

      flat_occupation_code_id_is_inside_range[k+:1] = occupation_code_id_is_inside_range[k];
      flat_occupation_code_insertion[k*8+:8] = occupation_code_insertion[k];
      flat_occupation_code_insertion_enable[k] = occupation_code_insertion_enable[k];
    end
  end

  // FIFO for occupation code
  reg  [15:0] selected_occupation_code;
  reg  [ 4:0] selected_occupation_code_len;
  reg         wr_en;
  wire        rd_en;
  wire fifo_full, fifo_empty, still_bits_to_read;
  wire [$clog2(8):0] bits_left;
  wire               bs_to_write;

  alib_accumulator_fifo #(
      .DEPTH(8),
      .WIDTH_INPUT(16),
      .WIDTH_OUTPUT(8)
  ) bs_fifo (
      .i_clk(i_SYSTEM_clk),
      .i_rst(i_SYSTEM_rst && ~i_reset_octree),
      .i_wr_data(selected_occupation_code),
      .i_wr_data_len(selected_occupation_code_len),
      .i_wr_en(wr_en),
      .o_full(fifo_full),
      .o_empty(fifo_empty),
      .o_bits_left(bits_left),
      .i_rd_en(rd_en),
      .i_rd_data_len(8),
      .o_rd_data(o_occupation_code)
  );

  assign still_bits_to_read = (bits_left != 0) && (fifo_empty);

  // Tree generation blocks
  generate
    for (i = 0; i < NUMBER_TREES; i = i + 1) begin : TREES

      if (PARALLEL_LEVEL == 0) begin
        assign occupation_root_code = 7'b0000000;  // Dont care

      end else if (PARALLEL_LEVEL == 1) begin
        assign occupation_root_code[i:i] = occupation_code_size_bytes[i] > 0 ? 1 : 0;
      end

      localparam signed [15:0] N_BB_MIN_X = (PARALLEL_LEVEL == 0) ? BB_MIN_X : (BB_MIN_X + BB_SIZE_X * (i[0] ? 1 : 0));
      localparam signed [15:0] N_BB_MIN_Y = (PARALLEL_LEVEL == 0) ? BB_MIN_Y : (BB_MIN_Y + BB_SIZE_Y * (i[1] ? 1 : 0));
      localparam signed [15:0] N_BB_MIN_Z = (PARALLEL_LEVEL == 0) ? BB_MIN_Z : (BB_MIN_Z + BB_SIZE_Z * (i[2] ? 1 : 0));
      localparam signed [15:0] N_BB_MAX_X = (PARALLEL_LEVEL == 0) ? BB_MAX_X : (BB_MAX_X - BB_SIZE_X * (i[0] ? 0 : 1));
      localparam signed [15:0] N_BB_MAX_Y = (PARALLEL_LEVEL == 0) ? BB_MAX_Y : (BB_MAX_Y - BB_SIZE_Y * (i[1] ? 0 : 1));
      localparam signed [15:0] N_BB_MAX_Z = (PARALLEL_LEVEL == 0) ? BB_MAX_Z : (BB_MAX_Z - BB_SIZE_Z * (i[2] ? 0 : 1));

      alib_octree_handler #(
          .NUMBER_NODES(NUMBER_NODES_PER_OCTREE),
          .MAX_DEPTH(MAX_DEPTH - 1),
          .BB_MIN_X(N_BB_MIN_X),
          .BB_MIN_Y(N_BB_MIN_Y),
          .BB_MIN_Z(N_BB_MIN_Z),
          .BB_MAX_X(N_BB_MAX_X),
          .BB_MAX_Y(N_BB_MAX_Y),
          .BB_MAX_Z(N_BB_MAX_Z)
      ) tree (
          .i_SYSTEM_clk(i_SYSTEM_clk),
          .i_SYSTEM_rst(i_SYSTEM_rst),
          .i_new_point(i_new_point),
          .i_point_x(i_point_x),
          .i_point_y(i_point_y),
          .i_point_z(i_point_z),
          .i_reset_octree(i_reset_octree),
          .i_start_dfs(i_start_dfs),
          .i_occupation_code_id(occupation_code_id[i]),
          .o_dfs_done(dfs_done[i]),
          .o_dfs_ready(dfs_ready[i]),
          .o_occupation_code_insertion(occupation_code_insertion[i]),
          .o_occupation_code_insertion_enable(occupation_code_insertion_enable[i]),
          .o_all_nodes_occupied(all_nodes_occupied[i]),
          .o_new_point_ready(new_point_ready[i]),
          .o_occupation_code(occupation_code[i]),
          .o_occupation_code_size_bytes(occupation_code_size_bytes[i])
      );

      assign occupation_code_id_is_inside_range[i] = ((occupation_code_id[i] < occupation_code_size_bytes[i]) && occupation_code_size_bytes[i]!=0) ? 1 : 0;
    end
  endgenerate

  // Octree combinational logic
  assign o_new_point_ready = combined_new_point_ready;
  assign o_all_nodes_occupied = combined_all_nodes_occupied;
  assign bs_to_write = size_code_written < (original_occupation_code_size_bytes - 1) ? 1'b1 : 1'b0;
  assign original_occupation_code_size_bytes = PARALLEL_LEVEL==0? occupation_code_size_bytes[0] : 
                                    occupation_code_size_bytes[0] + occupation_code_size_bytes[1] + 
                                    occupation_code_size_bytes[2] + occupation_code_size_bytes[3] + 
                                    occupation_code_size_bytes[4] + occupation_code_size_bytes[5] + 
                                    occupation_code_size_bytes[6] + occupation_code_size_bytes[7] + 1;
  assign o_occupation_code_last_byte = o_dfs_done && fifo_empty && last_write_done && ~wr_en ? 1'b1 : 1'b0;

  // FIFO Read Logic
  assign rd_en = ~fifo_empty ? i_occupation_code_rd : 0;


  // COMPRESSION and NONCOMPRESSION octree blocks
  generate
    if (COMPRESSION == 1) begin : COMPRESSION_BLOCK  // Compression enabled
      // Ranking system and compression signals
      wire rank_done, rank_ready;
      wire [ 7:0] queried_rank;
      wire [15:0] compression_code;
      wire [ 3:0] compression_code_len;
      reg
          start_rank_calc,
          rank_started,
          wait_compression_code,
          wait_rank_code_dictionary,
          wait_rank_code,
          dictionary_to_write;
      reg [7:0] queried_char;
      reg [7:0] dictionary_counter;

      // Ranked Frequency Table
      alib_ranked_frequency_table #(
          .COUNTER_BITS(10),
          .NUMBER_OF_PARALLEL_INPUTS(8)
      ) frequency_table (
          .i_clk(i_SYSTEM_clk),
          .i_rst(i_SYSTEM_rst && ~i_reset_octree),
          .i_char(flat_occupation_code_insertion),
          .i_valid(flat_occupation_code_insertion_enable),
          .i_query_char(queried_char),
          .i_start_rank_calc(start_rank_calc),
          .o_rank_done(rank_done),
          .o_ready(rank_ready),
          .o_query_rank(queried_rank)
      );

      // Code LUT
      alib_code_lut code_lut (
          .i_rank(queried_rank),
          .o_code(compression_code),
          .o_code_len(compression_code_len)
      );

      // Dictionary creation state machine
      always @(posedge i_SYSTEM_clk) begin : RANK_CALC_LOGIC
        if (i_SYSTEM_rst && ~i_reset_octree) begin
          if (~rank_started && combined_dfs_done) begin  // Ranking can be started
            rank_started <= 1;  // Start creating dictionary
            start_rank_calc <= 1;
          end else if (rank_started) begin  // Waiting for dictionary
            start_rank_calc <= 0;
          end
        end else begin
          start_rank_calc <= 0;
          rank_started <= 0;
        end
      end

      // Output signals affetcted by compression
      assign o_dfs_ready = combined_dfs_ready & rank_ready;
      assign o_dfs_done = combined_dfs_done & rank_done;
      assign o_occupation_code_ready = fifo_empty ? 1'b0 : rank_done ? 1'b1 : 1'b0;

      // FIFO Write Logic
      always @(posedge i_SYSTEM_clk) begin : FIFO_WRITE_LOGIC
        if (i_SYSTEM_rst && ~i_reset_octree && rank_done) begin  // Ready to write codes
          if (fifo_full || wr_en) begin
            wr_en <= 0;  // Wait
          end else if (wait_rank_code) begin
            wait_rank_code <= 0;
            wait_compression_code <= 1;
          end else if (wait_compression_code) begin  // Compression code is ready
            selected_occupation_code <= compression_code;
            selected_occupation_code_len <= compression_code_len;
            wr_en <= 1;
            wait_compression_code <= 0;
          end else if (wait_rank_code_dictionary) begin  // Rank code is ready (to write dictionary)
            selected_occupation_code <= queried_rank;
            selected_occupation_code_len <= 8;
            wait_rank_code_dictionary <= 0;
            wr_en <= 1;
          end else if (dictionary_to_write) begin  // First step: Write dictionary 
            dictionary_counter = dictionary_counter<255?  dictionary_counter + 1 : dictionary_counter;
            dictionary_to_write <= queried_char == 255 ? 0 : 1;
            queried_char <= dictionary_counter;
            wait_rank_code_dictionary <= 1;
          end else if (!root_setup) begin  // Second step: Write root code
            root_setup <= 1;
            wait_rank_code <= 1;  // Write root compression code
            if (PARALLEL_LEVEL == 0) begin
              occupation_code_id[0] <= occupation_code_id[0] + 1;
              queried_char <= occupation_code[0];
            end else begin
              queried_char <= occupation_root_code;
            end
          end else if (bs_to_write) begin  // Third step: Write the rest of the codes
            // Flatten the arrays when calling the function
            selected_tree = select_tree_id(flat_occupation_code_id_is_inside_range, PARALLEL_LEVEL);
            size_code_written <= size_code_written + 1;
            occupation_code_id[selected_tree] <= occupation_code_id[selected_tree] + 1;
            queried_char <= occupation_code[selected_tree];
            wait_rank_code <= 1;
          end else if (~fifo_empty) begin  // Fifth step: Wait for FIFO to be without valid reads
            wait_compression_code <= 0;
            wait_rank_code_dictionary <= 0;
            wait_rank_code <= 0;
          end else if (still_bits_to_read) begin  // Last step: Write the last code with the remaining bits
            selected_occupation_code <= 0;
            selected_occupation_code_len <= 8 - bits_left;
            wr_en <= 1;
            last_write_done <= 1;
            o_occupation_code_last_valid_bits <= bits_left[2:0];
          end else begin
            last_write_done <= 1;
          end
        end else begin
          o_occupation_code_last_valid_bits <= 0;
          wait_compression_code <= 0;
          wait_rank_code_dictionary <= 0;
          wait_rank_code <= 0;
          dictionary_to_write <= 1;
          wr_en <= 0;
          last_write_done <= 0;
          root_setup <= 0;
          queried_char <= 0;
          selected_tree <= 0;
          selected_occupation_code <= 0;
          selected_occupation_code_len <= 0;
          size_code_written <= 0;
          dictionary_counter <= 0;
          for (j = 0; j < NUMBER_TREES; j = j + 1) begin
            occupation_code_id[j] <= 0;
          end
        end
      end
    end else begin : NO_COMPRESSION_BLOCK  // Compression disabled

      // Output signals not affected by compression
      assign o_dfs_ready = combined_dfs_ready;
      assign o_dfs_done = combined_dfs_done;
      assign o_occupation_code_ready = fifo_empty ? 1'b0 : 1'b1;

      always @(posedge i_SYSTEM_clk) begin : FIFO_WRITE_LOGIC
        if (i_SYSTEM_rst && ~i_reset_octree && o_dfs_done) begin  // Ready to write codes
          if (wr_en) wr_en <= 0;  // Reset write enable
          else if (!root_setup) begin
            root_setup <= 1;
            wr_en <= 1;
            if (PARALLEL_LEVEL == 0) begin
              occupation_code_id[0] <= occupation_code_id[0] + 1;
              selected_occupation_code <= occupation_code[0];
            end else begin
              selected_occupation_code <= occupation_root_code;
            end
            selected_occupation_code_len <= 8;
          end else if (bs_to_write) begin
            if (~fifo_full) begin
              selected_tree =
                  select_tree_id(flat_occupation_code_id_is_inside_range, PARALLEL_LEVEL);
              size_code_written <= size_code_written + 1;
              selected_occupation_code <= occupation_code[selected_tree];
              selected_occupation_code_len <= 8;
              wr_en <= 1;
              occupation_code_id[selected_tree] <= occupation_code_id[selected_tree] + 1;
            end
          end else if (fifo_empty) begin  // Fifth step: Wait for FIFO to be without valid reads
            wr_en <= 0;
            last_write_done <= 1;
          end
        end else begin
          o_occupation_code_last_valid_bits <= 0;
          wr_en <= 0;
          last_write_done <= 0;
          root_setup <= 0;
          selected_tree <= 0;
          selected_occupation_code <= 0;
          selected_occupation_code_len <= 0;
          size_code_written <= 0;
          for (j = 0; j < NUMBER_TREES; j = j + 1) begin
            occupation_code_id[j] <= 0;
          end
        end
      end
    end

  endgenerate

  function [2:0] select_tree_id;
    input [7:0] occupation_code_id_is_inside_range;  // Flattened valid range as 8 bits
    input integer PARALLEL_LEVEL;

    integer i;
    reg     found;  // Flag for early exit

    begin
      select_tree_id = 0;  // Default to -1 if no valid tree is found
      found          = 0;  // Initialize the flag to 0 (not found)

      if (PARALLEL_LEVEL == 0) begin
        // Single-tree case: Always return tree 0 if within range
        select_tree_id = 0;
      end else begin
        // Multi-tree case: Find the first valid tree in range
        for (i = 0; i < 8 && !found; i = i + 1) begin

          if (occupation_code_id_is_inside_range[i]) begin
            select_tree_id = i;
            found = 1;  // Set flag to 1 to exit the loop
          end
        end
      end
    end
  endfunction

endmodule
