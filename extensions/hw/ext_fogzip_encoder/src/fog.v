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

module ext_fog #(
    parameter [31:0] NUMBER_NODES = 100000,
    parameter PARALLEL_PROCESSING = 1,
    parameter COMPRESSION = 1,
    parameter signed [15:0] BB_MIN_X = 0,
    parameter signed [15:0] BB_MIN_Y = 0,
    parameter signed [15:0] BB_MIN_Z = 0,
    parameter signed [15:0] BB_MAX_X = 0,
    parameter signed [15:0] BB_MAX_Y = 0,
    parameter signed [15:0] BB_MAX_Z = 0,
    parameter [7:0] MAX_DEPTH = 8
) (
    input  wire        i_SYSTEM_clk,
    input  wire        i_SYSTEM_rst,
    // Extension_Interface
    output wire [ 0:0] EXT_writeValid,
    input  wire [ 0:0] EXT_writeReady,
    output reg  [ 0:0] EXT_readReady,
    input  wire [ 0:0] EXT_readValid,
    input  wire [18:0] EXT_PCSize,
    output reg  [ 0:0] EXT_doneProcessing,
    output wire [15:0] EXT_writeCustomField,
    input  wire [15:0] EXT_readCustomField,
    output wire [18:0] EXT_writeID,
    output reg  [18:0] EXT_readID,
    input  wire [ 0:0] EXT_enable,
    output wire [31:0] EXT_status,
    // Cartesian Representation
    input  wire [15:0] EXT_pointX,
    input  wire [15:0] EXT_pointY,
    input  wire [15:0] EXT_pointZ,
    output reg  [31:0] EXT_MEM_writeAddress,
    output reg  [63:0] EXT_MEM_writePayload,
    output wire [31:0] EXT_MEM_readAddress,
    input  wire [63:0] EXT_MEM_readPayload,
    output reg         EXT_MEM_initWriteTxn,
    output wire        EXT_MEM_initReadTxn,
    input  wire        EXT_MEM_writeTxnDone,
    input  wire        EXT_MEM_readTxnDone,
    input  wire        EXT_MEM_error
);

  localparam [3:0] RESET = 4'b0000,  // Initial state
  IDLE = 4'b0001,  // Idle state

  // Reading and inserting points
  READ_POINT = 4'b0010, INSERT_POINT_OCTREE = 4'b0011, WAIT_INSERT = 4'b0100,

  // Depth-First Search (DFS) related states
  START_DFS           = 4'b0101, 
  WAIT_DFS            = 4'b0110,
  GET_BS_WORD         = 4'b0111,
  GET_BS_WORD_W1      = 4'b1000,
  GET_BS_WORD_W2      = 4'b1001,

  // Writing bitstream
  WRITE_BS_WORD = 4'b1010, WRITE_BS_SIZE = 4'b1011, WAIT_WRITE_BS_WORD = 4'b1100,

  // Final state
  DONE_PROCESSING = 4'b1101,  // Final state

  // Error state
  ERROR = 4'b1111;  // Error state

  reg [ 3:0] state;
  reg [18:0] point_counter;
  reg [31:0] bs_word_address;
  reg [ 2:0] bs_word_selector;
  reg [ 7:0] bs_word_temp     [7:0];
  reg new_point, reset_octree, start_dfs, rd_occupation_code, bs_size_set;
  wire dfs_done, dfs_ready, all_nodes_occupied, new_point_ready, points_to_read;
  wire bs_to_write, occupation_code_ready, occupation_code_last_byte;
  wire [ 2:0] occupation_code_last_valid_bits;
  wire [ 7:0] occupation_code_w;
  wire        read_done;
  reg  [31:0] bs_word_counter;

  (* keep="soft" *)wire [63:0] unconnected_wire_0;
  (* keep="soft" *)wire [15:0] unconnected_wire_1;
  (* keep="soft" *) wire unconnected_wire_2, unconnected_wire_3, unconnected_wire_4;

  assign unconnected_wire_0 = EXT_MEM_readPayload;
  assign unconnected_wire_1 = EXT_readCustomField;
  assign unconnected_wire_2 = EXT_MEM_error;
  assign unconnected_wire_3 = EXT_writeReady;
  assign read_done = (EXT_readReady && EXT_readValid) ? 1'b1 : 1'b0;
  assign points_to_read = point_counter < EXT_PCSize ? 1'b1 : 1'b0;
  assign bs_to_write = occupation_code_last_byte ? 1'b0 : 1'b1;


  alib_octree #(
      .NUMBER_NODES(NUMBER_NODES),
      .PARALLEL_LEVEL(PARALLEL_PROCESSING),
      .COMPRESSION(COMPRESSION),
      .SIZE_ID($clog2(NUMBER_NODES)),
      .BB_MIN_X(BB_MIN_X),
      .BB_MIN_Y(BB_MIN_Y),
      .BB_MIN_Z(BB_MIN_Z),
      .BB_MAX_X(BB_MAX_X),
      .BB_MAX_Y(BB_MAX_Y),
      .BB_MAX_Z(BB_MAX_Z),
      .MAX_DEPTH(MAX_DEPTH)
  ) uut (
      .i_SYSTEM_clk(i_SYSTEM_clk),
      .i_SYSTEM_rst(i_SYSTEM_rst),
      .i_occupation_code_rd(rd_occupation_code),
      .i_new_point(new_point),
      .i_point_x(EXT_pointX),
      .i_point_y(EXT_pointY),
      .i_point_z(EXT_pointZ),
      .i_reset_octree(reset_octree),
      .i_start_dfs(start_dfs),
      .o_dfs_done(dfs_done),
      .o_dfs_ready(dfs_ready),
      .o_all_nodes_occupied(all_nodes_occupied),
      .o_occupation_code_ready(occupation_code_ready),
      .o_new_point_ready(new_point_ready),
      .o_occupation_code(occupation_code_w),
      .o_occupation_code_last_valid_bits(occupation_code_last_valid_bits),
      .o_occupation_code_last_byte(occupation_code_last_byte)
  );

  assign EXT_status[3:0] = state;
  assign EXT_status[31:4] = 29'd0;
  assign EXT_MEM_readAddress = 0;
  assign EXT_MEM_initReadTxn = 0;
  assign EXT_writeID = 0;
  assign EXT_writeValid = 0;
  assign EXT_MEM_initReadTxn = 0;
  assign EXT_writeCustomField = 16'd0;


  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst) begin
      case (state)

        RESET:
        if (reset_octree) state <= IDLE;
        else state <= RESET;

        ERROR: state <= DONE_PROCESSING;

        IDLE:
        if (EXT_enable && points_to_read) state <= READ_POINT;
        else state <= IDLE;

        READ_POINT:
        if (~EXT_enable) state <= RESET;
        else if (read_done) state <= INSERT_POINT_OCTREE;
        else state <= READ_POINT;

        INSERT_POINT_OCTREE:
        if (~EXT_enable) state <= RESET;
        else state <= WAIT_INSERT;

        WAIT_INSERT:
        if (~EXT_enable) state <= RESET;
        else if (all_nodes_occupied) state <= ERROR;
        else if (new_point) state <= WAIT_INSERT;
        else if (new_point_ready) begin
          if (points_to_read) state <= READ_POINT;
          else if (dfs_ready) state <= START_DFS;
          else state <= WAIT_INSERT;
        end else state <= WAIT_INSERT;

        START_DFS:
        if (~EXT_enable) state <= RESET;
        else state <= WAIT_DFS;

        WAIT_DFS:
        if (~EXT_enable) state <= RESET;
        else if (dfs_done) state <= GET_BS_WORD_W2;
        else state <= WAIT_DFS;

        GET_BS_WORD:
        if (~EXT_enable) state <= RESET;
        else if (bs_word_selector == 7 || occupation_code_last_byte) state <= WRITE_BS_WORD;
        else state <= GET_BS_WORD_W2;

        GET_BS_WORD_W1:
        if (~EXT_enable) state <= RESET;
        else state <= GET_BS_WORD;

        GET_BS_WORD_W2:
        if (~EXT_enable) state <= RESET;
        else if (occupation_code_ready) state <= GET_BS_WORD_W1;
        else if (occupation_code_last_byte) state <= WRITE_BS_SIZE;
        else state <= GET_BS_WORD_W2;

        WRITE_BS_WORD:
        if (~EXT_enable) state <= RESET;
        else state <= WAIT_WRITE_BS_WORD;

        WAIT_WRITE_BS_WORD:
        if (~EXT_enable) state <= RESET;
        else if (EXT_MEM_writeTxnDone) begin
          if (bs_to_write) state <= GET_BS_WORD_W2;
          else if (~bs_size_set) state <= WRITE_BS_SIZE;
          else state <= DONE_PROCESSING;
        end else state <= WAIT_WRITE_BS_WORD;

        WRITE_BS_SIZE:
        if (~EXT_enable) state <= RESET;
        else state <= WAIT_WRITE_BS_WORD;

        DONE_PROCESSING:
        if (~EXT_enable) state <= RESET;
        else if (EXT_doneProcessing && !EXT_enable) state <= RESET;
        else state <= DONE_PROCESSING;

        default: state <= IDLE;
      endcase
    end else begin
      state <= RESET;
    end
  end

  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst) begin
      case (state)

        RESET: begin
          new_point <= 1'b0;
          EXT_readReady <= 1'b0;
          EXT_doneProcessing <= 1'b0;
          EXT_readID <= 1'b0;
          EXT_MEM_writeAddress <= 0;
          EXT_MEM_initWriteTxn <= 0;
          bs_word_address <= 32'd8;
          bs_word_selector <= 3'd0;
          point_counter <= 19'd0;
          rd_occupation_code <= 1'b0;
          reset_octree <= 1'b1;
          bs_word_temp[0] <= 8'd0;
          bs_word_temp[1] <= 8'd0;
          bs_word_temp[2] <= 8'd0;
          bs_word_temp[3] <= 8'd0;
          bs_word_temp[4] <= 8'd0;
          bs_word_temp[5] <= 8'd0;
          bs_word_temp[6] <= 8'd0;
          bs_word_temp[7] <= 8'd0;
          EXT_MEM_writePayload <= 64'd0;
          bs_size_set <= 1'b0;
          bs_word_counter = 0;
        end

        ERROR: begin
          EXT_readReady <= 1'b0;
        end

        IDLE: begin
          EXT_readReady <= 1'b0;
          EXT_doneProcessing <= 1'b0;
          EXT_readID <= EXT_readID;
          EXT_MEM_initWriteTxn <= 1'b0;
          reset_octree <= 1'b0;
          start_dfs <= 1'b0;
        end

        READ_POINT:
        if (read_done) begin
          point_counter <= point_counter + 1;
          EXT_readReady <= 1'b0;
        end else begin
          EXT_readReady <= 1'b1;
          EXT_readID <= point_counter;
        end

        INSERT_POINT_OCTREE: begin
          new_point <= 1'b1;
        end

        WAIT_INSERT: begin
          new_point <= 1'b0;
        end

        START_DFS: begin
          start_dfs <= 1'b1;
        end

        WAIT_DFS: begin
          start_dfs <= 1'b0;
        end

        GET_BS_WORD: begin
          bs_word_temp[bs_word_selector] <= occupation_code_w;
          bs_word_selector <= bs_word_selector + 1;
          bs_word_counter = bs_word_counter + 1;
        end

        GET_BS_WORD_W1: begin
          rd_occupation_code <= 1'b0;
        end

        GET_BS_WORD_W2: begin
          if (occupation_code_ready) rd_occupation_code <= 1'b1;
        end

        WRITE_BS_SIZE: begin
          EXT_MEM_writeAddress <= 0;
          EXT_MEM_writePayload[31:0] <= bs_word_counter;
          EXT_MEM_writePayload[63:32] <= occupation_code_last_valid_bits;
          EXT_MEM_initWriteTxn <= 1'b1;
          bs_size_set <= 1'b1;
        end

        WRITE_BS_WORD: begin
          EXT_MEM_writePayload[7:0]   = bs_word_temp[0];
          EXT_MEM_writePayload[15:8]  = bs_word_temp[1];
          EXT_MEM_writePayload[23:16] = bs_word_temp[2];
          EXT_MEM_writePayload[31:24] = bs_word_temp[3];
          EXT_MEM_writePayload[39:32] = bs_word_temp[4];
          EXT_MEM_writePayload[47:40] = bs_word_temp[5];
          EXT_MEM_writePayload[55:48] = bs_word_temp[6];
          EXT_MEM_writePayload[63:56] = bs_word_temp[7];
          EXT_MEM_writeAddress <= bs_word_address;
          bs_word_selector <= 3'd0;
          EXT_MEM_initWriteTxn <= 1'b1;
          bs_word_address <= bs_word_address + 8;
        end

        WAIT_WRITE_BS_WORD: begin
          if (EXT_MEM_writeTxnDone) EXT_MEM_initWriteTxn <= 1'b0;
          else EXT_MEM_initWriteTxn <= 1'b1;
        end

        DONE_PROCESSING: begin
          EXT_doneProcessing <= 1'b1;
          EXT_readReady <= 1'b0;
          point_counter <= 0;
        end

        default: begin
          EXT_readReady <= 1'b0;
          EXT_doneProcessing <= 1'b0;
          EXT_readID <= 1'b0;
          point_counter <= 32'd0;
        end
      endcase
    end else begin
      new_point <= 1'b0;
      EXT_readReady <= 1'b0;
      EXT_doneProcessing <= 1'b0;
      EXT_readID <= 1'b0;
      EXT_MEM_writeAddress <= 0;
      EXT_MEM_initWriteTxn <= 0;
      bs_word_address <= 32'd8;
      bs_word_selector <= 3'd0;
      point_counter <= 19'd0;
      rd_occupation_code <= 1'b0;
      reset_octree <= 1'b1;
      bs_word_temp[0] <= 8'd0;
      bs_word_temp[1] <= 8'd0;
      bs_word_temp[2] <= 8'd0;
      bs_word_temp[3] <= 8'd0;
      bs_word_temp[4] <= 8'd0;
      bs_word_temp[5] <= 8'd0;
      bs_word_temp[6] <= 8'd0;
      bs_word_temp[7] <= 8'd0;
      EXT_MEM_writePayload <= 64'd0;
      bs_size_set <= 1'b0;
      bs_word_counter = 0;
    end
  end

endmodule
