/*
 * Copyright 2023 ALFA Project. All rights reserved.
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

/* Module: MemMU
	Memory Management Unit

    The MMU module is the main . Here the interfaces, point clouds setups and representations are configured
    and connected to the remaining of ALFA Units. This module is composed of four main sections:

    - Representation instantiation
    - Point Cloud instantiation
    - Frame management
    - State machine for each interface:

        -  - ;
        - Read point cloud for algorithm processing.  
*/
module MemMU #(
    /* Parameters integer: Point Clouds, Representations, Frames and States.
    */
    parameter integer OFFSET = 0,
    parameter [0:0] BRAM_DDR = 1'b1,
    parameter [2:0] POINTCLOUD_ID = 1'b0,
    parameter [7:0] REPRESENTATION_TYPE = 1'b0,
    parameter integer REPRESENTATION_PARAM1 = 32'd360,
    parameter integer REPRESENTATION_PARAM2 = 32'd90,
    parameter integer REPRESENTATION_PARAM3 = 32'd11,
    parameter integer REPRESENTATION_PARAM4 = 32'd5
) (

    /* Input: Input ports

    */
    input wire [0:0] i_SYSTEM_clk,
    input wire [0:0] i_SYSTEM_rst,
    input wire [31:0] i_MonU_MemMU_parameter,
    input wire [0:0] i_CU_SIU_ExMU,
    input wire [18:0] i_SIU_pointID,
    input wire [0:0] i_SIU_newFrame,
    input wire [15:0] i_SIU_angleH,
    input wire [15:0] i_SIU_angleV,
    input wire [15:0] i_SIU_distR0,
    input wire [7:0] i_SIU_reflR0,
    input wire [15:0] i_SIU_distR1,
    input wire [7:0] i_SIU_reflR1,
    input wire [7:0] i_SIU_label,
    input wire [18:0] i_ExMU_pointReadID,
    input wire [18:0] i_ExMU_pointReWriteID,
    input wire [2047:0] i_ExMU_pointReWritePayload,



    /* Output: Output ports

    */
    output wire [  31:0] o_MemMU_pointReadAddress,
    output reg  [  31:0] o_MemMU_pointWriteAddress,
    output reg  [2047:0] o_MemMU_pointWritePayload,
    output wire [  18:0] o_MemMU_size,
    output reg  [  31:0] o_status
);

  wire [  31:0] writeAddress;
  wire [  63:0] writePayload;
  wire [  31:0] readAddress;
  wire [  31:0] reWriteAddress;
  wire [2047:0] reWritePayload;

  MemMU_pointcloud #(
      .OFFSET(OFFSET),
      .BRAM_DDR(BRAM_DDR),
      .POINTCLOUD_ID(POINTCLOUD_ID),
      .REPRESENTATION_TYPE(REPRESENTATION_TYPE),
      .REPRESENTATION_PARAM1(REPRESENTATION_PARAM1),
      .REPRESENTATION_PARAM2(REPRESENTATION_PARAM2),
      .REPRESENTATION_PARAM3(REPRESENTATION_PARAM3),
      .REPRESENTATION_PARAM4(REPRESENTATION_PARAM4)
  ) inst_MemMU_pointcloud (
      .i_SYSTEM_clk(i_SYSTEM_clk),
      .i_SYSTEM_rst(i_SYSTEM_rst),
      .i_MonU_MemMU_parameter(i_MonU_MemMU_parameter),
      .i_SIU_pointID(i_SIU_pointID),
      .i_SIU_newFrame(i_SIU_newFrame),
      .i_SIU_angleH(i_SIU_angleH),
      .i_SIU_angleV(i_SIU_angleV),
      .i_SIU_distR0(i_SIU_distR0),
      .i_SIU_reflR0(i_SIU_reflR0),
      .i_SIU_distR1(i_SIU_distR1),
      .i_SIU_reflR1(i_SIU_reflR1),
      .i_SIU_label(i_SIU_label),
      .i_ExMU_pointReWriteID(i_ExMU_pointReWriteID),
      .i_ExMU_pointReadID(i_ExMU_pointReadID),
      .i_ExMU_pointReWritePayload(i_ExMU_pointReWritePayload),
      .o_MemMU_P_writeAddress(writeAddress),
      .o_MemMU_P_writePayload(writePayload),
      .o_MemMU_P_readAddress(readAddress),
      .o_MemMU_P_rewriteAddress(reWriteAddress),
      .o_MemMU_P_rewritePayload(reWritePayload),
      .o_MemMU_P_size(o_MemMU_size)
  );

  assign o_MemMU_pointReadAddress = readAddress;

  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst) begin
      o_status <= 1'b1;
      if (~i_CU_SIU_ExMU) begin
        o_MemMU_pointWriteAddress <= writeAddress;
        o_MemMU_pointWritePayload <= writePayload;
      end else begin
        o_MemMU_pointWriteAddress <= reWriteAddress;
        o_MemMU_pointWritePayload <= reWritePayload;
      end
    end else begin
      o_status <= 1'b0;
      o_MemMU_pointWriteAddress <= 0;
      o_MemMU_pointWritePayload <= 0;
    end
  end
endmodule
