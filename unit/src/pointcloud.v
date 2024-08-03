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

/* Module: MemMU_P
    Memory Management Unit Pointcloud::

    The MemMU Representation configuration module makes ALFA handle multiple point clouds representations. *TODO*. 
*/
module MemMU_pointcloud #(

    /* Parameters integer: Pointcloud.

        MEMORY_OFFSET = 0;
        POINTCLOUD_ID - Unique sensor ID.
        NUMBER_OF_REPRESENTATIONS - Number of representations configurated in ALFA.
        REPRESENTATION_TYPE - *TODO*
        REPRESENTATION_PARAM1 - *TODO*
        REPRESENTATION_PARAM2 - *TODO*
        REPRESENTATION_PARAM3 - *TODO*
        REPRESENTATION_PARAM4 - *TODO*
    */
    parameter [31:0] OFFSET = 0,
    parameter [0:0] BRAM_DDR = 1'b1,
    parameter [2:0] POINTCLOUD_ID = 0,
    parameter [7:0] REPRESENTATION_TYPE = 0,
    parameter [7:0] REPRESENTATION_PARAM1 = 360,
    parameter [7:0] REPRESENTATION_PARAM2 = 90,
    parameter [7:0] REPRESENTATION_PARAM3 = 11,
    parameter [7:0] REPRESENTATION_PARAM4 = 5
) (

    /* Input: Input ports

        <System::clk> - System clock. 
        <System::rst> - System reset.
        <SIU::pointID> - Sensor point ID.
        <SIU::angleH> - Sensor horizontal angle.
        <SIU::angleV> - Sensor vertical angle.
        <SIU::distR0> - First return distance value.
        <SIU::distR1> - Second return distance value.
        <SIU::reflR0> - First return intensity/reflection value.
        <SIU::reflR1> - Second return intensity/reflection value.
        <ExMU::label> - Point label.
        <MemMU::representation> - *TODO*.
        <MemMU::frame> - *TODO*.
        <MemMU_address> - *TODO*.

    */
    input i_SYSTEM_clk,
    input i_SYSTEM_rst,
    input [31:0] i_MonU_MemMU_parameter,
    input [18:0] i_SIU_pointID,
    input [0:0] i_SIU_newFrame,
    input [15:0] i_SIU_angleH,
    input [15:0] i_SIU_angleV,
    input [15:0] i_SIU_distR0,
    input [7:0] i_SIU_reflR0,
    input [15:0] i_SIU_distR1,
    input [7:0] i_SIU_reflR1,
    input [7:0] i_SIU_label,
    input [18:0] i_ExMU_pointReWriteID,
    input [18:0] i_ExMU_pointReadID,
    input [2047:0] i_ExMU_pointReWritePayload,
    output wire [31:0] o_MemMU_P_writeAddress,
    output wire [63:0] o_MemMU_P_writePayload,
    output wire [31:0] o_MemMU_P_readAddress,
    output wire [31:0] o_MemMU_P_rewriteAddress,
    output wire [2047:0] o_MemMU_P_rewritePayload,
    output wire [18:0] o_MemMU_P_size
);

  wire [18:0] representationAddress;

  reg  [ 0:0] frame;

  generate
    if (BRAM_DDR == 0) begin  //BRAM
      assign o_MemMU_P_writeAddress[18:0] = representationAddress;
      assign o_MemMU_P_writeAddress[21:19] = POINTCLOUD_ID;
      assign o_MemMU_P_writeAddress[31:22] = OFFSET[31:22];

      assign o_MemMU_P_readAddress[18:0] = i_ExMU_pointReadID;
      assign o_MemMU_P_readAddress[21:19] = POINTCLOUD_ID;
      assign o_MemMU_P_readAddress[31:22] = OFFSET[31:22];

      assign o_MemMU_P_rewriteAddress[18:0] = i_ExMU_pointReWriteID;
      assign o_MemMU_P_rewriteAddress[21:19] = POINTCLOUD_ID;
      assign o_MemMU_P_rewriteAddress[31:22] = OFFSET[31:22];

      assign o_MemMU_P_rewritePayload = i_ExMU_pointReWritePayload;
    end else begin  //DDR

      wire [31:0] writeAddress_high_bits;
      wire [31:0] readAddress_high_bits;
      wire [31:0] rewriteAddress_high_bits;

      assign writeAddress_high_bits[2:0] = 3'b00;
      assign writeAddress_high_bits[21:3] = representationAddress;
      assign writeAddress_high_bits[31:22] = 0;

      assign readAddress_high_bits[2:0] = 3'b00;
      assign readAddress_high_bits[21:3] = i_ExMU_pointReadID;
      assign readAddress_high_bits[31:22] = 0;

      assign rewriteAddress_high_bits[2:0] = 3'b00;
      assign rewriteAddress_high_bits[21:3] = i_ExMU_pointReWriteID;
      assign rewriteAddress_high_bits[31:22] = 0;


      assign o_MemMU_P_writeAddress = writeAddress_high_bits + i_MonU_MemMU_parameter;
      assign o_MemMU_P_readAddress = readAddress_high_bits + i_MonU_MemMU_parameter;
      assign o_MemMU_P_rewriteAddress = rewriteAddress_high_bits + i_MonU_MemMU_parameter;

      assign o_MemMU_P_rewritePayload = i_ExMU_pointReWritePayload;

    end

    if (REPRESENTATION_TYPE == 0) begin
      MemMU_sphericalRepresentation #(
          .FOV_H(REPRESENTATION_PARAM1),
          .FOV_V(REPRESENTATION_PARAM2),
          .NUMBER_OF_ADDR_BITS_H(REPRESENTATION_PARAM3),
          .NUMBER_OF_ADDR_BITS_V(REPRESENTATION_PARAM4)
      ) representation (
          .i_SYSTEM_clk(i_SYSTEM_clk),
          .i_SYSTEM_rst(i_SYSTEM_rst),
          .i_SIU_angleH(i_SIU_angleH),
          .i_SIU_angleV(i_SIU_angleV),
          .i_SIU_distR0(i_SIU_distR0),
          .i_SIU_distR1(i_SIU_distR1),
          .i_SIU_reflR0(i_SIU_reflR0),
          .i_SIU_reflR1(i_SIU_reflR1),
          .i_SIU_label(i_SIU_label),
          .o_MemMU_SR_size(o_MemMU_P_size),
          .o_MemMU_SR_payload(o_MemMU_P_writePayload),
          .o_MemMU_SR_address(representationAddress)
      );
    end else if (REPRESENTATION_TYPE == 1) begin
      MemMU_cartesianRepresentation #(
          .RESOLUTION_X(REPRESENTATION_PARAM1),
          .RESOLUTION_Y(REPRESENTATION_PARAM2),
          .RESOLUTION_Z(REPRESENTATION_PARAM3)
      ) representation (
          .i_SYSTEM_clk(i_SYSTEM_clk),
          .i_SYSTEM_rst(i_SYSTEM_rst),
          .i_SIU_pointID(i_SIU_pointID),
          .i_SIU_angleH(i_SIU_angleH),
          .i_SIU_angleV(i_SIU_angleV),
          .i_SIU_distR0(i_SIU_distR0),
          .i_SIU_distR1(i_SIU_distR1),
          .i_SIU_reflR0(i_SIU_reflR0),
          .i_SIU_reflR1(i_SIU_reflR1),
          .i_SIU_label(i_SIU_label),
          .o_MemMU_CR_size(o_MemMU_P_size),
          .o_MemMU_CR_payload(o_MemMU_P_writePayload),
          .o_MemMU_CR_address(representationAddress)
      );
    end
  endgenerate

  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst) begin
      if (i_SIU_newFrame) frame <= ~frame;
      else frame <= frame;
    end else frame <= 1'b0;
  end

endmodule
