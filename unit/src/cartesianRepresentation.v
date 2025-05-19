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

/* Module: MemMU_CR
    Memory Management Unit Cartesian Representation::

    The MemMU Cartesian Representation configuration module makes ALFA handle point
    clouds in TODO 
*/
module MemMU_cartesianRepresentation #(

                                                                                                    /* Parameters integer: Representation

        TODO - TODO. 
    */
                                                                                                    parameter integer RESOLUTION_X = 10,
                                                                                                    parameter integer RESOLUTION_Y = 10,
                                                                                                    parameter integer RESOLUTION_Z = 10
) (

                                                                                                    /* Input: Input ports
    
        <SYSTEM::clk> - System clock. 
        <SYSTEM::rst> - System reset.
        <SIU::pointID> - Sensor point ID.
        <SIU::angleH> - Sensor horizontal angle.
        <SIU::angleV> - Sensor vertical angle.
        <SIU::distR0> - First return distance value.
        <SIU::distR1> - Second return distance value.
        <SIU::reflR0> - First return intensity/reflection value.
        <SIU::reflR1> - Second return intensity/reflection value.
        <SIU::label> - Point label.
    */
                                                                                                    input [0:0] i_SYSTEM_clk,
                                                                                                    input [0:0] i_SYSTEM_rst,
                                                                                                    input [18:0] i_SIU_pointID,
                                                                                                    input [15:0] i_SIU_angleH,
                                                                                                    input [15:0] i_SIU_angleV,
                                                                                                    input [15:0] i_SIU_distR0,
                                                                                                    input [15:0] i_SIU_distR1,
                                                                                                    input [7:0] i_SIU_reflR0,
                                                                                                    input [7:0] i_SIU_reflR1,
                                                                                                    input [7:0] i_SIU_label,


                                                                                                    /* Output: Output ports

        payload - 64 bits registered output. Outputs the representation payload.
        address - 17 bits registered output. Outputs the cartesian representation address based on <SIU::angleH> and <SIU::angleV>.
    */
                                                                                                    output reg [18:0] o_MemMU_CR_size,
                                                                                                    output reg [63:0] o_MemMU_CR_payload,
                                                                                                    output reg [18:0] o_MemMU_CR_address
);

  /* Wires: Temp Connetors

        payload - 63 bits connector for the representation payload.
        address - 15 bits connector for the representation address based on <SIU::angleH> and <SIU::angleV>.
    */
  wire [63:0] t_payload;
  wire [18:0] t_address;


  // MemMU_cartesianRepresentationAddress: address
  // Instantiation of a <MemMU_CR_A> module for getting address.
  MemMU_cartesianRepresentationAddress address (
                                                                                                      .i_SIU_pointID(i_SIU_pointID),
                                                                                                      .o_MemMU_CR_A_address(t_address)
  );

  // MemMU_cartesianRepresentationPayload: payload
  // Instantiation of a <MemMU_SR_P> module to retrieve the payload.
  MemMU_cartesianRepresentationPayload #(
                                                                                                      .RESOLUTION_X(RESOLUTION_X),
                                                                                                      .RESOLUTION_Y(RESOLUTION_Y),
                                                                                                      .RESOLUTION_Z(RESOLUTION_Z)
  ) payload (
                                                                                                      .i_SIU_angleH(i_SIU_angleH),
                                                                                                      .i_SIU_angleV(i_SIU_angleV),
                                                                                                      .i_SIU_distR0(i_SIU_distR0),
                                                                                                      .i_SIU_distR1(i_SIU_distR1),
                                                                                                      .i_SIU_reflR0(i_SIU_reflR0),
                                                                                                      .i_SIU_reflR1(i_SIU_reflR1),
                                                                                                      .i_SIU_label(i_SIU_label),
                                                                                                      .o_MemMU_CR_P_payload(t_payload)
  );

  /* Always: Main Sequential block
        *Posedge clock and reset active low.* TODO
    */
  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst) begin
      o_MemMU_CR_address <= t_address;
      o_MemMU_CR_payload <= t_payload;
      if (i_SIU_pointID > o_MemMU_CR_size) o_MemMU_CR_size = i_SIU_pointID;
    end else begin
      o_MemMU_CR_address <= 0;
      o_MemMU_CR_payload <= 0;
      o_MemMU_CR_size = 0;
    end
  end

endmodule
