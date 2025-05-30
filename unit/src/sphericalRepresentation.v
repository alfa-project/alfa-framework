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

/* Module: MemMU_SR
    Memory Management Unit Spherical Representation::

    The MemMU Spherical Representation configuration module makes ALFA handle point
    clouds in spherical representations. Since this representation is widespread across most 3D 
    LiDARs outputs, it speeds up the storing process since no much preprocessing is needed. Also,
    the point cloud size in spherical representations makes it very memory efficient. This module 
    is composed of two angles to address converters, one for each axis, converting angles to address 
    sequentially. Those translations are provided by two <MemMU_SR_A> modules. 
*/
module MemMU_sphericalRepresentation #(

    /* Parameters integer: Representation

        FOV_H - Representation Horizontal Field of View. 
        FOV_V - Representation Vertical Field of View. 
        NUMBER_OF_ADDR_BITS_H - Number of address bits for storing the representation's horizontal angle.
        NUMBER_OF_ADDR_BITS_V - Number of address bits for storing the representation's vertical angle.
    */
    parameter integer FOV_H = 360,
    parameter integer FOV_V = 90,
    parameter integer NUMBER_OF_ADDR_BITS_H = 11,
    parameter integer NUMBER_OF_ADDR_BITS_V = 5
) (

    /* Input: Input ports
    
        <SYSTEM::clk> - System clock. 
        <SYSTEM::rst> - System reset.
        <SIU::angleH> - Sensor horizontal angle.
        <SIU::angleV> - Sensor vertical angle.
        <SIU::distR0> - First return distance value.
        <SIU::distR1> - Second return distance value.
        <SIU::reflR0> - First return intensity/reflection value.
        <SIU::reflR1> - Second return intensity/reflection value.
        <SIU::label> - Point label.
    */
    input [ 0:0] i_SYSTEM_clk,
    input [ 0:0] i_SYSTEM_rst,
    input [15:0] i_SIU_angleH,
    input [15:0] i_SIU_angleV,
    input [15:0] i_SIU_distR0,
    input [15:0] i_SIU_distR1,
    input [ 7:0] i_SIU_reflR0,
    input [ 7:0] i_SIU_reflR1,
    input [ 7:0] i_SIU_label,


    /* Output: Output ports

        payload - 64 bits registered output. Outputs the representation payload.
        address - 17 bits registered output. Outputs the spherical representation address based on <SIU::angleH> and <SIU::angleV>.
    */
    output [18:0] o_MemMU_SR_size,
    output reg [63:0] o_MemMU_SR_payload,
    output reg [18:0] o_MemMU_SR_address
);

  /* Wires: Connectors for horizontal axis

        t_correction_h - 4 bits connector for <MemMU_SR_A::correction>. 
        t_address_h - <NUMBER_OF_ADDR_BITS_H> bits connector for <MemMU_SR_A::address>.  
    */
  wire [                      3:0] t_correction_h;
  wire [NUMBER_OF_ADDR_BITS_H-1:0] t_address_h;

  /* Wires: Connectors for vertical axis

        t_correction_v - 4 bits connector for <MemMU_SR_A::correction>. 
        t_address_v - <NUMBER_OF_ADDR_BITS_V> bits connector for <MemMU_SR_A::address>.      
    */
  wire [                      3:0] t_correction_v;
  wire [NUMBER_OF_ADDR_BITS_V-1:0] t_address_v;

  /* Wires: Temp Connetors

        correction - 8 bits connector for the correction value between the address and actually the measured angle.     
        payload - 63 bits connector for the representation payload.
        address - 15 bits connector for the representation address based on <SIU::angleH> and <SIU::angleV>.
    */
  wire [                      7:0] t_correction;
  wire [                     63:0] t_payload;
  wire [                     16:0] t_address;


  // MemMU_sphericalRepresentationAddress: angleToAddress_h
  // Instantiation of a <MemMU_SR_A> module for horizontal angles.
  MemMU_sphericalRepresentationAddress #(
      .NUMBER_OF_ADDR_BITS(NUMBER_OF_ADDR_BITS_H),
      .FOV(FOV_H)
  ) angleToAddress_h (
      .i_SIU_angle(i_SIU_angleH),
      .o_MemMU_SR_A_address(t_address_h),
      .o_MemMU_SR_A_correction(t_correction_h)
  );

  // MemMU_sphericalRepresentationAddress: angleToAddress_v
  // Instantiation of a <MemMU_SR_A> module for vertical angles.   
  MemMU_sphericalRepresentationAddress #(
      .NUMBER_OF_ADDR_BITS(NUMBER_OF_ADDR_BITS_V),
      .FOV(FOV_V)
  ) angleToAddress_v (
      .i_SIU_angle(i_SIU_angleV),
      .o_MemMU_SR_A_address(t_address_v),
      .o_MemMU_SR_A_correction(t_correction_v)
  );

  // MemMU_sphericalRepresentationPayload: payload
  // Instantiation of a <MemMU_SR_P> module to retrieve the payload.
  MemMU_sphericalRepresentationPayload payload (
      .i_MemMU_SR_A_correction(t_correction),
      .i_SIU_distR0(i_SIU_distR0),
      .i_SIU_distR1(i_SIU_distR1),
      .i_SIU_reflR0(i_SIU_reflR0),
      .i_SIU_reflR1(i_SIU_reflR1),
      .i_SIU_label(i_SIU_label),
      .o_MemMU_SR_P_payload(t_payload)
  );

  /* Assigns: Main Combinational block

        *assign* t_address:::
            - For the first <NUMBER_OF_ADDR_BITS_H> bits - <t_address_h>. 
            - For the next <NUMBER_OF_ADDR_BITS_V> bits -  <t_address_v>.
            - For the remaining bits - zero.

        *assign* t_correction::: 
            - For the first 4 bits - <t_correction_h>. 
            - For the second 4 bits - <t_correction_v> 

        *assign* o_MemMU_SR_size::: 
            - To - 2^(<NUMBER_OF_ADDR_BITS_H>+<NUMBER_OF_ADDR_BITS_V>) * 8 .    
    */
  assign t_correction[3:0] = t_correction_h;
  assign t_correction[7:4] = t_correction_v;
  assign o_MemMU_SR_size   = 2 ^ (NUMBER_OF_ADDR_BITS_H + NUMBER_OF_ADDR_BITS_V + 3);

  generate
    assign t_address[NUMBER_OF_ADDR_BITS_H-1:0] = t_address_h;
    assign t_address[NUMBER_OF_ADDR_BITS_H+NUMBER_OF_ADDR_BITS_V-1:NUMBER_OF_ADDR_BITS_H] = t_address_v;
    if ((NUMBER_OF_ADDR_BITS_V + NUMBER_OF_ADDR_BITS_H) < 17)
      assign t_address[16: ((NUMBER_OF_ADDR_BITS_V)+(NUMBER_OF_ADDR_BITS_H))] = 32'b0000_0000_0000_0000_0000_0000_0000_0000;
  endgenerate

  /* Always: Main Sequential block
        *Posedge clock and reset active low.* If the reset is active, the output of <o_MemMU_SR_A_address> and <o_MemMU_SR_A_correction> are zero. 
        Else, the output address is the one presented in <index_array> and the correction output is the diference between the adress's corresponded
        angle and the sensor angle.
    */
  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst) begin
      o_MemMU_SR_address <= t_address;
      o_MemMU_SR_payload <= t_payload;
    end else begin
      o_MemMU_SR_address <= 0;
      o_MemMU_SR_payload <= 0;
    end
  end

endmodule
