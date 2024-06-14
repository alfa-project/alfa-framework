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

/* Module: MemMU_CR_P
    The MemMU Cartesian Representation Payload Module::

    The MemMU Cartesian Representation Payload module is *TODO*
*/
module MemMU_cartesianRepresentationPayload #(

    /* Parameters integer: Representation

        TODO - TODO. 
    */ 
	parameter integer RESOLUTION_X = 10,
	parameter integer RESOLUTION_Y = 10,
	parameter integer RESOLUTION_Z = 10
	)(

    /* Input: Input ports
    
        <MemMU::corretion> - Representation correction value.
        <SIU::distR0> - First return distance value.
        <SIU::distR1> - Second return distance value.
        <SIU::reflR0> - First return intensity/reflection value.
        <SIU::reflR1> - Second return intensity/reflection value.
        <SIU::label>  - Point label.
    */
    input [15:0] i_SIU_angleH,
    input [15:0] i_SIU_angleV,
    input [15:0] i_SIU_distR0, 
    input [15:0] i_SIU_distR1,
    input [7:0]  i_SIU_reflR0,
    input [7:0]  i_SIU_reflR1,
    input [7:0]  i_SIU_label,

    /* Output: Output ports

        payload - 64 bits output. Outputs the point clouds payload in the basic struture.
    */
    output [63:0] o_MemMU_CR_P_payload
    );

    /* Assigns: Main Combinational block

        *assign* o_MemMU_CR_P_payload:::

        - For the first 16 bits - <SIU::distR0>.
        - For the next 8 bits - <SIU::reflR0>.
        - For the next 16 bits - <SIU::distR1>.
        - For the next 8 bits - <SIU::reflR1>.
        - For the next 8 bits - <MemMU::correction>.
        - For the remaining bits - <SIU::label>.
    */  
    assign o_MemMU_CR_P_payload[15:0]  = i_SIU_distR0;
    assign o_MemMU_CR_P_payload[23:16] = i_SIU_reflR0;
    assign o_MemMU_CR_P_payload[39:24] = i_SIU_distR1;
    assign o_MemMU_CR_P_payload[47:40] = i_SIU_reflR1;
    assign o_MemMU_CR_P_payload[55:48] = 0;
    assign o_MemMU_CR_P_payload[63:56] = i_SIU_label;

endmodule