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

/* Module: MemMU_SR_A
    The MemMU Spherical Representation Address Module::

    The MemMU Spherical Representation Address module is responsible for translating sensor 
    angles to addresses, being this module a sequential approach. Based on the <FOV> and <NUMBER_OF_ADDR_BITS>
    the module tranlate a range of sensor angles into one address. The goal is configure the module so that the sensor
    only output one value for each address, thus the range of angles are the same as the sensor resolution. 
    Also, since ALFA defines linear angular increments that do not reflect the sensor ones, 
    a correction value is also stored to guarantee the maximum resolution possible. For example: For 
    a 360º <FOV> and 11 bits <NUMBER_OF_ADDR_BITS>, the maximum resolution is 360/2^(11)= 0.17578 degrees.
    This means that each address represents the angular value of 0.17578 times the address index. 
    Due to sensors' nonlinear firing paths, the difference in angle between the theoretical and the 
    actual sensor angles is stored as a correction value.
*/
module MemMU_sphericalRepresentationAddress #(

    /* Parameters integer: Representation

        FOV - Representation Field of View. 
        NUMBER_OF_ADDR_BITS - Number of address bits for storing the representation's angle.
    */
    parameter NUMBER_OF_ADDR_BITS = 14,
    parameter FOV = 360
    )(

    /* Input: Input ports

        <SIU::angleH> or <SIU::angleV> - Sensor angle.
    */  
    input [15:0] i_SIU_angle,    

    /* Output: Output ports

        correction - 4 bits output. Outputs the correction value between the address and actually the measured angle.
        address - [NUMBER_OF_ADDR_BITS-1:0] bits output. Outputs the representation address (Sequential).
    */
    output [3:0] o_MemMU_SR_A_correction,
    output [NUMBER_OF_ADDR_BITS-1:0] o_MemMU_SR_A_address
    );
    
    /* Localparams integer: Representation

        NUMBER_OF_INDIVIDUAL_MEASUREMENTS - Number of measurements to achieve the total FOV. 
        RESOLUTION - Resolution (degrees) for each measurement.
    */
    localparam integer NUMBER_OF_INDIVIDUAL_MEASUREMENTS = (2**NUMBER_OF_ADDR_BITS);
    localparam integer RESOLUTION = ((FOV*100) / NUMBER_OF_INDIVIDUAL_MEASUREMENTS)+1;
    localparam integer MAXFOV = (FOV*100)-1;
    localparam integer DEEP = NUMBER_OF_ADDR_BITS;
    localparam integer FACTOR = 2**(DEEP)/RESOLUTION;
    
    /* Wires: Connectors for horizontal axis

        true_angle - 16 bits connector to truncate FOV degree angles . 
        ang_of_address - 16 x <NUMBER_OF_INDIVIDUAL_MEASUREMENTS> static array with all angle to address translation. .  
        index_array - <NUMBER_OF_ADDR_BITS> bits connector that converts the received <i_SIU_angle> signal into the index used in <ang_of_address> 
    */
    wire [15:0] true_angle;
    wire [15:0] ang_of_address [NUMBER_OF_INDIVIDUAL_MEASUREMENTS-1:0];
    wire [NUMBER_OF_ADDR_BITS-1:0] index_array;
    wire [31:0] index_array_temp;
    
    assign true_angle = i_SIU_angle>=MAXFOV? MAXFOV : i_SIU_angle;
    
    /* Always: Main Combinational block
        *TODO*.
    */
    assign o_MemMU_SR_A_address  = (true_angle*FACTOR)>>DEEP;
    assign o_MemMU_SR_A_correction = true_angle - ((true_angle*FACTOR)>>DEEP)*RESOLUTION;
       
endmodule