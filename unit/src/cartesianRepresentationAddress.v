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

/* Module: MemMU_CR_A
    The MemMU Cartesian Representation Address Module::

    The MemMU Cartesian Representation Address module is responsible for translating sensor 
    angles to addresses, being this module a sequential approach. TODO
*/
module MemMU_cartesianRepresentationAddress (

    /* Input: Input ports

        TODO.
    */  
    input [18:0] i_SIU_pointID,    

    /* Output: Output ports

        TODO
    */
    output [18:0] o_MemMU_CR_A_address
    );
    
    /* Always: Main Combinational block
        *TODO*.
    */
    assign o_MemMU_CR_A_address  = i_SIU_pointID;     
endmodule