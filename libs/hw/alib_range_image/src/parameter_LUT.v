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
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 05/08/2024 03:02:16 PM
// Design Name:
// Module Name: parameter_LUT
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module parameter_LUT(
  input  wire  [1:0] i_SensorType , //00-HDL64 01-HDL32 10-VLP16 11-VLS128
  output reg  [15:0] o_riWidth    ,
  output reg   [7:0] o_riHeight   ,
  output reg  [15:0] o_maxAngle   ,
  output reg  [15:0] o_multiplier ,
  output reg   [7:0] o_shiftFactor,
  output reg  [15:0] o_deltaAngle ,
  output reg   [7:0] o_Hoffset    ,
  output reg   [7:0] o_Voffset
  );

  always @(*) begin
    o_riWidth     <= (i_SensorType == 2'b00) ? 15'd2047 : (i_SensorType == 2'b01) ? 15'd2047 : (i_SensorType == 2'b10) ? 15'd2047 : (i_SensorType == 2'b11) ? 15'd2047 : 15'd0;
    o_riHeight    <= (i_SensorType == 2'b00) ? 8'd127 : (i_SensorType == 2'b01) ? 8'd31 : (i_SensorType == 2'b10) ? 8'd15 : (i_SensorType == 2'b11) ? 8'd255 : 8'd0;
    o_maxAngle    <= (i_SensorType == 2'b00) ? 15'd550 : (i_SensorType == 2'b01) ? 15'd1100 : (i_SensorType == 2'b10) ? 15'd1600 : (i_SensorType == 2'b11) ? 15'd1600 : 15'd0;
    o_multiplier  <= (i_SensorType == 2'b00) ? 15'd338 : (i_SensorType == 2'b01) ? 15'd998 : (i_SensorType == 2'b10) ? 15'd347 : (i_SensorType == 2'b11) ? 15'd988 : 15'd0;
    o_shiftFactor <= (i_SensorType == 2'b00) ? 8'd20 : (i_SensorType == 2'b01) ? 8'd22 : (i_SensorType == 2'b10) ? 8'd20 : (i_SensorType == 2'b11) ? 8'd22 : 8'd0;
    o_deltaAngle  <= (i_SensorType == 2'b00) ? 15'd3100 : (i_SensorType == 2'b01) ? 15'd4200 : (i_SensorType == 2'b10) ? 15'd3020 : (i_SensorType == 2'b11) ? 15'd4250 : 15'd0;
    o_Hoffset     <= (i_SensorType == 2'b00) ? 8'd8 : (i_SensorType == 2'b01) ? 8'd8 : (i_SensorType == 2'b10) ? 8'd8 : (i_SensorType == 2'b11) ? 8'd8 : 8'd0;
    o_Voffset     <= (i_SensorType == 2'b00) ? 8'd12 : (i_SensorType == 2'b01) ? 8'd67 : (i_SensorType == 2'b10) ? 8'd94 : (i_SensorType == 2'b11) ? 8'd8 : 8'd0;
  end

endmodule
