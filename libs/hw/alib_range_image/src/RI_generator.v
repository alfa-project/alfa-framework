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
// Create Date: 05/02/2024 10:12:55 AM
// Design Name:
// Module Name: RI_generator
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

module RI_generator (
  input  wire        clk           ,
  input  wire        reset         ,
  input  wire [15:0] i_azimuth     ,
  input  wire [15:0] i_elevation   ,
  input  wire [15:0] i_range       ,
  input  wire  [1:0] i_SensorSelect,
  output wire        o_validAngle  ,
  output reg  [18:0] o_wAddress    ,
  output reg  [15:0] o_x           ,
  output reg   [7:0] o_y           ,
  output reg  [15:0] o_range
  );

  wire [15:0] riWidth       ;
  wire  [7:0] riHeight      ;
  wire [15:0] maxAngle      ;
  wire [15:0] multiplier    ;
  wire  [7:0] shiftFactor   ;
  wire [15:0] deltaAngle    ;
  wire  [7:0] Hoffset       ;
  wire  [7:0] Voffset       ;
  wire        lowerBound    ;
  wire        upperBound    ;
  reg  [31:0] ytemp         ;
  reg  [31:0] xtemp         ;
  reg  [15:0] azimuth_temp  ;
  reg  [15:0] elevation_temp;
  reg  [15:0] rtemp, rtemp2 ;

  always @(posedge clk) begin
    if(reset) begin
      //Deal with negative numbers
      azimuth_temp <= (i_azimuth[15:15] == 1'b1) ? (18000 - (-i_azimuth)) : (18000 + i_azimuth);
      elevation_temp <= (i_elevation[15:15] == 1'b1) ?  (maxAngle + (-i_elevation)): (maxAngle - i_elevation);
      rtemp <= i_range;
      //calculate coordinates
      xtemp <= (riWidth * (azimuth_temp + Hoffset) + 2);
      o_x <= xtemp/36000;

      ytemp <= (Voffset + elevation_temp) * riHeight;
      o_y <= (ytemp * multiplier) >> shiftFactor;

      rtemp2 <= rtemp;
      o_range <= rtemp2;
      o_wAddress <= o_x + o_y * (riWidth + 1);
    end
    else begin
      o_x <= 16'd0;
      o_y <= 8'd0;
      ytemp <= 32'd0;
      xtemp <= 32'd0;
      azimuth_temp <= 16'd0;
      elevation_temp <= 16'd0;
    end
  end

  //Verify if angle is within vertical limits
  assign lowerBound = ((-i_elevation) + maxAngle <= deltaAngle) ? 1'b1 : 1'b0;
  assign upperBound = (i_elevation[15:15] == 1'b0 && elevation_temp <= maxAngle) ? 1'b1 : 1'b0;
  assign o_validAngle = (lowerBound == 1'b1 || upperBound == 1'b1) ? 1'b1 : 1'b0;

  parameter_LUT parameter_LUT (
    .i_SensorType  (i_SensorSelect),
    .o_riWidth     (riWidth       ),
    .o_riHeight    (riHeight      ),
    .o_maxAngle    (maxAngle      ),
    .o_multiplier  (multiplier    ),
    .o_shiftFactor (shiftFactor   ),
    .o_deltaAngle  (deltaAngle    ),
    .o_Hoffset     (Hoffset       ),
    .o_Voffset     (Voffset       )
    );

endmodule
