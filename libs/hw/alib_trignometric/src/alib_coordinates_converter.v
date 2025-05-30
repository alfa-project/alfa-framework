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

// The calculation are based on the following equations:
// X=R*COS(vertical_angle)*SIN(horizontal_angle);
// Y=R*COS(vertical_angle)*COS(horizontal_angle);
// Z=R*SIN(vertical_angle);
// x,y,z values are in cm

module alib_coordinates_converter (
    input wire i_SYSTEM_clk,
    input wire i_SYSTEM_rst,

    // Extension_Interface
    output reg  [ 0:0] EXT_writeValid,
    input  wire [ 0:0] EXT_writeReady,
    output reg  [ 0:0] EXT_readReady,
    input  wire [ 0:0] EXT_readValid,
    input  wire [18:0] EXT_PCSize,
    output reg  [ 0:0] EXT_doneProcessing,
    output reg  [15:0] EXT_writeCustomField,
    input  wire [15:0] EXT_readCustomField,
    output reg  [18:0] EXT_writeID,
    output reg  [18:0] EXT_readID,
    input  wire [ 0:0] EXT_enable,
    output wire [31:0] EXT_status,

    // Cartesian Representation
    output wire [15:0] EXT_pointX,
    output wire [15:0] EXT_pointY,
    output wire [15:0] EXT_pointZ,

    input wire [15:0] EXT_pointAngleH,
    input wire [15:0] EXT_pointAngleV,
    input wire [15:0] EXT_pointRadius
);

  localparam integer divisor = 28147;  // ~= ((1/10000000000)*(2**48)) for x, y, and z decimal values;

  wire [16:0] value_x_y_1, value_x_2, value_y_2, value_z_1;
  wire signal_sin1, signal_sin2, signal_cos1, signal_cos2;

  wire [1:0] signal_x, signal_y;


  wire [48:0] value_x_cal;
  wire [48:0] value_y_cal;
  wire [48:0] value_z_cal;
  wire [96:0] value_x_cal_2;
  wire [96:0] value_y_cal_2;
  wire [96:0] value_z_cal_2;


  //X,Y,Z values in 16 bits
  assign value_x_cal_2 = (value_x_cal * divisor) >> 48;
  assign value_y_cal_2 = (value_y_cal * divisor) >> 48;
  assign value_z_cal_2 = (value_z_cal * divisor) >> 48;

  assign o_value_x[15:0] = value_x_cal_2[15:0];
  assign o_value_y[15:0] = value_y_cal_2[15:0];
  assign o_value_z[15:0] = value_z_cal_2[15:0];

  assign value_x_cal = (value_x_y_1 * value_x_2) * i_distance;
  assign value_y_cal = (value_x_y_1 * value_y_2) * i_distance;
  assign value_z_cal = value_z_1 * i_distance;

  //X,Y,Z signal bit defined
  assign signal_x = signal_sin1 + signal_cos1;
  assign signal_y = signal_cos1 + signal_cos2;
  assign o_value_x[16:16] = signal_x[0:0];
  assign o_value_y[16:16] = signal_y[0:0];
  assign o_value_z[16:16] = signal_sin2;


  //ALFA module that achieve 2 sin and 2 cos funtions
  alib_2sin2cos1acos alib_2sin2cos1acos (
      //sin 1
      .i_clk(i_clk),
      .i_rst(i_rst),
      .i_angle_sin1(i_angle_h),
      .o_value_sin1(value_x_2),
      .o_sig_sin1(signal_sin1),
      //sin 2
      .i_angle_sin2(i_angle_v),
      .o_value_sin2(value_z_1),
      .o_sig_sin2(signal_sin2),
      //cos 1
      .i_angle_cos1(i_angle_v),
      .o_value_cos1(value_x_y_1),
      .o_sig_cos1(signal_cos1),
      //cos 2
      .i_angle_cos2(i_angle_h),
      .o_value_cos2(value_y_2),
      .o_sig_cos2(signal_cos2),
      //acos NC
      .i_value_acos1(),
      .o_angle_acos2()
  );

endmodule
