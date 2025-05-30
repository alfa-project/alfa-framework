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

module alib_coordinates_converter_v2 #(
    parameter [0:0] c_type = 0  // 0 for cartesian_to_spherical, 1 for spherical_to_cartesian
) (
    input  wire [15:0] i_coordinate_1,
    input  wire [15:0] i_coordinate_2,
    input  wire [15:0] i_coordinate_3,
    output wire [15:0] o_coordinate_1,
    output wire [15:0] o_coordinate_2,
    output wire [15:0] o_coordinate_3
);


  // Cartesian to spherical conversion
  if (c_type == 0) begin : cartesian_to_spherical

    // Parameters for fixed-point representation
    parameter FIXED_POINT = 8;  // Number of fractional bits for fixed-point (1 cm = 256 units)

    // Intermediate values for sine and cosine
    wire signed [16:0] sin_theta, cos_theta, sin_phi, cos_phi;
    wire signed [31:0] r_sin_theta, r_sin_theta_cos_phi, r_sin_theta_sin_phi;
    wire [16:0] theta, phi;

    // Intermediate sign bits from the LUT
    wire sig_sin_theta, sig_cos_theta, sig_sin_phi, sig_cos_phi;

    // Instantiate the LUT module
    alib_2sin2cos1acos sin_cos_lut (
        .i_angle_sin1(theta),
        .o_value_sin1(sin_theta),
        .o_sig_sin1(sig_sin_theta),
        .i_angle_cos1(theta),
        .o_value_cos1(cos_theta),
        .o_sig_cos1(sig_cos_theta),
        .i_angle_sin2(phi),
        .o_value_sin2(sin_phi),
        .o_sig_sin2(sig_sin_phi),
        .i_angle_cos2(phi),
        .o_value_cos2(cos_phi),
        .o_sig_cos2(sig_cos_phi),
        .i_angle_acos1(i_coordinate_1),
        .o_angle_acos2(theta)
    );
    // Calculate intermediate values
    assign r_sin_theta = (i_coordinate_1 * sin_theta) >>> FIXED_POINT;  // r * sin(theta)
    assign r_sin_theta_cos_phi = (r_sin_theta * cos_phi) >>> FIXED_POINT; // r * sin(theta) * cos(phi)
    assign r_sin_theta_sin_phi = (r_sin_theta * sin_phi) >>> FIXED_POINT; // r * sin(theta) * sin(phi)
    assign o_coordinate_3 = (i_coordinate_1 * cos_theta) >>> FIXED_POINT;  // r * cos(theta)

    // Assign outputs (ensuring the results fit in 16-bit signed integers)
    assign o_coordinate_1 = r_sin_theta_cos_phi[15:0];
    assign o_coordinate_2 = r_sin_theta_sin_phi[15:0];
  end  // Spherical to cartesian conversion
  else begin : spherical_to_cartesian

    // Calculate the square of x, y, and z directly
    wire signed [33:0] square_x_y_z;
    assign square_x_y_z = $signed(
        {i_coordinate_1, 16'b0}
    ) * $signed(
        {i_coordinate_1, 16'b0}
    ) + $signed(
        {i_coordinate_2, 16'b0}
    ) * $signed(
        {i_coordinate_2, 16'b0}
    ) + $signed(
        {i_coordinate_3, 16'b0}
    ) * $signed(
        {i_coordinate_3, 16'b0}
    );

    // Instantiate the sqrt module to calculate the magnitude r
    alib_sqrt sqrt_r (
        .value(square_x_y_z),  // Pass the square of x, y, and z directly
        .sqrt_value(o_coordinate_1)  // Output of sqrt is the magnitude r
    );

    // Calculate the length of the projection of the vector onto the xy-plane
    wire        [15:0] xy_projection_length;

    // Calculate the square of x and y directly
    wire signed [31:0] square_x_y;
    assign square_x_y = $signed(
        {i_coordinate_1, 16'b0}
    ) * $signed(
        {i_coordinate_1, 16'b0}
    ) + $signed(
        {i_coordinate_2, 16'b0}
    ) * $signed(
        {i_coordinate_2, 16'b0}
    );

    // Instantiate the sqrt module to calculate the magnitude in the xy-plane
    alib_sqrt sqrt_x_y (
        .value(square_x_y),  // Pass the square of x and y directly
        .sqrt_value(xy_projection_length)  // Output of sqrt is the magnitude in the xy-plane
    );

    // Calculate theta (azimuthal angle) using atan2 function
    alib_atan atan_theta (
        .y(i_coordinate_2),
        .x(i_coordinate_1),
        .arctan_angle(o_coordinate_2)  // Output of atan is theta
    );

    // Calculate phi (polar angle) using atan2 function
    alib_atan atan_phi (
        .y(xy_projection_length),
        .x(i_coordinate_3),
        .arctan_angle(o_coordinate_3)  // Output of atan is phi
    );

  end

endmodule
