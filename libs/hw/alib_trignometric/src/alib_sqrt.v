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

module alib_sqrt (
    input  wire [32:0] value,
    output wire [16:0] sqrt_value
);

  // LUT size
  parameter LUT_SIZE = 4096;

  // Square root LUT
  reg [16:0] lut[0:LUT_SIZE-1];

  // Generate square root values for LUT
  initial begin
    integer i;
    for (i = 0; i < LUT_SIZE; i = i + 1) begin
      lut[i] = $sqrt(i * (2 ** 32) / LUT_SIZE);
    end
  end

  // Newton-Raphson function
  function [16:0] newton_raphson;
    input [16:0] value;
    input [16:0] guess;
    integer        i;
    reg     [16:0] x;
    reg     [31:0] x_sq;

    begin
      x = guess;
      for (i = 0; i < 2; i = i + 1) begin  // Using 2 iterations as an example
        x_sq = x * x;
        x = (x + (value / x)) >> 1;
      end
      newton_raphson = x;
    end
  endfunction

  wire [16:0] initial_guess;

  // Get the initial guess from the LUT
  assign initial_guess = (value > LUT_SIZE) ? 17'd0 : lut[value];

  // Refine the initial guess using Newton-Raphson method
  assign sqrt_value = newton_raphson(value, initial_guess);

endmodule

