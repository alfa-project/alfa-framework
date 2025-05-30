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

module tb_code_lut;

  // Inputs
  reg  [ 7:0] i_rank;

  // Outputs
  wire [15:0] o_code;
  wire [ 3:0] o_code_len;

  // Instantiate the code_lut module
  alib_code_lut uut (
      .i_rank(i_rank),
      .o_code(o_code),
      .o_code_len(o_code_len)
  );

  // Test sequence
  initial begin
    // Display header
    $display("Rank | Code             | Code Length");
    $display("------------------------------------");

    // Test different ranks and display the output
    i_rank = 8'd0;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd3;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd4;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd11;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd12;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd27;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd28;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd91;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd92;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);
    i_rank = 8'd255;
    #10 $display("%3d  | %b | %2d", i_rank, o_code, o_code_len);

    // Finish simulation
    #10 $stop;
  end

endmodule

