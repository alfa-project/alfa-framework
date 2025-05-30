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

module alib_code_lut (
    // Rank (0-255) as input
    input [7:0] i_rank,
    // Code as output (up to 16 bits for the longest code)
    output reg [15:0] o_code,
    // Length of code in bits (up to 5 for simplicity)     
    output reg [3:0] o_code_len
);

  reg [7:0] adjusted_rank;  // Temporary variable for bit extraction

  always @(*) begin
    // Default values
    o_code = 16'b0;
    o_code_len = 0;

    // Tier 1: Ranks 0-3, 3-bit codes
    if (i_rank < 4) begin
      o_code = {1'b0, i_rank[1:0]} & 16'h0007;  // 0 prefix + 2 bits for rank
      o_code_len = 3;

      // Tier 2: Ranks 4-11, 5-bit codes
    end else if (i_rank < 12) begin
      adjusted_rank = i_rank - 4;
      o_code = {2'b10, adjusted_rank[2:0]} & 16'h001F;  // 10 prefix + 3 bits
      o_code_len = 5;

      // Tier 3: Ranks 12-27, 7-bit codes
    end else if (i_rank < 28) begin
      adjusted_rank = i_rank - 12;
      o_code = {3'b110, adjusted_rank[3:0]} & 16'h007F;  // 110 prefix + 4 bits
      o_code_len = 7;

      // Tier 4: Ranks 28-91, 10-bit codes
    end else if (i_rank < 92) begin
      adjusted_rank = i_rank - 28;
      o_code = {4'b1110, adjusted_rank[5:0]} & 16'h03FF;  // 1110 prefix + 6 bits
      o_code_len = 10;

      // Tier 5: Ranks 92-255, 13-bit codes
    end else if (i_rank < 256) begin
      adjusted_rank = i_rank - 92;
      o_code = {5'b11110, adjusted_rank[7:0]} & 16'h1FFF;  // 11110 prefix + 8 bits
      o_code_len = 13;
    end
  end
endmodule
