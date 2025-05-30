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

module tb_alib_octree;

  // Parameters
  localparam [7:0] MAX_DEPTH = 10;
  localparam NUMBER_NODES = 1000;
  localparam signed [15:0] BB_MIN_X = -20000;
  localparam signed [15:0] BB_MIN_Y = -20000;
  localparam signed [15:0] BB_MIN_Z = -20000;
  localparam signed [15:0] BB_MAX_X = 20000;
  localparam signed [15:0] BB_MAX_Y = 20000;
  localparam signed [15:0] BB_MAX_Z = 20000;

  localparam SIZE_ID = $clog2(NUMBER_NODES);

  // Inputs
  reg               i_SYSTEM_clk;
  reg               i_SYSTEM_rst;
  reg               i_new_point;
  reg signed [15:0] i_point_x;
  reg signed [15:0] i_point_y;
  reg signed [15:0] i_point_z;
  reg               i_start_dfs;
  reg               i_reset_octree;

  // Outputs
  wire o_dfs_done, o_dfs_ready;
  wire o_new_point_processed, o_occupation_code_ready;
  reg        i_occupation_code_rd;
  wire [7:0] o_occupation_code;
  wire [2:0] o_occupation_code_last_valid_bits;
  wire       o_occupation_code_last_byte;


  // Instantiate the Unit Under Test (UUT)
  alib_octree #(
      .NUMBER_NODES(NUMBER_NODES),
      .PARALLEL_LEVEL(1),
      .COMPRESSION(1),
      .MAX_DEPTH(MAX_DEPTH),
      .BB_MIN_X(BB_MIN_X),
      .BB_MIN_Y(BB_MIN_Y),
      .BB_MIN_Z(BB_MIN_Z),
      .BB_MAX_X(BB_MAX_X),
      .BB_MAX_Y(BB_MAX_Y),
      .BB_MAX_Z(BB_MAX_Z)
  ) uut (
      .i_SYSTEM_clk(i_SYSTEM_clk),
      .i_SYSTEM_rst(i_SYSTEM_rst),
      .i_new_point(i_new_point),
      .i_point_x(i_point_x),
      .i_point_y(i_point_y),
      .i_point_z(i_point_z),
      .i_reset_octree(i_reset_octree),
      .i_occupation_code_rd(i_occupation_code_rd),
      .i_start_dfs(i_start_dfs),
      .o_dfs_ready(o_dfs_ready),
      .o_dfs_done(o_dfs_done),
      .o_all_nodes_occupied(o_all_nodes_occuppied),
      .o_new_point_ready(o_new_point_processed),
      .o_occupation_code(o_occupation_code),
      .o_occupation_code_ready(o_occupation_code_ready),
      .o_occupation_code_last_valid_bits(o_occupation_code_last_valid_bits),
      .o_occupation_code_last_byte(o_occupation_code_last_byte)
  );

  task decode_and_read_bs();
    reg [7:0] char_to_rank[0:255];  // Dictionary: Character to rank mapping
    reg [7:0] rank_to_char[0:255];  // Inverted dictionary: Rank to character mapping
    reg [31:0] bit_buffer;  // Buffer to store bits from the stream
    reg [5:0] bit_pointer;  // Tracks the position of the last valid bit in the buffer
    reg [7:0] decoded_char;  // Decoded character
    reg [7:0] rank;  // Decoded rank
    reg match_found;  // Indicates if decoding was successful
    integer i;

    begin
      bit_buffer  = 0;
      bit_pointer = 0;

      wait (o_occupation_code_ready);

      // Step 1: Extract the dictionary (char to rank mapping)
      $display("Reading dictionary...");
      for (i = 0; i < 256; i = i + 1) begin
        wait (o_occupation_code_ready);  // Wait until the next byte is ready
        #10;
        i_occupation_code_rd <= 1;  // Request to read a new byte
        #10;
        i_occupation_code_rd <= 0;
        #10;
        char_to_rank[i] = o_occupation_code;  // Populate the char-to-rank mapping
        $display("Character %0d has rank %0d", i, char_to_rank[i]);
      end
      $display("Finished reading dictionary.");

      // Step 2: Create the rank-to-char mapping
      for (i = 0; i < 256; i = i + 1) begin
        rank_to_char[char_to_rank[i]] = i;
      end

      // Step 3.a: Fill bitstream buffer
      for (i = 0; i < 2; i = i + 1) begin
        wait (o_occupation_code_ready);
        #10;
        i_occupation_code_rd <= 1;  // Request new byte
        #10;
        i_occupation_code_rd <= 0;
        #10;
        // Append new byte to buffer
        bit_buffer  = bit_buffer | (o_occupation_code << (24 - bit_pointer));
        bit_pointer = bit_pointer + 8;  // Update pointer
        //$display("Buffer refilled:", bit_pointer);
      end

      // Step 3: Decode the compressed bitstream
      $display("Decoding compressed bitstream...");
      while (o_occupation_code_last_byte == 0) begin
        // Fill buffer if needed
        if (bit_pointer < 24) begin
          $display("Waiting for new byte...");
          wait (o_occupation_code_ready);
          #10;
          i_occupation_code_rd <= 1;  // Request new byte
          #10;
          i_occupation_code_rd <= 0;
          #10;
          // Append new byte to buffer
          bit_buffer  = bit_buffer | (o_occupation_code << (24 - bit_pointer));
          bit_pointer = bit_pointer + 8;  // Update pointer
          $display("Buffer refilled: Buffer = %b, Pointer = %0d", bit_buffer, bit_pointer);
          #10;
        end

        // Decode the next rank
        match_found = 0;

        if (bit_pointer >= 3) begin
          // Tier 1: Ranks 0-3
          if (bit_buffer[31] == 0) begin
            rank = bit_buffer[30:29];
            bit_buffer = bit_buffer << 3;  // Consume 3 bits
            bit_pointer = bit_pointer - 3;
            match_found = 1;
          end  // Tier 2: Ranks 4-11
          else if (bit_pointer >= 5 && bit_buffer[31:30] == 2'b10) begin
            rank = 4 + bit_buffer[29:27];
            bit_buffer = bit_buffer << 5;  // Consume 5 bits
            bit_pointer = bit_pointer - 5;
            match_found = 1;
          end  // Tier 3: Ranks 12-27
          else if (bit_pointer >= 7 && bit_buffer[31:29] == 3'b110) begin
            rank = 12 + bit_buffer[28:25];
            bit_buffer = bit_buffer << 7;  // Consume 7 bits
            bit_pointer = bit_pointer - 7;
            match_found = 1;
          end  // Tier 4: Ranks 28-91
          else if (bit_pointer >= 10 && bit_buffer[31:28] == 4'b1110) begin
            rank = 28 + bit_buffer[27:22];
            bit_buffer = bit_buffer << 10;  // Consume 10 bits
            bit_pointer = bit_pointer - 10;
            match_found = 1;
          end  // Tier 5: Ranks 92-255
          else if (bit_pointer >= 13 && bit_buffer[31:27] == 5'b11110) begin
            rank = 92 + bit_buffer[26:19];
            bit_buffer = bit_buffer << 13;  // Consume 13 bits
            bit_pointer = bit_pointer - 13;
            match_found = 1;
          end
        end

        #10;

        if (match_found) begin
          decoded_char = rank_to_char[rank];  // Map rank to original character
          $display("Decoded character: %0d, Remaining Buffer = %b, Remaining bits = ",
                   decoded_char, bit_buffer, bit_pointer);
        end else begin
          $display("Error: Failed to decode bitstream!");
          disable decode_and_read_bs;  // Exit the task on error
        end
        #10;
      end

      if (o_occupation_code_last_valid_bits != 0)
        bit_pointer = bit_pointer - 8 + o_occupation_code_last_valid_bits;

      // Write the last data that is already in the buffer
      $display("Emptying the buffer...");
      while (bit_pointer > 0) begin
        if (bit_pointer >= 3) begin
          // Tier 1: Ranks 0-3
          if (bit_buffer[31] == 0) begin
            rank = bit_buffer[30:29];
            bit_buffer = bit_buffer << 3;  // Consume 3 bits
            bit_pointer = bit_pointer - 3;
            match_found = 1;
          end  // Tier 2: Ranks 4-11
          else if (bit_pointer >= 5 && bit_buffer[31:30] == 2'b10) begin
            rank = 4 + bit_buffer[29:27];
            bit_buffer = bit_buffer << 5;  // Consume 5 bits
            bit_pointer = bit_pointer - 5;
            match_found = 1;
          end  // Tier 3: Ranks 12-27
          else if (bit_pointer >= 7 && bit_buffer[31:29] == 3'b110) begin
            rank = 12 + bit_buffer[28:25];
            bit_buffer = bit_buffer << 7;  // Consume 7 bits
            bit_pointer = bit_pointer - 7;
            match_found = 1;
          end  // Tier 4: Ranks 28-91
          else if (bit_pointer >= 10 && bit_buffer[31:28] == 4'b1110) begin
            rank = 28 + bit_buffer[27:22];
            bit_buffer = bit_buffer << 10;  // Consume 10 bits
            bit_pointer = bit_pointer - 10;
            match_found = 1;
          end  // Tier 5: Ranks 92-255
          else if (bit_pointer >= 13 && bit_buffer[31:27] == 5'b11110) begin
            rank = 92 + bit_buffer[26:19];
            bit_buffer = bit_buffer << 13;  // Consume 13 bits
            bit_pointer = bit_pointer - 13;
            match_found = 1;
          end
        end

        #10;

        if (match_found) begin
          decoded_char = rank_to_char[rank];  // Map rank to original character
          $display("Decoded character: %0d, Remaining Buffer = %b, Remaining bits = ",
                   decoded_char, bit_buffer, bit_pointer);
        end else begin
          $display("Error: Failed to decode bitstream!");
          disable decode_and_read_bs;  // Exit the task on error
        end
      end

      $display("Finished decoding bitstream.");
    end
  endtask


  // Task to generate random values and simulate adding new points
  task generate_random_points(input integer num_points);
    integer i;  // Loop variable
    begin
      for (i = 0; i < num_points; i = i + 1) begin
        // Generate random values
        i_point_x = $random % 16384;  // Generates a random value between -16384 and 16384
        i_point_y = $random % 16384;  // Generates a random value between -16384 and 16384
        i_point_z = $random % 16384;  // Generates a random value between -16384 and 16384

        // Output the generated values to the console
        $display("Generated point %0d: (x: %d, y: %d, z: %d)", i, i_point_x, i_point_y, i_point_z);

        // Simulate adding a new point to the octree
        #10;
        i_new_point = 1;
        #10;
        i_new_point = 0;
        #10;
        wait (o_new_point_processed);
        #10;
      end
    end
  endtask

  // Clock generation
  initial begin
    i_SYSTEM_clk = 1;
    forever #5 i_SYSTEM_clk = ~i_SYSTEM_clk;  // 100MHz clock
  end

  // Test stimuli
  initial begin
    // Initialize Inputs
    i_SYSTEM_rst = 0;
    i_new_point = 0;
    i_point_x = 0;
    i_point_y = 0;
    i_point_z = 0;
    i_start_dfs = 0;
    i_occupation_code_rd <= 0;

    // Wait for the global reset
    #100;
    // Release reset
    i_SYSTEM_rst = 1;
    #10;
    i_reset_octree = 1;
    #10;
    i_reset_octree = 0;
    #100;
    // Add a new points to the octree
    generate_random_points(20);

    #10;
    wait (o_dfs_ready);
    #10;
    // Start depth-first search (DFS)
    i_start_dfs = 1;
    #10;
    i_start_dfs = 0;
    #10;
    // Wait for DFS to complete
    wait (o_dfs_done);
    decode_and_read_bs();

    #100;
    i_reset_octree = 1;
    #10;
    i_reset_octree = 0;
    #100;
    // Add a new points to the octree
    generate_random_points(20);

    #10;
    wait (o_dfs_ready);
    #10;
    // Start depth-first search (DFS)
    i_start_dfs = 1;
    #10;
    i_start_dfs = 0;
    #10;
    // Wait for DFS to complete
    wait (o_dfs_done);
    decode_and_read_bs();
    // End of test
    #1000;
    $finish;
  end

endmodule
