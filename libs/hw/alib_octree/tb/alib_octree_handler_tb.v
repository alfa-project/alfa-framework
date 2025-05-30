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

module tb_alib_octree_handler;

  // Parameters
  wire [7:0] MAX_DEPTH = 10;
  localparam NUMBER_NODES = 200;
  wire signed [                15:0] BB_MIN_X = -20000;
  wire signed [                15:0] BB_MIN_Y = -20000;
  wire signed [                15:0] BB_MIN_Z = -20000;
  wire signed [                15:0] BB_MAX_X = 20000;
  wire signed [                15:0] BB_MAX_Y = 20000;
  wire signed [                15:0] BB_MAX_Z = 20000;

  // Inputs
  reg                                i_SYSTEM_clk;
  reg                                i_SYSTEM_rst;
  reg                                i_new_point;
  reg signed  [                15:0] i_point_x;
  reg signed  [                15:0] i_point_y;
  reg signed  [                15:0] i_point_z;
  reg                                i_start_dfs;
  reg                                i_reset_octree;

  // Outputs
  wire                               o_dfs_done;
  wire                               o_new_point_processed;
  wire        [(8*NUMBER_NODES)-1:0] o_occupation_code;
  wire        [                31:0] o_occupation_code_size_bytes;


  // Instantiate the Unit Under Test (UUT)
  alib_octree_handler #(
      .NUMBER_NODES(NUMBER_NODES)
  ) uut (
      .i_SYSTEM_clk(i_SYSTEM_clk),
      .i_SYSTEM_rst(i_SYSTEM_rst),
      .i_MAX_DEPTH(MAX_DEPTH),
      .i_BB_MIN_X(BB_MIN_X),
      .i_BB_MIN_Y(BB_MIN_Y),
      .i_BB_MIN_Z(BB_MIN_Z),
      .i_BB_MAX_X(BB_MAX_X),
      .i_BB_MAX_Y(BB_MAX_Y),
      .i_BB_MAX_Z(BB_MAX_Z),
      .i_new_point(i_new_point),
      .i_point_x(i_point_x),
      .i_point_y(i_point_y),
      .i_point_z(i_point_z),
      .i_reset_octree(i_reset_octree),
      .i_start_dfs(i_start_dfs),
      .o_dfs_done(o_dfs_done),
      .o_all_nodes_occuppied(o_all_nodes_occuppied),
      .o_new_point_processed(o_new_point_processed),
      .o_occupation_code(o_occupation_code),
      .o_occupation_code_size_bytes(o_occupation_code_size_bytes)
  );

  // Task to generate random values and simulate adding new points
  task generate_random_points(input integer num_points);
    integer i;  // Loop variable
    begin
      for (i = 0; i < num_points; i = i + 1) begin
        // Generate random values
        i_point_x = $random % 32768;  // Generates a random value between -32768 and 32767
        i_point_y = $random % 32768;  // Generates a random value between -32768 and 32767
        i_point_z = $random % 32768;  // Generates a random value between -32768 and 32767

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
    generate_random_points(10);

    #10;
    // Start depth-first search (DFS)
    i_start_dfs = 1;
    #10;
    i_start_dfs = 0;
    #10
    // Wait for DFS to complete
    wait (o_dfs_done);

    #10;
    i_reset_octree = 1;
    #10;
    i_reset_octree = 0;
    #100;
    // Add a new points to the octree
    generate_random_points(10);

    #10;
    // Start depth-first search (DFS)
    i_start_dfs = 1;
    #10;
    i_start_dfs = 0;
    #10;
    // Wait for DFS to complete
    wait (o_dfs_done);

    // Check results
    // Add your checks or additional stimuli here

    // End of test
    #1000;
    $finish;
  end

endmodule
