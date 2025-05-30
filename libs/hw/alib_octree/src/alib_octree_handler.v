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

`define B_N_MODES 5

`define TURN_ALL_ENABLES_OFF \
  enable_parent_update <= 0; \
  enable_occupation_code_update <= 0; \
  enable_child_update <= 0; \

module alib_octree_handler #(
    parameter [31:0] NUMBER_NODES = 100000,
    parameter [31:0] SIZE_ID = $clog2(NUMBER_NODES),
    parameter signed [15:0] BB_MIN_X = 0,
    parameter signed [15:0] BB_MIN_Y = 0,
    parameter signed [15:0] BB_MIN_Z = 0,
    parameter signed [15:0] BB_MAX_X = 0,
    parameter signed [15:0] BB_MAX_Y = 0,
    parameter signed [15:0] BB_MAX_Z = 0,
    parameter [3:0] MAX_DEPTH = 8

) (
    input wire i_SYSTEM_clk,
    input wire i_SYSTEM_rst,
    input wire i_new_point,
    input wire signed [15:0] i_point_x,
    input wire signed [15:0] i_point_y,
    input wire signed [15:0] i_point_z,
    input wire i_reset_octree,
    input wire i_start_dfs,
    input wire [SIZE_ID-1:0] i_occupation_code_id,
    output reg o_dfs_done,
    output wire o_dfs_ready,
    output reg o_new_point_ready,
    output wire o_all_nodes_occupied,
    output wire [7:0] o_occupation_code_insertion,
    output wire o_occupation_code_insertion_enable,
    output wire [7:0] o_occupation_code,
    output wire [31:0] o_occupation_code_size_bytes
);

  localparam SIZE_DEPTH = $clog2(MAX_DEPTH);

  // System Initialization and Idle States
  localparam [`B_N_MODES-1:0] RESET = `B_N_MODES'b00000,  // System reset state
  IDLE = `B_N_MODES'b00001,  // System idle state

  // Node Navigation States
  JUMP_CHILD = `B_N_MODES'b00010,  // Jump to child node
  JUMP_PARENT = `B_N_MODES'b00100,  // Jump to parent node
  NODE_INIT = `B_N_MODES'b00101,  // Initialize node
  NODE_READ = `B_N_MODES'b00110,  // Read node data
  UPDATE_BRANCH = `B_N_MODES'b00011,  // Update node data

  // Point Addition States
  NEW_POINT = `B_N_MODES'b00111,  // Start adding a new point
  END_NEW_POINT = `B_N_MODES'b01000,  // Finish adding a new point

  // Depth-First Search (DFS) States
  DFS_FIRST_ENTRY = `B_N_MODES'b01001,  // First entry into DFS
  DFS_NEXT_VALID_BRANCH = `B_N_MODES'b01010,  // Find next valid branch in DFS
  DFS_JUMP_ROOT = `B_N_MODES'b01011,  // Jump to root node in DFS
  DFS_CLEAR_BRANCH = `B_N_MODES'b01100,  // Clear branch in DFS
  DFS_JUMP_PARENT = `B_N_MODES'b01101,  // DFS jump to parent node
  DFS_JUMP_CHILD = `B_N_MODES'b01110,  // DFS jump to child node
  DFS_READ = `B_N_MODES'b01111,  // Read node during DFS
  END_DFS = `B_N_MODES'b10000,  // End DFS process

  // Wait States
  WAIT_ONE_CYCLE = `B_N_MODES'b10001,  // Wait for one cycle
  WAIT_TWO_CYCLES = `B_N_MODES'b10010,  // Wait for two cycles

  // Error State
  ERROR = `B_N_MODES'b11111;  // Error state indicator

  // Error and Misc signals
  reg [SIZE_ID-1:0] id, node_counter;
  reg [SIZE_DEPTH-1:0] depth;
  reg [`B_N_MODES-1:0] handler_mode;
  reg [`B_N_MODES-1:0] next_handler_mode_after_wait;
  reg first_read, inside_bb_found;
  reg  dfs_reg;

  wire is_IDLE;

  assign o_all_nodes_occupied = (node_counter == NUMBER_NODES) ? 1'b1 : 1'b0;
  assign is_IDLE = handler_mode == IDLE ? 1'b1 : 1'b0;

  // Function to calculate the next valid branch increments
  function [2:0] calculate_next_valid_branch;
    input [7:0] occupation_code;
    integer i;  // Intermediate variable to iterate over the occupation_code bits
    reg     found;  // Flag to indicate if a valid branch has been found
    begin
      calculate_next_valid_branch = 3'b000;  // Default value if no valid branch is found
      found = 1'b0;  // Initialize the flag to false
      for (i = 0; i <= 7 && !found; i = i + 1) begin
        if (occupation_code[i] == 1) begin
          calculate_next_valid_branch = i;
          found = 1'b1;  // Set the flag to true to exit the loop
        end
      end
    end
  endfunction

  // Calculate the half voxel size for each depth
  localparam signed [16:0] HALF_VOXEL_SIZE_ROOT_X = (BB_MAX_X - BB_MIN_X) / 2;
  localparam signed [16:0] HALF_VOXEL_SIZE_ROOT_Y = (BB_MAX_Y - BB_MIN_Y) / 2;
  localparam signed [16:0] HALF_VOXEL_SIZE_ROOT_Z = (BB_MAX_Z - BB_MIN_Z) / 2;
  wire signed [16:0] voxel_size_x[0:MAX_DEPTH-1];
  wire signed [16:0] voxel_size_y[0:MAX_DEPTH-1];
  wire signed [16:0] voxel_size_z[0:MAX_DEPTH-1];
  generate
    genvar i;

    for (i = 0; i < MAX_DEPTH; i = i + 1) begin
      assign voxel_size_x[i] = (HALF_VOXEL_SIZE_ROOT_X >> i);
      assign voxel_size_y[i] = (HALF_VOXEL_SIZE_ROOT_Y >> i);
      assign voxel_size_z[i] = (HALF_VOXEL_SIZE_ROOT_Z >> i);
    end
  endgenerate

  // Circular FIFO to pipeline input points
  wire signed [15:0] point_x, point_y, point_z;
  wire fifo_is_inside, fifo_is_inside_x, fifo_is_inside_y, fifo_is_inside_z;
  wire fifo_full, fifo_empty;
  reg wr_en, rd_en;
  reg new_point, new_point_processed;

  assign o_dfs_ready = fifo_empty && !new_point && !rd_en && is_IDLE && o_new_point_ready? dfs_reg : 0;
  assign fifo_is_inside_x = ((i_point_x >= BB_MIN_X) && (i_point_x < BB_MAX_X)) ? 1'b1 : 1'b0;
  assign fifo_is_inside_y = ((i_point_y >= BB_MIN_Y) && (i_point_y < BB_MAX_Y)) ? 1'b1 : 1'b0;
  assign fifo_is_inside_z = ((i_point_z >= BB_MIN_Z) && (i_point_z < BB_MAX_Z)) ? 1'b1 : 1'b0;

  assign fifo_is_inside = (fifo_is_inside_x && fifo_is_inside_y && fifo_is_inside_z) ? 1'b1 : 1'b0;

  alib_points_circular_fifo #(10) u_fifo (
      .clk(i_SYSTEM_clk),
      .rst(i_SYSTEM_rst && ~i_reset_octree),
      .point_x_in(i_point_x),
      .point_y_in(i_point_y),
      .point_z_in(i_point_z),
      .wr_en(wr_en),
      .rd_en(rd_en),
      .point_x_out(point_x),
      .point_y_out(point_y),
      .point_z_out(point_z),
      .full(fifo_full),
      .empty(fifo_empty)
  );

  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst && ~i_reset_octree) begin
      if (fifo_is_inside && i_new_point && !fifo_full) wr_en <= 1;
      else wr_en <= 0;

      if (rd_en) rd_en <= 0;
      else if (!fifo_empty && is_IDLE && !new_point && (new_point_processed || first_read))
        rd_en <= 1;

      if (rd_en) new_point <= 1;
      else if (new_point_processed) new_point <= 0;
      else new_point <= new_point;

    end else begin
      wr_en <= 0;
      rd_en <= 0;
      new_point <= 0;
    end
  end

  // Small patch to avoid starting the DFS before the ready signal is still to be asserted
  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst && ~i_reset_octree) begin
      if (i_new_point) o_new_point_ready <= 0;
      else if (!fifo_full) o_new_point_ready <= 1;
    end else begin
      o_new_point_ready <= 0;
    end
  end

  // Bounding box calculation
  reg signed [15:0] centroid_x_reg, centroid_y_reg, centroid_z_reg;
  wire signed [15:0] bb_min_x, bb_min_y, bb_min_z, bb_max_x, bb_max_y, bb_max_z;
  assign bb_min_x = centroid_x_reg - voxel_size_x[depth];
  assign bb_min_y = centroid_y_reg - voxel_size_y[depth];
  assign bb_min_z = centroid_z_reg - voxel_size_z[depth];
  assign bb_max_x = centroid_x_reg + voxel_size_x[depth];
  assign bb_max_y = centroid_y_reg + voxel_size_y[depth];
  assign bb_max_z = centroid_z_reg + voxel_size_z[depth];

  // Check if the point is inside the bounding box
  wire is_inside_bb, inside_x, inside_y, inside_z;
  assign inside_x = ((point_x >= bb_min_x) && (point_x < bb_max_x)) ? 1'b1 : 1'b0;
  assign inside_y = ((point_y >= bb_min_y) && (point_y < bb_max_y)) ? 1'b1 : 1'b0;
  assign inside_z = ((point_z >= bb_min_z) && (point_z < bb_max_z)) ? 1'b1 : 1'b0;
  assign is_inside_bb = (inside_x & inside_y & inside_z) ? 1'b1 : 1'b0;

  // Parent storage, index calculation and node init
  reg  [SIZE_ID-1:0] parent_id_reg;
  reg  [        2:0] c_index_parent;
  reg                enable_parent_update;
  wire [        2:0] p_index;
  wire [SIZE_ID-1:0] parent_id;
  wire [2+SIZE_ID:0] node_configuration_i, node_configuration_o;

  assign node_configuration_i[2:0] = c_index_parent;
  assign node_configuration_i[SIZE_ID+2:3] = parent_id_reg;
  assign p_index = node_configuration_o[2:0];
  assign parent_id = node_configuration_o[SIZE_ID+2:3];

  alib_dram #(
      .DATA_WIDTH(3 + SIZE_ID),
      .DEPTH(NUMBER_NODES)
  ) parent_conf_bram (
      .clk (i_SYSTEM_clk),
      .rst (i_SYSTEM_rst && ~i_reset_octree),
      .addr(id),
      .din (node_configuration_i),
      .we  (enable_parent_update),
      .dout(node_configuration_o)
  );

  // Children storage, index calculation and occupation_code
  reg  [SIZE_ID-1:0] child_id_reg        [0:7];
  reg                enable_child_update;
  reg  [SIZE_ID-1:0] dfs_child_to_jump;
  wire [SIZE_ID-1:0] child_id;
  wire [SIZE_ID-1:0] child_id_b          [0:7];
  wire [        2:0] c_index;
  wire [        7:0] occupation_code;
  wire [(SIZE_ID*8)-1:0] child_id_reg_grouped, child_id_b_grouped;

  assign c_index[0:0] = (point_x >= centroid_x_reg) ? 1 : 0;
  assign c_index[1:1] = (point_y >= centroid_y_reg) ? 1 : 0;
  assign c_index[2:2] = (point_z >= centroid_z_reg) ? 1 : 0;
  assign child_id = child_id_b[c_index];

  assign child_id_reg_grouped = {
    child_id_reg[7],
    child_id_reg[6],
    child_id_reg[5],
    child_id_reg[4],
    child_id_reg[3],
    child_id_reg[2],
    child_id_reg[1],
    child_id_reg[0]
  };

  generate
    for (i = 0; i < 8; i = i + 1) begin
      assign child_id_b[i] = child_id_b_grouped[(i+1)*SIZE_ID-1:i*SIZE_ID];
      assign occupation_code[i:i] = child_id_b[i] == 0 ? 1'b0 : 1'b1;
    end
  endgenerate

  alib_uram #(
      .DATA_WIDTH(SIZE_ID * 8),
      .DEPTH(NUMBER_NODES)
  ) child_id_bram (
      .clk (i_SYSTEM_clk),
      .rst (i_SYSTEM_rst && ~i_reset_octree),
      .addr(id),
      .din (child_id_reg_grouped),
      .we  (enable_child_update),
      .dout(child_id_b_grouped)
  );

  // Check if the node is a leaf, root or not initialized
  wire is_leaf, is_root, is_child_not_init, is_dfs_empty, is_tree_empty;
  reg init_child;
  assign is_leaf = (depth == MAX_DEPTH - 1) ? 1'b1 : 1'b0;
  assign is_root = (id == 1) ? 1'b1 : 1'b0;
  assign is_child_not_init = child_id == 0 ? 1'b1 : 1'b0;
  assign is_dfs_empty = occupation_code == 0 ? 1'b1 : 1'b0;
  assign is_tree_empty = node_counter == 2 ? 1'b1 : 1'b0;

  // Occupation code calculation and storage
  reg  [SIZE_ID-1:0] occupation_counter;
  reg  [        7:0] occupation_code_reg;
  reg                enable_occupation_code_update;
  wire [        7:0] o_occupation_code_temp;
  reg  [SIZE_ID-1:0] occupation_code_addr;

  assign o_occupation_code_size_bytes = {{(32 - SIZE_ID) {1'b0}}, occupation_counter};
  assign o_occupation_code = occupation_counter > 0 ? o_occupation_code_temp : 0;

  // Ranking system for compression
  assign o_occupation_code_insertion_enable = enable_occupation_code_update;
  assign o_occupation_code_insertion = occupation_code_reg;

  alib_bram_r_w #(
      .DATA_WIDTH(8),
      .DEPTH(NUMBER_NODES)
  ) occupation_code_dfs_bram (
      .clk(i_SYSTEM_clk),
      .rst(i_SYSTEM_rst && ~i_reset_octree),
      .addra(occupation_code_addr),
      .addrb(i_occupation_code_id),
      .din(occupation_code_reg),
      .we(enable_occupation_code_update),
      .dout(o_occupation_code_temp)
  );

  integer l;

  // State Machine (signals and logic)
  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst && ~i_reset_octree) begin
      case (handler_mode)

        ERROR: begin
          // Turn off all enables
          `TURN_ALL_ENABLES_OFF
        end

        RESET: begin
          // Define root centroids, depth and id
          centroid_x_reg <= (BB_MAX_X + BB_MIN_X) / 2;
          centroid_y_reg <= (BB_MAX_Y + BB_MIN_Y) / 2;
          centroid_z_reg <= (BB_MAX_Z + BB_MIN_Z) / 2;
          depth <= 0;
          id <= 1;
          inside_bb_found <= 0;

          // Set the next node id to 2
          node_counter <= 2;

          // Reset all output signals
          new_point_processed <= 0;
          first_read <= 1;
          o_dfs_done <= 0;
          dfs_reg <= 1;

          // Reset bram input registers
          parent_id_reg <= 0;
          c_index_parent <= 0;
          occupation_code_addr <= 0;
          occupation_code_reg <= 0;
          occupation_counter <= 0;
          for (l = 0; l < 8; l = l + 1) child_id_reg[l] <= 0;

          // Clear next handler mode after wait

          init_child <= 0;
          dfs_child_to_jump <= 0;
          //Turn off all enables
          `TURN_ALL_ENABLES_OFF

        end

        IDLE: begin
          `TURN_ALL_ENABLES_OFF
        end

        NODE_INIT: begin
          enable_parent_update <= 1;
          for (l = 0; l < 8; l = l + 1) begin
            child_id_reg[l] <= 0;
          end
          enable_child_update <= 1;
          enable_occupation_code_update <= 0;
          init_child <= 0;

        end

        NEW_POINT: begin
          first_read <= 0;
          dfs_reg <= 0;
          o_dfs_done <= 0;
          new_point_processed <= 0;
          `TURN_ALL_ENABLES_OFF
        end

        END_NEW_POINT: begin
          new_point_processed <= 1;
          inside_bb_found <= 0;
          if (fifo_empty) dfs_reg <= 1;
          `TURN_ALL_ENABLES_OFF
        end

        JUMP_CHILD: begin
          depth <= depth + 1;
          id <= child_id;
          parent_id_reg <= id;
          inside_bb_found <= 1;
          centroid_x_reg <= centroid_x_reg + (c_index[0:0] ? voxel_size_x[depth+1] : -voxel_size_x[depth+1]);
          centroid_y_reg <= centroid_y_reg + (c_index[1:1] ? voxel_size_y[depth+1] : -voxel_size_y[depth+1]);
          centroid_z_reg <= centroid_z_reg + (c_index[2:2] ? voxel_size_z[depth+1] : -voxel_size_z[depth+1]);
          c_index_parent[0:0] <= c_index[0:0];
          c_index_parent[1:1] <= c_index[1:1];
          c_index_parent[2:2] <= c_index[2:2];
          `TURN_ALL_ENABLES_OFF
        end

        JUMP_PARENT: begin
          depth <= depth - 1;
          id <= parent_id;
          centroid_x_reg <= centroid_x_reg + (p_index[0:0] ? -voxel_size_x[depth] : voxel_size_x[depth]);
          centroid_y_reg <= centroid_y_reg + (p_index[1:1] ? -voxel_size_y[depth] : voxel_size_y[depth]);
          centroid_z_reg <= centroid_z_reg + (p_index[2:2] ? -voxel_size_z[depth] : voxel_size_z[depth]);
          `TURN_ALL_ENABLES_OFF
        end

        UPDATE_BRANCH: begin
          for (l = 0; l < 8; l = l + 1) begin
            if (l == c_index) child_id_reg[l] <= is_leaf ? id : node_counter;
            else child_id_reg[l] <= child_id_b[l];
          end
          enable_child_update <= 1;
          node_counter <= is_leaf ? node_counter : node_counter + 1;
          enable_parent_update <= 0;
          enable_occupation_code_update <= 0;
          init_child <= is_leaf ? 0 : 1;
        end

        DFS_FIRST_ENTRY: begin
          occupation_code_reg <= occupation_code;
          occupation_code_addr <= occupation_counter;
          occupation_counter <= occupation_counter + 1;
          enable_occupation_code_update <= 1;
        end

        DFS_NEXT_VALID_BRANCH: begin
          dfs_child_to_jump <= child_id_b[calculate_next_valid_branch(occupation_code)];
          enable_occupation_code_update <= 0;
        end

        DFS_CLEAR_BRANCH: begin
          for (l = 0; l < 8; l = l + 1) begin
            if (l == calculate_next_valid_branch(occupation_code)) child_id_reg[l] <= 0;
            else child_id_reg[l] <= child_id_b[l];
          end
          enable_child_update <= 1;
        end

        DFS_JUMP_CHILD: begin
          id <= dfs_child_to_jump;
          depth <= depth + 1;
          `TURN_ALL_ENABLES_OFF
        end

        DFS_JUMP_PARENT: begin
          id <= parent_id;
          depth <= depth - 1;
          `TURN_ALL_ENABLES_OFF
        end

        DFS_READ: begin
          `TURN_ALL_ENABLES_OFF
        end

        DFS_JUMP_ROOT: begin
          id <= 1;
          depth <= 0;
          occupation_counter <= 0;
          dfs_reg <= 0;
          o_dfs_done <= 0;
          `TURN_ALL_ENABLES_OFF
        end

        END_DFS: begin
          o_dfs_done <= 1;
          `TURN_ALL_ENABLES_OFF
        end

        WAIT_ONE_CYCLE: begin
          `TURN_ALL_ENABLES_OFF
        end

        WAIT_TWO_CYCLES: begin
          `TURN_ALL_ENABLES_OFF
        end

        default: begin
          id <= id;
          o_dfs_done <= o_dfs_done;
          dfs_reg <= dfs_reg;
        end

      endcase
    end else begin
      // Define root centroids, depth and id
      centroid_x_reg <= (BB_MAX_X + BB_MIN_X) / 2;
      centroid_y_reg <= (BB_MAX_Y + BB_MIN_Y) / 2;
      centroid_z_reg <= (BB_MAX_Z + BB_MIN_Z) / 2;
      depth <= 0;
      id <= 1;
      inside_bb_found <= 0;

      // Set the next node id to 2
      node_counter <= 2;

      // Reset all output signals
      new_point_processed <= 0;
      o_dfs_done <= 0;
      dfs_reg <= 1;

      // Reset bram input registers
      parent_id_reg <= 0;
      c_index_parent <= 0;
      dfs_child_to_jump <= 0;
      occupation_counter <= 0;
      for (l = 0; l < 8; l = l + 1) child_id_reg[l] <= 0;
      occupation_code_addr <= 0;
      occupation_code_reg  <= 0;

      //Turn off all enables
      `TURN_ALL_ENABLES_OFF
    end
  end

  // State Machine (next state logic)
  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst && ~i_reset_octree) begin
      case (handler_mode)

        RESET: begin
          handler_mode <= NODE_INIT;
          next_handler_mode_after_wait <= RESET;
        end

        IDLE: begin
          if (o_all_nodes_occupied) handler_mode <= ERROR;
          else if (new_point) handler_mode <= NEW_POINT;
          else if (i_start_dfs && dfs_reg) handler_mode <= DFS_JUMP_ROOT;
          else handler_mode <= IDLE;
        end

        NODE_INIT: begin
          if (is_root) next_handler_mode_after_wait <= IDLE;
          else next_handler_mode_after_wait <= UPDATE_BRANCH;
          handler_mode <= WAIT_TWO_CYCLES;
        end

        NEW_POINT: begin
          handler_mode <= NODE_READ;
        end

        UPDATE_BRANCH: begin
          if (o_all_nodes_occupied) handler_mode <= ERROR;
          else if (is_leaf) next_handler_mode_after_wait <= END_NEW_POINT;
          else next_handler_mode_after_wait <= JUMP_CHILD;
          handler_mode <= WAIT_TWO_CYCLES;
        end

        JUMP_CHILD: begin
          if (init_child) handler_mode <= NODE_INIT;
          else begin
            next_handler_mode_after_wait <= NODE_READ;
            handler_mode <= WAIT_ONE_CYCLE;
          end
        end

        NODE_READ: begin
          if (o_all_nodes_occupied) handler_mode <= ERROR;
          else if (is_inside_bb || inside_bb_found)
            if (is_child_not_init) handler_mode <= UPDATE_BRANCH;
            else if (is_leaf) handler_mode <= END_NEW_POINT;
            else handler_mode <= JUMP_CHILD;
          else if (is_root) handler_mode <= END_NEW_POINT;
          else handler_mode <= JUMP_PARENT;
        end

        JUMP_PARENT: begin
          next_handler_mode_after_wait <= NODE_READ;
          handler_mode <= WAIT_ONE_CYCLE;
        end

        END_NEW_POINT: begin
          next_handler_mode_after_wait <= IDLE;
          handler_mode <= WAIT_ONE_CYCLE;
        end

        DFS_JUMP_ROOT: begin
          handler_mode <= WAIT_ONE_CYCLE;
          if (!is_tree_empty) next_handler_mode_after_wait <= DFS_FIRST_ENTRY;
          else next_handler_mode_after_wait <= END_DFS;
        end

        DFS_FIRST_ENTRY: begin
          handler_mode <= DFS_NEXT_VALID_BRANCH;
        end

        DFS_NEXT_VALID_BRANCH: begin
          if (is_leaf) handler_mode <= DFS_JUMP_PARENT;
          else handler_mode <= DFS_CLEAR_BRANCH;
        end

        DFS_CLEAR_BRANCH: begin
          handler_mode <= WAIT_ONE_CYCLE;
          next_handler_mode_after_wait <= DFS_JUMP_CHILD;
        end

        DFS_JUMP_CHILD: begin
          handler_mode <= WAIT_ONE_CYCLE;
          next_handler_mode_after_wait <= DFS_FIRST_ENTRY;
        end

        DFS_JUMP_PARENT: begin
          handler_mode <= WAIT_ONE_CYCLE;
          next_handler_mode_after_wait <= DFS_READ;
        end

        DFS_READ: begin
          if (is_dfs_empty) begin
            if (is_root) handler_mode <= END_DFS;
            else handler_mode <= DFS_JUMP_PARENT;
          end else handler_mode <= DFS_NEXT_VALID_BRANCH;
        end

        END_DFS: begin
          handler_mode <= IDLE;
        end

        WAIT_ONE_CYCLE: begin
          handler_mode <= next_handler_mode_after_wait;
        end

        WAIT_TWO_CYCLES: begin
          handler_mode <= WAIT_ONE_CYCLE;
        end

        default: begin
          handler_mode <= handler_mode;

        end

      endcase
    end else begin
      handler_mode <= RESET;
      next_handler_mode_after_wait <= RESET;
    end
  end

endmodule
