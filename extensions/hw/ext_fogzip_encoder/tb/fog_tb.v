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

module tb_ext_fog;

  localparam POINTCLOUD_SIZE = 200;
  parameter MEMORY_SIZE = 1024;  // Parameter to hold the size of the memory

  // Inputs
  reg               i_SYSTEM_clk;
  reg               i_SYSTEM_rst;

  wire       [ 0:0] EXT_writeValid;
  reg        [ 0:0] EXT_writeReady;
  wire       [ 0:0] EXT_readReady;
  reg        [ 0:0] EXT_readValid;
  reg        [18:0] EXT_PCSize;
  wire       [ 0:0] EXT_doneProcessing;
  wire       [15:0] EXT_writeCustomField;
  reg        [15:0] EXT_readCustomField;
  wire       [18:0] EXT_writeID;
  wire       [18:0] EXT_readID;
  reg        [ 0:0] EXT_enable;
  wire       [31:0] EXT_status;
  reg signed [15:0] EXT_pointX;
  reg signed [15:0] EXT_pointY;
  reg signed [15:0] EXT_pointZ;
  wire       [31:0] EXT_MEM_writeAddress;
  wire       [63:0] EXT_MEM_writePayload;
  wire       [31:0] EXT_MEM_readAddress;
  reg        [63:0] EXT_MEM_readPayload;
  wire              EXT_MEM_initWriteTxn;
  wire              EXT_MEM_initReadTxn;
  reg               EXT_MEM_writeTxnDone;
  reg               EXT_MEM_readTxnDone;
  reg               EXT_MEM_error;

  reg signed [15:0] mem_point_x          [0:POINTCLOUD_SIZE-1];
  reg signed [15:0] mem_point_y          [0:POINTCLOUD_SIZE-1];
  reg signed [15:0] mem_point_z          [0:POINTCLOUD_SIZE-1];
  reg        [15:0] custom_field         [0:POINTCLOUD_SIZE-1];

  reg        [ 7:0] memory               [    0:MEMORY_SIZE-1];  // Memory to store bytes

  ext_fog #(
      .NUMBER_NODES(100000),
      .PARALLEL_PROCESSING(1),
      .COMPRESSION(1),
      .BB_MIN_X(-20000),
      .BB_MIN_Y(-20000),
      .BB_MIN_Z(-20000),
      .BB_MAX_X(20000),
      .BB_MAX_Y(20000),
      .BB_MAX_Z(20000),
      .MAX_DEPTH(9)
  ) uut (
      .i_SYSTEM_clk(i_SYSTEM_clk),
      .i_SYSTEM_rst(i_SYSTEM_rst),
      .EXT_writeValid(EXT_writeValid),
      .EXT_writeReady(EXT_writeReady),
      .EXT_readReady(EXT_readReady),
      .EXT_readValid(EXT_readValid),
      .EXT_PCSize(EXT_PCSize),
      .EXT_doneProcessing(EXT_doneProcessing),
      .EXT_writeCustomField(EXT_writeCustomField),
      .EXT_readCustomField(EXT_readCustomField),
      .EXT_writeID(EXT_writeID),
      .EXT_readID(EXT_readID),
      .EXT_enable(EXT_enable),
      .EXT_status(EXT_status),
      .EXT_pointX(EXT_pointX),
      .EXT_pointY(EXT_pointY),
      .EXT_pointZ(EXT_pointZ),
      .EXT_MEM_writeAddress(EXT_MEM_writeAddress),
      .EXT_MEM_writePayload(EXT_MEM_writePayload),
      .EXT_MEM_readAddress(EXT_MEM_readAddress),
      .EXT_MEM_readPayload(EXT_MEM_readPayload),
      .EXT_MEM_initWriteTxn(EXT_MEM_initWriteTxn),
      .EXT_MEM_initReadTxn(EXT_MEM_initReadTxn),
      .EXT_MEM_writeTxnDone(EXT_MEM_writeTxnDone),
      .EXT_MEM_readTxnDone(EXT_MEM_readTxnDone),
      .EXT_MEM_error(EXT_MEM_error)
  );

  // Task to generate random values into mem
  task gen_random_points(input integer num_points);
    integer i;  // Loop variable
    begin
      EXT_PCSize <= num_points;

      for (i = 0; i < num_points; i = i + 1) begin
        // Generate random values
        mem_point_x[i]  = $random % 16384;  // Generates a random value between -16384 and 16384
        mem_point_y[i]  = $random % 16384;  // Generates a random value between -16384 and 16384
        mem_point_z[i]  = $random % 16384;  // Generates a random value between -16384 and 16384
        custom_field[i] = i;

        // Output the generated values to the console
        $display("Generated point %0d: (x: %d, y: %d, z: %d)", i, mem_point_x[i], mem_point_y[i],
                 mem_point_z[i]);
      end
    end
  endtask

  // Task to define fixed points into mem
  task define_fixed_points(input integer num_points);
    integer i;  // Loop variable
    begin
      EXT_PCSize <= num_points;

      // Define the fixed points
      mem_point_x[0]  = 13604;
      mem_point_y[0]  = -8575;
      mem_point_z[0]  = -10743;
      mem_point_x[1]  = -10653;
      mem_point_y[1]  = 15117;
      mem_point_z[1]  = 6541;
      mem_point_x[2]  = -15259;
      mem_point_y[2]  = -11758;
      mem_point_z[2]  = 8961;
      mem_point_x[3]  = 3341;
      mem_point_y[3]  = 12662;
      mem_point_z[3]  = 3389;
      mem_point_x[4]  = 6125;
      mem_point_y[4]  = 14220;
      mem_point_z[4]  = 10745;
      mem_point_x[5]  = -6970;
      mem_point_y[5]  = -15163;
      mem_point_z[5]  = -11606;
      mem_point_x[6]  = 14309;
      mem_point_y[6]  = -3465;
      mem_point_z[6]  = -10734;
      mem_point_x[7]  = 7055;
      mem_point_y[7]  = 10738;
      mem_point_z[7]  = -10546;
      mem_point_x[8]  = -1304;
      mem_point_y[8]  = -12603;
      mem_point_z[8]  = 2396;
      mem_point_x[9]  = -5955;
      mem_point_y[9]  = -10195;
      mem_point_z[9]  = -6555;
      mem_point_x[10] = -7581;
      mem_point_y[10] = 1802;
      mem_point_z[10] = -7552;
      mem_point_x[11] = 8480;
      mem_point_y[11] = 1450;
      mem_point_z[11] = -13155;
      mem_point_x[12] = -362;
      mem_point_y[12] = -2029;
      mem_point_z[12] = -2035;
      mem_point_x[13] = -10669;
      mem_point_y[13] = 7531;
      mem_point_z[13] = -5419;
      mem_point_x[14] = -13822;
      mem_point_y[14] = -338;
      mem_point_z[14] = 10525;
      mem_point_x[15] = -3377;
      mem_point_y[15] = 2339;
      mem_point_z[15] = 9482;
      mem_point_x[16] = -13622;
      mem_point_y[16] = -13252;
      mem_point_z[16] = 15858;
      mem_point_x[17] = 8586;
      mem_point_y[17] = 13121;
      mem_point_z[17] = -2856;
      mem_point_x[18] = 13176;
      mem_point_y[18] = -11639;
      mem_point_z[18] = 3563;
      mem_point_x[19] = 9654;
      mem_point_y[19] = 14790;
      mem_point_z[19] = 5038;

      // Output the generated values to the console
      for (i = 0; i < num_points; i = i + 1) begin
        $display("Fixed point %0d: (x: %d, y: %d, z: %d)", i, mem_point_x[i], mem_point_y[i],
                 mem_point_z[i]);
      end
    end
  endtask

  // Task to simulate EXT interface (read points)
  task read_mem();
    begin
      EXT_pointX <= mem_point_x[EXT_readID];
      EXT_pointY <= mem_point_y[EXT_readID];
      EXT_pointZ <= mem_point_z[EXT_readID];
      EXT_readCustomField <= custom_field[EXT_readID];
      #10;
      EXT_readValid <= 1;
      #10;
      EXT_readValid <= 0;
    end
  endtask

  always @(posedge EXT_readReady) begin
    read_mem();
  end

  // Task to simulate EXT interface (write custom field)
  task write_mem();
    begin
      custom_field[EXT_writeID] <= EXT_writeCustomField;
      #10;
      EXT_writeReady <= 1;
      #10;
      EXT_writeReady <= 0;
    end
  endtask

  always @(posedge EXT_writeValid) begin
    write_mem();
  end

  // Task to simulate EXT MEM interface (write data)
  task simulate_ext_mem_write();
    integer       i;
    reg     [7:0] c_byte;
    begin
      for (i = 0; i < 8; i = i + 1) begin
        c_byte = EXT_MEM_writePayload >> (i * 8);
        memory[EXT_MEM_writeAddress+i] = c_byte;  // Write byte to memory
      end

      $display("Writing payload %b to address %d", EXT_MEM_writePayload, EXT_MEM_writeAddress);
      #3000;
      EXT_MEM_writeTxnDone <= 1;
      #10;
      EXT_MEM_writeTxnDone <= 0;
    end
  endtask

  // Task to output the entire memory
  task output_memory();
    integer i;
    begin
      $display("Memory Content:");
      for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
        if (i == MEMORY_SIZE - 1) begin
          $display("%0d", memory[i]);
        end else begin
          $display("%0d,", memory[i]);
        end
      end
    end
  endtask




  always @(posedge EXT_MEM_initWriteTxn) begin
    simulate_ext_mem_write();
  end


  // Reset
  task reset_ext();
    begin
      i_SYSTEM_rst <= 0;
      EXT_writeReady <= 0;
      EXT_readValid <= 0;
      EXT_readCustomField <= 0;
      EXT_enable <= 0;
      EXT_pointX <= 0;
      EXT_pointY <= 0;
      EXT_pointZ <= 0;
      EXT_MEM_readPayload <= 0;
      EXT_MEM_writeTxnDone <= 0;
      EXT_MEM_readTxnDone <= 0;
      EXT_MEM_error <= 0;
      #10;
      i_SYSTEM_rst <= 1;
      #10;
    end
  endtask

  // Task to do an entire frame processing
  task process_frame();
    begin
      reset_ext();
      #10;
      EXT_enable <= 1;
      wait (EXT_doneProcessing);
      #10;
      output_memory();  // Output memory at the end of processing
      EXT_enable <= 0;
      #10;
    end
  endtask

  // Clock generation
  initial begin
    i_SYSTEM_clk = 1;
    forever #5 i_SYSTEM_clk = ~i_SYSTEM_clk;  // 100MHz clock
  end

  // Test stimuli
  initial begin
    gen_random_points(POINTCLOUD_SIZE);
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    process_frame();
    #10;
    $finish;
  end

endmodule
