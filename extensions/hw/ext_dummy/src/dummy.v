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

module dummy (
    input  wire        i_SYSTEM_clk,
    input  wire        i_SYSTEM_rst,
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
    input wire [15:0] EXT_pointX,
    input wire [15:0] EXT_pointY,
    input wire [15:0] EXT_pointZ
);

  reg  [18:0] point_counter;
  reg  [ 2:0] state;
  wire [ 0:0] error;

  localparam [2:0] IDLE = 3'b001, 
                READ_POINT = 3'b010, 
                PROCESS = 3'b011, 
                WRITE_POINT = 3'b100, 
                DONE_PROCESSING = 3'b101,
                END_PROCESSING = 3'b110,
                ERROR = 3'b111, 
                RESET = 3'b000;

  assign EXT_status[2:0] = state;
  assign EXT_status[31:3] = 29'd0;
  assign error = ((point_counter > EXT_PCSize) && (state != IDLE)) ? 1'b1 : 1'b0;

  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst) begin
      case (state)

        RESET: state <= IDLE;

        ERROR: state <= ERROR;

        IDLE:
        if (EXT_enable) begin
          if (point_counter < EXT_PCSize) state <= READ_POINT;
          else state <= DONE_PROCESSING;
        end else state <= IDLE;

        READ_POINT:
        if (error) state <= ERROR;
        else if (EXT_readReady && EXT_readValid) begin
          state <= PROCESS;
          point_counter <= point_counter + 1;
        end else state <= READ_POINT;

        PROCESS:
        if (error != 0) state <= ERROR;
        else state <= WRITE_POINT;

        WRITE_POINT:
        if (error != 0) state <= ERROR;
        else if (EXT_writeValid && EXT_writeReady) state <= IDLE;

        DONE_PROCESSING:
        if (error != 0) state <= ERROR;
        else begin
          if (EXT_doneProcessing) begin
            state <= IDLE;
            point_counter <= 32'd0;
          end else state <= DONE_PROCESSING;
        end

        default: state <= IDLE;
      endcase
    end else begin
      state <= RESET;
      point_counter <= 32'd0;
    end
  end



  always @(posedge i_SYSTEM_clk) begin
    if (i_SYSTEM_rst) begin
      case (state)

        RESET: begin
          EXT_writeValid <= 1'b0;
          EXT_readReady <= 1'b0;
          EXT_doneProcessing <= 1'b0;
          EXT_writeCustomField <= 1'b0;
          EXT_writeID <= 1'b0;
          EXT_readID <= 1'b0;
        end

        ERROR: begin
          EXT_writeValid <= 1'b0;
          EXT_readReady  <= 1'b0;
        end

        IDLE: begin
          EXT_writeValid <= 1'b0;
          EXT_readReady <= 1'b0;
          EXT_doneProcessing <= 1'b0;
          EXT_writeCustomField <= EXT_writeCustomField;
          EXT_writeID <= EXT_writeID;
          EXT_readID <= EXT_readID;
        end

        READ_POINT: begin
          EXT_readReady <= 1'b1;
          EXT_readID <= point_counter;
        end

        PROCESS: begin
          EXT_readReady <= 1'b0;
          EXT_writeCustomField <= 1;
        end

        WRITE_POINT: begin
          EXT_writeValid <= 1'b1;
          EXT_writeID <= EXT_readID;
        end

        DONE_PROCESSING: begin
          EXT_doneProcessing <= 1'b1;
          EXT_writeValid <= 1'b0;
          EXT_readReady <= 1'b0;
        end

        default: begin
          EXT_writeValid <= 1'b0;
          EXT_readReady <= 1'b0;
          EXT_doneProcessing <= 1'b0;
          EXT_writeCustomField <= 1'b0;
          EXT_writeID <= 1'b0;
          EXT_readID <= 1'b0;
        end
      endcase
    end else begin
      EXT_writeValid <= 1'b0;
      EXT_readReady <= 1'b0;
      EXT_doneProcessing <= 1'b0;
      EXT_writeCustomField <= 1'b0;
      EXT_writeID <= 1'b0;
      EXT_readID <= 1'b0;
    end
  end

endmodule
