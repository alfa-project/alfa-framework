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
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 08/06/2024 03:33:34 PM
// Design Name:
// Module Name: Stage1
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module Stage1(
  input  wire        i_clk             ,
  input  wire        i_rst             ,
  input  wire        i_extEnable       ,
  input  wire        i_stall           ,
  input  wire [15:0] EXT_pointAngleH   ,
  input  wire [15:0] EXT_pointAngleV   ,
  input  wire [15:0] EXT_pointRadius   ,
  output reg   [0:0] EXT_readReady     ,
  input  wire  [0:0] EXT_readValid     ,
  input  wire [18:0] EXT_PCSize        ,
  output reg  [18:0] EXT_readID        ,
  output reg   [0:0] EXT_doneProcessing,
  output wire [15:0] o_point_h         ,
  output wire [15:0] o_point_v         ,
  output wire [15:0] o_point_r         ,
  output wire        fifo_empty        ,
  output wire        fifo_full         ,
  input  wire        rd_fifo           ,
  output wire        o_allpoints
  );

  reg  [2:0] state        ;
  reg [18:0] point_counter;
  reg        wr_fifo      ;

  initial begin
    point_counter <= 0;
  end

  localparam [2:0] IDLE            = 3'b001,
                   READ_POINT      = 3'b010,
                   FULL            = 3'b011,
                   DONE_PROCESSING = 3'b100,
                   WRITE_POINT     = 3'b101,
                   RESET           = 3'b000;


  always @(posedge i_clk) begin
    if(i_rst) begin
      case(state)
        IDLE:
          if(i_extEnable) begin
            if (point_counter<EXT_PCSize)
              state  <= READ_POINT;
            else
              state  <= DONE_PROCESSING;
          end
          else state <= IDLE;

        READ_POINT:
          if(!fifo_full) begin
            if(EXT_readReady && EXT_readValid) begin
              state  <= WRITE_POINT;
              point_counter <= point_counter + 1;
            end
            else
              state  <= READ_POINT;
          end
          else
            state <= FULL;

        WRITE_POINT:
          state <= IDLE;

        FULL:
          if(!fifo_full)
            state <= READ_POINT;
          else
            state <= FULL;

        DONE_PROCESSING:
          if(EXT_doneProcessing) begin //
            state <= IDLE;
            point_counter <= 19'd0;
          end
          else
            state <= DONE_PROCESSING;

        default:
          state <= IDLE;

      endcase
    end
  end


  always @(posedge i_clk) begin
    if(i_rst) begin
      case (state)
        IDLE: begin
          EXT_readReady <= 1'b0;
          EXT_doneProcessing <= 1'b0;
          EXT_readID <= EXT_readID;
          wr_fifo <= 1'b0;
        end

        READ_POINT: begin
          EXT_readReady <= 1'b1;
          EXT_readID <= point_counter;
        end

        FULL: begin
          wr_fifo <= 0;
        end

        WRITE_POINT:
          wr_fifo <= 1'b1;

        DONE_PROCESSING: begin
          if(!i_stall) begin
            EXT_doneProcessing <= 1'b1;
            EXT_readReady <= 1'b0;
          end
        end

        default :  begin
          EXT_readReady <= 1'b0;
          EXT_doneProcessing <= 1'b0;
          EXT_readID <= 1'b0;
        end
      endcase
    end
    else begin
      EXT_readReady <= 1'b0;
      EXT_doneProcessing <= 1'b0;
      EXT_readID <= 1'b0;
    end
  end

  assign o_allpoints = (point_counter == EXT_PCSize) ? 1'b1 : 1'b0;

  points_circular_fifo fifo(
    .clk         (i_clk          ),
    .rst         (i_rst          ),
    .point_h_in  (EXT_pointAngleH),
    .point_v_in  (EXT_pointAngleV),
    .point_r_in  (EXT_pointRadius),
    .wr_en       (wr_fifo        ),
    .rd_en       (rd_fifo        ),
    .point_h_out (o_point_h      ),
    .point_v_out (o_point_v      ),
    .point_r_out (o_point_r      ),
    .full        (fifo_full      ),
    .empty       (fifo_empty     )
    );

endmodule
