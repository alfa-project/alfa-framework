`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 08/08/2024 03:02:39 PM
// Design Name:
// Module Name: WriteMemory
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


module WriteMemory(
  input  wire        i_clk               ,
  input  wire        i_rst               ,
  input  wire        i_allpoints         ,
  input  wire [15:0] i_range             ,
  input  wire        EXT_MEM_writeTxnDone,
  output reg  [18:0] bram_rd_address     ,
  output reg  [31:0] EXT_MEM_writeAddress,
  output reg  [63:0] EXT_MEM_writePayload,
  output reg         EXT_MEM_initWriteTxn,
  output reg         o_stall
  );

  reg   [2:0] state  ;
  wire        done   ;
  reg  [28:0] address;
  reg  [15:0] x      ;
  reg  [ 7:0] y      ;
  reg         stall  ;
  reg         active ;

  localparam [2:0] IDLE        = 3'b000,
                   READ_POINT  = 3'b001,
                   WRITE_POINT = 3'b011,
                   STALL       = 3'b010;

  initial begin
    state <= IDLE;
    o_stall <= 1'b0;
    active <= 1'b0;
  end


  always @(posedge i_clk) begin
    if(i_rst) begin
      case(state)
        IDLE:
          if(active) begin
            state <= READ_POINT;
            o_stall <= 1'b1;
          end
          else begin
            state <= IDLE;
            o_stall <= 1'b0;
            address <= 29'd0;
          end

        READ_POINT:
          state <= STALL;

        STALL:
          state <= WRITE_POINT;

        WRITE_POINT:
          if (EXT_MEM_writeTxnDone) begin
            state  <= IDLE;
            address <= address + 1;
          end
          else
            state <= WRITE_POINT;

        default:
            state <= IDLE;

      endcase
    end
    else begin
      state <= IDLE;
      address <= 29'd0;
    end
  end

  always @(posedge i_clk) begin
    if(i_rst) begin
      case (state)
        IDLE: begin
          if(!active) begin
            bram_rd_address <= 19'b0;
            x <= 16'd0;
            y <= 8'd0;
          end
          else begin
            EXT_MEM_initWriteTxn  <= 1'b0;
          end
        end

        READ_POINT: begin
          bram_rd_address <= bram_rd_address + 1;
        end

        STALL: begin
          x <= bram_rd_address % 2048;
          y <= bram_rd_address / 2048;
        end

        WRITE_POINT: begin
          EXT_MEM_writeAddress <= {address, 3'd0};
          EXT_MEM_writePayload <= {x, y, 8'd0, i_range, 16'd0};
          EXT_MEM_initWriteTxn  <= 1'b1;
        end
        default: begin end
      endcase
    end
    else begin
        bram_rd_address <= 19'b0;
      x <= 16'd0;
      y <= 8'd0;
    end
  end

  always @(posedge i_clk) begin
    if (i_rst) begin
      if (i_allpoints && ~done)
        active <= 1'b1;
      else
        active <= 1'b0;
    end
  end

  assign done = (bram_rd_address == 19'd262143) ? 1'b1: 1'b0;

endmodule
