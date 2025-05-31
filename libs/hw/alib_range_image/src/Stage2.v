`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/07/2024 11:48:00 AM
// Design Name: 
// Module Name: Stage2
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


module Stage2 (
  input  wire        i_clk         ,
  input  wire        i_rst         ,
  input  wire [15:0] i_point_h     ,
  input  wire [15:0] i_point_v     ,
  input  wire [15:0] i_point_r     ,
  input  wire  [1:0] i_sensorSelect,
  input  wire        fifo_empty    ,
  output reg         o_rd_fifo     ,
  output wire        o_wr_bram     ,
  output wire [18:0] o_wAddress    ,
  output wire [15:0] x             ,
  output wire  [7:0] y             ,
  output wire [15:0] r
  );

  reg  [18:0] previous_address;
  wire        validAngle      ;
  
  always @(posedge i_clk) begin
    if(i_rst) begin
      previous_address <= o_wAddress;
      if(!fifo_empty)
        o_rd_fifo <= 1'b1;
      else
        o_rd_fifo <= 1'b0;
    end
  end

  assign o_wr_bram = ((previous_address != o_wAddress) && validAngle) ? 1'b1 : 1'b0;
  
  RI_generator RI_generator(
    .clk           (i_clk         ),
    .reset         (i_rst         ),
    .i_azimuth     (i_point_h     ),
    .i_elevation   (i_point_v     ),
    .i_range       (i_point_r     ),
    .i_SensorSelect(i_sensorSelect),
    .o_validAngle  (validAngle    ),
    .o_wAddress    (o_wAddress    ),
    .o_x           (x             ),
    .o_y           (y             ),
    .o_range       (r             )
    );
   
endmodule
