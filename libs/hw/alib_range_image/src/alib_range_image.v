`timescale 1ns / 1ps

module FRIC (
  input  wire        i_SYSTEM_clk        ,
  input  wire        i_SYSTEM_rst        ,
  // Extension_Interface
  output reg   [0:0] EXT_writeValid      ,
  input  wire  [0:0] EXT_writeReady      ,
  output wire  [0:0] EXT_readReady       ,
  input  wire  [0:0] EXT_readValid       ,
  input  wire [18:0] EXT_PCSize          ,
  output wire  [0:0] EXT_doneProcessing  ,
  output reg  [15:0] EXT_writeCustomField,
  input  wire [15:0] EXT_readCustomField ,
  output reg  [18:0] EXT_writeID         ,
  output wire [18:0] EXT_readID          ,
  input  wire  [0:0] EXT_enable          ,
  output wire [31:0] EXT_status          ,
  // Cartesian Representation
  input  wire [15:0] EXT_pointAngleH     ,
  input  wire [15:0] EXT_pointAngleV     ,
  input  wire [15:0] EXT_pointRadius     ,
  // EXT memory
  output wire [31:0] EXT_MEM_writeAddress,
  output wire [63:0] EXT_MEM_writePayload,
  output reg  [31:0] EXT_MEM_readAddress ,
  input  wire [63:0] EXT_MEM_readPayload ,
  output wire        EXT_MEM_initWriteTxn,
  output reg         EXT_MEM_initReadTxn ,
  input  wire        EXT_MEM_writeTxnDone,
  input  wire        EXT_MEM_readTxnDone ,
  input  wire        EXT_MEM_error       ,
  // FRIC Interface
  input  wire  [1:0] i_sensorSelect      ,
  output wire [15:0] o_h                 ,
  output wire [15:0] o_v                 ,
  output wire [15:0] o_r                 ,
  output wire        empty               ,
  output wire        full                ,
  output wire [15:0] o_x                 ,
  output wire  [7:0] o_y
  );

  wire [15:0] i_r      ;
  wire [15:0] o_range  ;
  wire [18:0] rdAddress;
  wire        wr_bram  ;
  wire [18:0] wrAddress;
  wire        rd_fifo  ;
  wire        stall    ;
  wire        allpoints;
  wire        start    ;

  Stage1 ReadPoint (
    .i_clk              (i_SYSTEM_clk      ),
    .i_rst              (i_SYSTEM_rst      ),
    .i_extEnable        (EXT_enable        ),
    .i_stall            (stall             ),
    .EXT_pointAngleH    (EXT_pointAngleH   ),
    .EXT_pointAngleV    (EXT_pointAngleV   ),
    .EXT_pointRadius    (EXT_pointRadius   ),
    .EXT_readReady      (EXT_readReady     ),
    .EXT_readValid      (EXT_readValid     ),
    .EXT_PCSize         (EXT_PCSize        ),
    .EXT_readID         (EXT_readID        ),
    .EXT_doneProcessing (EXT_doneProcessing),
    .o_point_h          (o_h               ),
    .o_point_v          (o_v               ),
    .o_point_r          (o_r               ),
    .fifo_empty         (empty             ),
    .fifo_full          (full              ),
    .rd_fifo            (rd_fifo           ),
    .o_allpoints        (allpoints         )
    );

  Stage2 PocessPoint(
    .i_clk          (i_SYSTEM_clk  ),
    .i_rst          (i_SYSTEM_rst  ),
    .i_point_h      (o_h           ),
    .i_point_v      (o_v           ),
    .i_point_r      (o_r           ),
    .i_sensorSelect (i_sensorSelect),
    .fifo_empty     (empty         ),
    .o_rd_fifo      (rd_fifo       ),
    .o_wr_bram      (wr_bram       ),
    .o_wAddress     (wrAddress     ),
    .x              (o_x           ),
    .y              (o_y           ),
    .r              (i_r           )
    );

  WriteMemory PSMem(
    .i_clk                (i_SYSTEM_clk        ),
    .i_rst                (i_SYSTEM_rst        ),
    .i_allpoints          (allpoints           ),
    .i_range              (o_range             ),
    .EXT_MEM_writeTxnDone (EXT_MEM_writeTxnDone),
    .bram_rd_address      (rdAddress           ),
    .EXT_MEM_writeAddress (EXT_MEM_writeAddress),
    .EXT_MEM_writePayload (EXT_MEM_writePayload),
    .EXT_MEM_initWriteTxn (EXT_MEM_initWriteTxn),
    .o_stall              (stall               )

    );

  alib_bram_r_w WriteBack (
    .clk   (i_SYSTEM_clk),  // Write clock
    .addra (wrAddress   ),  // Write address bus
    .din   (i_r         ),  // RAM input data
    .we    (wr_bram     ),  // Write enable
    // Port B (Read Only)
    .addrb (rdAddress   ),  // Read address bus
    .rst   (i_SYSTEM_rst),  // Output reset
    .dout  (o_range     )   // RAM output data
    );

endmodule