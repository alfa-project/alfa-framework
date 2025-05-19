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

/* Module: ALFAUnits
	TODO

    TODO
 
*/
module ALFAUnit #(

                                                                                                    /* Parameters integer: TODO

        TODO = TODO;
    */
                                                                                                    parameter [ 0:0] INTERFACE_TYPE         = 1'h0,
                                                                                                    parameter [ 0:0] SIU_SUPPORT            = 1'h0,
                                                                                                    parameter [ 0:0] EXMU_SUPPORT           = 1'h1,
                                                                                                    parameter [31:0] OFFSET                 = 32'h0,
                                                                                                    parameter [ 0:0] BRAM_DDR               = 1'h1,
                                                                                                    parameter [ 7:0] REPRESENTATION_TYPE    = 1'h0,
                                                                                                    parameter [31:0] REPRESENTATION_PARAM_1 = 360,
                                                                                                    parameter [31:0] REPRESENTATION_PARAM_2 = 90,
                                                                                                    parameter [31:0] REPRESENTATION_PARAM_3 = 11,
                                                                                                    parameter [31:0] REPRESENTATION_PARAM_4 = 5,
                                                                                                    parameter [ 7:0] POINTCLOUD_ID          = 0,
                                                                                                    parameter [ 7:0] NUMBER_OF_DEBUG_POINTS = 20,
                                                                                                    parameter [ 7:0] NUMBER_OF_USER_DEFINES = 10
) (

                                                                                                    // System interface    
                                                                                                    input wire i_SYSTEM_clk,
                                                                                                    input wire i_SYSTEM_rst,


                                                                                                    // Extension Interface
                                                                                                    input  wire [ 0:0] EXT_writeValid,
                                                                                                    output wire [ 0:0] EXT_writeReady,
                                                                                                    input  wire [ 0:0] EXT_readReady,
                                                                                                    output wire [ 0:0] EXT_readValid,
                                                                                                    output wire [18:0] EXT_PCSize,
                                                                                                    input  wire [ 0:0] EXT_doneProcessing,
                                                                                                    input  wire [15:0] EXT_writeCustomField,
                                                                                                    input  wire [18:0] EXT_writeID,
                                                                                                    input  wire [18:0] EXT_readID,
                                                                                                    output wire [ 0:0] EXT_enable,
                                                                                                    output wire [15:0] EXT_pointAngleH,
                                                                                                    output wire [15:0] EXT_pointAngleV,
                                                                                                    output wire [15:0] EXT_pointRadius,
                                                                                                    output wire [15:0] EXT_readCustomField,
                                                                                                    output wire [15:0] EXT_pointX,
                                                                                                    output wire [15:0] EXT_pointY,
                                                                                                    output wire [15:0] EXT_pointZ,

                                                                                                    input  wire [31:0] EXT_debug_point_0,
                                                                                                    input  wire [31:0] EXT_debug_point_1,
                                                                                                    input  wire [31:0] EXT_debug_point_2,
                                                                                                    input  wire [31:0] EXT_debug_point_3,
                                                                                                    input  wire [31:0] EXT_debug_point_4,
                                                                                                    input  wire [31:0] EXT_debug_point_5,
                                                                                                    input  wire [31:0] EXT_debug_point_6,
                                                                                                    input  wire [31:0] EXT_debug_point_7,
                                                                                                    input  wire [31:0] EXT_debug_point_8,
                                                                                                    input  wire [31:0] EXT_debug_point_9,
                                                                                                    input  wire [31:0] EXT_debug_point_10,
                                                                                                    input  wire [31:0] EXT_debug_point_11,
                                                                                                    input  wire [31:0] EXT_debug_point_12,
                                                                                                    input  wire [31:0] EXT_debug_point_13,
                                                                                                    input  wire [31:0] EXT_debug_point_14,
                                                                                                    input  wire [31:0] EXT_debug_point_15,
                                                                                                    input  wire [31:0] EXT_debug_point_16,
                                                                                                    input  wire [31:0] EXT_debug_point_17,
                                                                                                    input  wire [31:0] EXT_debug_point_18,
                                                                                                    input  wire [31:0] EXT_debug_point_19,
                                                                                                    output wire [31:0] EXT_user_define_0,
                                                                                                    output wire [31:0] EXT_user_define_1,
                                                                                                    output wire [31:0] EXT_user_define_2,
                                                                                                    output wire [31:0] EXT_user_define_3,
                                                                                                    output wire [31:0] EXT_user_define_4,
                                                                                                    output wire [31:0] EXT_user_define_5,
                                                                                                    output wire [31:0] EXT_user_define_6,
                                                                                                    output wire [31:0] EXT_user_define_7,
                                                                                                    output wire [31:0] EXT_user_define_8,
                                                                                                    output wire [31:0] EXT_user_define_9,
                                                                                                    input  wire [31:0] EXT_status,

                                                                                                    // Master Interface PC
                                                                                                    output wire [ 5:0] M_AXI_AWID,
                                                                                                    output wire [31:0] M_AXI_AWADDR,
                                                                                                    output wire [ 7:0] M_AXI_AWLEN,
                                                                                                    output wire [ 2:0] M_AXI_AWSIZE,
                                                                                                    output wire [ 1:0] M_AXI_AWBURST,
                                                                                                    output wire [ 0:0] M_AXI_AWLOCK,
                                                                                                    output wire [ 3:0] M_AXI_AWCACHE,
                                                                                                    output wire [ 2:0] M_AXI_AWPROT,
                                                                                                    output wire [ 3:0] M_AXI_AWQOS,
                                                                                                    output wire [ 0:0] M_AXI_AWVALID,
                                                                                                    input  wire [ 0:0] M_AXI_AWREADY,
                                                                                                    output wire [63:0] M_AXI_WDATA,
                                                                                                    output wire [ 7:0] M_AXI_WSTRB,
                                                                                                    output wire [ 0:0] M_AXI_WLAST,
                                                                                                    output wire [ 0:0] M_AXI_WVALID,
                                                                                                    input  wire [ 0:0] M_AXI_WREADY,
                                                                                                    input  wire [ 0:0] M_AXI_BID,
                                                                                                    input  wire [ 1:0] M_AXI_BRESP,
                                                                                                    input  wire [ 0:0] M_AXI_BVALID,
                                                                                                    output wire [ 0:0] M_AXI_BREADY,
                                                                                                    output wire [ 5:0] M_AXI_ARID,
                                                                                                    output wire [31:0] M_AXI_ARADDR,
                                                                                                    output wire [ 7:0] M_AXI_ARLEN,
                                                                                                    output wire [ 2:0] M_AXI_ARSIZE,
                                                                                                    output wire [ 1:0] M_AXI_ARBURST,
                                                                                                    output wire [ 0:0] M_AXI_ARLOCK,
                                                                                                    output wire [ 3:0] M_AXI_ARCACHE,
                                                                                                    output wire [ 2:0] M_AXI_ARPROT,
                                                                                                    output wire [ 3:0] M_AXI_ARQOS,
                                                                                                    output wire [ 0:0] M_AXI_ARVALID,
                                                                                                    input  wire [ 0:0] M_AXI_ARREADY,
                                                                                                    input  wire [ 0:0] M_AXI_RID,
                                                                                                    input  wire [63:0] M_AXI_RDATA,
                                                                                                    input  wire [ 1:0] M_AXI_RRESP,
                                                                                                    input  wire [ 0:0] M_AXI_RLAST,
                                                                                                    input  wire [ 0:0] M_AXI_RVALID,
                                                                                                    output wire [ 0:0] M_AXI_RREADY,

                                                                                                    // Master Interface EXT_MEM
                                                                                                    output wire [ 5:0] M1_AXI_AWID,
                                                                                                    output wire [31:0] M1_AXI_AWADDR,
                                                                                                    output wire [ 7:0] M1_AXI_AWLEN,
                                                                                                    output wire [ 2:0] M1_AXI_AWSIZE,
                                                                                                    output wire [ 1:0] M1_AXI_AWBURST,
                                                                                                    output wire [ 0:0] M1_AXI_AWLOCK,
                                                                                                    output wire [ 3:0] M1_AXI_AWCACHE,
                                                                                                    output wire [ 2:0] M1_AXI_AWPROT,
                                                                                                    output wire [ 3:0] M1_AXI_AWQOS,
                                                                                                    output wire [ 0:0] M1_AXI_AWVALID,
                                                                                                    input  wire [ 0:0] M1_AXI_AWREADY,
                                                                                                    output wire [63:0] M1_AXI_WDATA,
                                                                                                    output wire [ 7:0] M1_AXI_WSTRB,
                                                                                                    output wire [ 0:0] M1_AXI_WLAST,
                                                                                                    output wire [ 0:0] M1_AXI_WVALID,
                                                                                                    input  wire [ 0:0] M1_AXI_WREADY,
                                                                                                    input  wire [ 0:0] M1_AXI_BID,
                                                                                                    input  wire [ 1:0] M1_AXI_BRESP,
                                                                                                    input  wire [ 0:0] M1_AXI_BVALID,
                                                                                                    output wire [ 0:0] M1_AXI_BREADY,
                                                                                                    output wire [ 5:0] M1_AXI_ARID,
                                                                                                    output wire [31:0] M1_AXI_ARADDR,
                                                                                                    output wire [ 7:0] M1_AXI_ARLEN,
                                                                                                    output wire [ 2:0] M1_AXI_ARSIZE,
                                                                                                    output wire [ 1:0] M1_AXI_ARBURST,
                                                                                                    output wire [ 0:0] M1_AXI_ARLOCK,
                                                                                                    output wire [ 3:0] M1_AXI_ARCACHE,
                                                                                                    output wire [ 2:0] M1_AXI_ARPROT,
                                                                                                    output wire [ 3:0] M1_AXI_ARQOS,
                                                                                                    output wire [ 0:0] M1_AXI_ARVALID,
                                                                                                    input  wire [ 0:0] M1_AXI_ARREADY,
                                                                                                    input  wire [ 0:0] M1_AXI_RID,
                                                                                                    input  wire [63:0] M1_AXI_RDATA,
                                                                                                    input  wire [ 1:0] M1_AXI_RRESP,
                                                                                                    input  wire [ 0:0] M1_AXI_RLAST,
                                                                                                    input  wire [ 0:0] M1_AXI_RVALID,
                                                                                                    output wire [ 0:0] M1_AXI_RREADY,

                                                                                                    // Slave Interface  
                                                                                                    input  wire [31:0] S_AXI_AWADDR,
                                                                                                    input  wire [ 2:0] S_AXI_AWPROT,
                                                                                                    input  wire [ 0:0] S_AXI_AWVALID,
                                                                                                    output wire [ 0:0] S_AXI_AWREADY,
                                                                                                    input  wire [31:0] S_AXI_WDATA,
                                                                                                    input  wire [ 3:0] S_AXI_WSTRB,
                                                                                                    input  wire [ 0:0] S_AXI_WVALID,
                                                                                                    output wire [ 0:0] S_AXI_WREADY,
                                                                                                    output wire [ 1:0] S_AXI_BRESP,
                                                                                                    output wire [ 0:0] S_AXI_BVALID,
                                                                                                    input  wire [ 0:0] S_AXI_BREADY,
                                                                                                    input  wire [31:0] S_AXI_ARADDR,
                                                                                                    input  wire [ 2:0] S_AXI_ARPROT,
                                                                                                    input  wire [ 0:0] S_AXI_ARVALID,
                                                                                                    output wire [ 0:0] S_AXI_ARREADY,
                                                                                                    output wire [31:0] S_AXI_RDATA,
                                                                                                    output wire [ 1:0] S_AXI_RRESP,
                                                                                                    output wire [ 0:0] S_AXI_RVALID,
                                                                                                    input  wire [ 0:0] S_AXI_RREADY,

                                                                                                    // EXT memory
                                                                                                    input wire [31:0] EXT_MEM_writeAddress,
                                                                                                    input wire [63:0] EXT_MEM_writePayload,
                                                                                                    input wire [31:0] EXT_MEM_readAddress,
                                                                                                    output wire [63:0] EXT_MEM_readPayload,
                                                                                                    input wire EXT_MEM_initWriteTxn,
                                                                                                    input wire EXT_MEM_initReadTxn,
                                                                                                    output wire EXT_MEM_writeTxnDone,
                                                                                                    output wire EXT_MEM_readTxnDone,
                                                                                                    output wire EXT_MEM_error
);

  //SIU interface
  wire [   0:0] SIU_newPoint;
  wire [  18:0] SIU_pointID;
  wire [   0:0] SIU_newFrame;
  wire [  15:0] SIU_angleH;
  wire [  15:0] SIU_angleV;
  wire [  15:0] SIU_distR0;
  wire [   7:0] SIU_reflR0;
  wire [  15:0] SIU_distR1;
  wire [   7:0] SIU_reflR1;
  wire [   7:0] SIU_label;

  //ExMU interface
  wire [   0:0] ExMU_readInCache;
  wire [   0:0] ExMU_writeInCache;
  wire [   0:0] ExMU_newPoint;
  wire [  18:0] ExMU_pointReadID;
  wire [  18:0] ExMU_pointReWriteID;
  wire [2047:0] ExMU_pointReWritePayload;


  //MemMU interface
  wire [  31:0] MemMU_pointReadAddress;
  wire [  31:0] MemMU_pointWriteAddress;
  wire [2047:0] MemMU_pointWritePayload;
  wire [  18:0] MemMU_size;

  //CU interface
  wire [   0:0] CU_INT_initWriteTx;
  wire [   0:0] CU_INT_initReadTxn;
  wire [   0:0] CU_SIU_ExMU;
  wire [   0:0] CU_ExMU_readCache;
  wire [   0:0] CU_ExMU_writeCache;
  wire [   0:0] CU_ExMU_readWriteID;
  wire [   0:0] CU_ExMU_readPoint;
  wire [   0:0] CU_ExMU_writePoint;
  wire [   0:0] CU_ExMU_writeMem;
  wire [   0:0] CU_MonU_pcReady_software;
  wire [   0:0] CU_MonU_processingDone_software;
  wire [   0:0] CU_MonU_pcReady_hardware;
  wire [   0:0] CU_MonU_processingDone_hardware;
  wire [   0:0] CU_MonU_pcReady;

  //INT interface
  wire [   0:0] INT_writeTxnDone;
  wire [   0:0] INT_readTxnDone;
  wire [2047:0] INT_readPayload;
  wire [   0:0] INT_error;

  //MonU interface
  wire [  31:0] MonU_CU_parameter;
  wire [  31:0] MonU_MemMU_parameter;
  wire [  31:0] MonU_ExMU_parameter;
  wire [  31:0] MonU_parameter;
  wire [  31:0] MonU_SIU_parameter;
  wire [  31:0] MonU_CU_status;
  wire [  31:0] MonU_MemMU_status;
  wire [  31:0] MonU_ExMU_status;
  wire [  31:0] MonU_status;
  wire [  31:0] MonU_SIU_status;
  wire [  31:0] MonU_PointCloudSize;

  MemMU #(
                                                                                                      .OFFSET               (OFFSET),
                                                                                                      .BRAM_DDR             (BRAM_DDR),
                                                                                                      .POINTCLOUD_ID        (POINTCLOUD_ID),
                                                                                                      .REPRESENTATION_TYPE  (REPRESENTATION_TYPE),
                                                                                                      .REPRESENTATION_PARAM1(REPRESENTATION_PARAM_1),
                                                                                                      .REPRESENTATION_PARAM2(REPRESENTATION_PARAM_2),
                                                                                                      .REPRESENTATION_PARAM3(REPRESENTATION_PARAM_3),
                                                                                                      .REPRESENTATION_PARAM4(REPRESENTATION_PARAM_4)
  ) MemMU_inst (
                                                                                                      .i_SYSTEM_clk              (i_SYSTEM_clk),
                                                                                                      .i_SYSTEM_rst              (i_SYSTEM_rst),
                                                                                                      .i_MonU_MemMU_parameter    (MonU_MemMU_parameter),
                                                                                                      .i_CU_SIU_ExMU             (CU_SIU_ExMU),
                                                                                                      .i_SIU_pointID             (SIU_pointID),
                                                                                                      .i_SIU_newFrame            (SIU_newFrame),
                                                                                                      .i_SIU_angleH              (SIU_angleH),
                                                                                                      .i_SIU_angleV              (SIU_angleV),
                                                                                                      .i_SIU_distR0              (SIU_distR0),
                                                                                                      .i_SIU_reflR0              (SIU_reflR0),
                                                                                                      .i_SIU_distR1              (SIU_distR1),
                                                                                                      .i_SIU_reflR1              (SIU_reflR1),
                                                                                                      .i_SIU_label               (SIU_label),
                                                                                                      .i_ExMU_pointReadID        (ExMU_pointReadID),
                                                                                                      .i_ExMU_pointReWriteID     (ExMU_pointReWriteID),
                                                                                                      .i_ExMU_pointReWritePayload(ExMU_pointReWritePayload),
                                                                                                      .o_MemMU_pointReadAddress  (MemMU_pointReadAddress),
                                                                                                      .o_MemMU_pointWriteAddress (MemMU_pointWriteAddress),
                                                                                                      .o_MemMU_pointWritePayload (MemMU_pointWritePayload),
                                                                                                      .o_MemMU_size              (MemMU_size),
                                                                                                      .o_status                  (MonU_MemMU_status)
  );

  ExMU #(
                                                                                                      .REPRESENTATION_TYPE(REPRESENTATION_TYPE)
  ) ExMU_inst (
                                                                                                      .i_SYSTEM_clk          (i_SYSTEM_clk),
                                                                                                      .i_SYSTEM_rst          (i_SYSTEM_rst),
                                                                                                      .i_INT_readPayload     (INT_readPayload),
                                                                                                      .i_CU_ExMU_readCache   (CU_ExMU_readCache),
                                                                                                      .i_CU_ExMU_writeCache  (CU_ExMU_writeCache),
                                                                                                      .i_CU_ExMU_readWriteID (CU_ExMU_readWriteID),
                                                                                                      .i_CU_ExMU_readPoint   (CU_ExMU_readPoint),
                                                                                                      .i_CU_ExMU_writePoint  (CU_ExMU_writePoint),
                                                                                                      .i_CU_ExMU_writeMem    (CU_ExMU_writeMem),
                                                                                                      .o_ExMU_readInCache    (ExMU_readInCache),
                                                                                                      .o_ExMU_writeInCache   (ExMU_writeInCache),
                                                                                                      .i_EXT_writeCustomField(EXT_writeCustomField),
                                                                                                      .i_EXT_writeID         (EXT_writeID),
                                                                                                      .o_ExMU_writePayload   (ExMU_pointReWritePayload),
                                                                                                      .o_ExMU_writeID        (ExMU_pointReWriteID),
                                                                                                      .i_EXT_readID          (EXT_readID),
                                                                                                      .o_ExMU_readID         (ExMU_pointReadID),
                                                                                                      .o_EXT_pointAngleH     (EXT_pointAngleH),
                                                                                                      .o_EXT_pointAngleV     (EXT_pointAngleV),
                                                                                                      .o_EXT_pointRadius     (EXT_pointRadius),
                                                                                                      .o_EXT_readCustomField (EXT_readCustomField),
                                                                                                      .o_EXT_pointX          (EXT_pointX),
                                                                                                      .o_EXT_pointY          (EXT_pointY),
                                                                                                      .o_EXT_pointZ          (EXT_pointZ),
                                                                                                      .o_status              (MonU_ExMU_status)
  );

  CU #(
                                                                                                      .SIU_SUPPORT (SIU_SUPPORT),
                                                                                                      .EXMU_SUPPORT(EXMU_SUPPORT)
  ) CU_inst (
                                                                                                      .i_SYSTEM_clk            (i_SYSTEM_clk),
                                                                                                      .i_SYSTEM_rst            (i_SYSTEM_rst),
                                                                                                      .i_SIU_newPoint          (SIU_newPoint),
                                                                                                      .i_SIU_newFrame          (SIU_newFrame),
                                                                                                      .i_ExMU_readInCache      (ExMU_readInCache),
                                                                                                      .i_ExMU_writeInCache     (ExMU_writeInCache),
                                                                                                      .i_EXT_doneProcessing    (EXT_doneProcessing),
                                                                                                      .i_EXT_writeValid        (EXT_writeValid),
                                                                                                      .i_EXT_readReady         (EXT_readReady),
                                                                                                      .i_MemMU_status          (MonU_MemMU_status),
                                                                                                      .i_ExMU_status           (MonU_ExMU_status),
                                                                                                      .i_MonU_status           (MonU_status),
                                                                                                      .i_SIU_status            (MonU_SIU_status),
                                                                                                      .i_INT_writeTxnDone      (INT_writeTxnDone),
                                                                                                      .i_INT_readTxnDone       (INT_readTxnDone),
                                                                                                      .o_CU_EXT_enable         (EXT_enable),
                                                                                                      .i_CU_MonU_frameDone     (CU_MonU_frameDone),
                                                                                                      .o_CU_MonU_extReady      (CU_MonU_extReady),
                                                                                                      .i_CU_MonU_pcReady       (CU_MonU_pcReady_software),
                                                                                                      .o_CU_EXT_writeReady     (EXT_writeReady),
                                                                                                      .o_CU_EXT_readValid      (EXT_readValid),
                                                                                                      .o_CU_INT_initWriteTx    (CU_INT_initWriteTx),
                                                                                                      .o_CU_INT_initReadTx     (CU_INT_initReadTx),
                                                                                                      .o_CU_ExMU_readCache     (CU_ExMU_readCache),
                                                                                                      .o_CU_ExMU_writeCache    (CU_ExMU_writeCache),
                                                                                                      .o_CU_ExMU_readWriteID   (CU_ExMU_readWriteID),
                                                                                                      .o_CU_ExMU_readPoint     (CU_ExMU_readPoint),
                                                                                                      .o_CU_ExMU_writePoint    (CU_ExMU_writePoint),
                                                                                                      .o_CU_ExMU_writeMem      (CU_ExMU_writeMem),
                                                                                                      .o_CU_MonU_pcReady       (CU_MonU_pcReady_hardware),
                                                                                                      .o_CU_MonU_processingDone(CU_MonU_processingDone_hardware),
                                                                                                      .o_CU_SIU_ExMU           (CU_SIU_ExMU),
                                                                                                      .o_status                (MonU_CU_status)
  );


  generate
    if (SIU_SUPPORT == 1'b1) begin : SIU
      // SIU implementation
      assign SIU_newPoint = 0;
      assign SIU_pointID = 0;
      assign EXT_PCSize[18:0] = MemMU_size;
      assign CU_MonU_pcReady = CU_MonU_pcReady_hardware;
    end else begin : NO_SIU
      assign SIU_newPoint    = 0;
      assign SIU_pointID     = 0;
      assign SIU_newFrame    = 0;
      assign SIU_angleH      = 0;
      assign SIU_angleV      = 0;
      assign SIU_distR0      = 0;
      assign SIU_reflR0      = 0;
      assign SIU_distR1      = 0;
      assign SIU_reflR1      = 0;
      assign SIU_label       = 0;
      assign MonU_SIU_status = 0;
      assign EXT_PCSize      = MonU_PointCloudSize;
      assign CU_MonU_pcReady = CU_MonU_pcReady_software;
    end

    if (INTERFACE_TYPE == 1'b0) begin : AXI
      AXI #(
                                                                                                          .C_M_TARGET_SLAVE_BASE_ADDR(0),
                                                                                                          .C_M_AXI_BURST_LEN_WRITE(32),
                                                                                                          .C_M_AXI_BURST_LEN_READ(32),
                                                                                                          .C_M_AXI_ID_WIDTH(6),
                                                                                                          .C_M_AXI_ADDR_WIDTH(32),
                                                                                                          .C_M_AXI_DATA_WIDTH(64)
      ) AXI_PC_inst (
                                                                                                          .M_AXI_ACLK    (i_SYSTEM_clk),
                                                                                                          .M_AXI_ARESETN (i_SYSTEM_rst),
                                                                                                          .M_AXI_AWID    (M_AXI_AWID),
                                                                                                          .M_AXI_AWADDR  (M_AXI_AWADDR),
                                                                                                          .M_AXI_AWLEN   (M_AXI_AWLEN),
                                                                                                          .M_AXI_AWSIZE  (M_AXI_AWSIZE),
                                                                                                          .M_AXI_AWBURST (M_AXI_AWBURST),
                                                                                                          .M_AXI_AWLOCK  (M_AXI_AWLOCK),
                                                                                                          .M_AXI_AWCACHE (M_AXI_AWCACHE),
                                                                                                          .M_AXI_AWPROT  (M_AXI_AWPROT),
                                                                                                          .M_AXI_AWQOS   (M_AXI_AWQOS),
                                                                                                          .M_AXI_AWVALID (M_AXI_AWVALID),
                                                                                                          .M_AXI_AWREADY (M_AXI_AWREADY),
                                                                                                          .M_AXI_WDATA   (M_AXI_WDATA),
                                                                                                          .M_AXI_WSTRB   (M_AXI_WSTRB),
                                                                                                          .M_AXI_WLAST   (M_AXI_WLAST),
                                                                                                          .M_AXI_WVALID  (M_AXI_WVALID),
                                                                                                          .M_AXI_WREADY  (M_AXI_WREADY),
                                                                                                          .M_AXI_BID     (M_AXI_BID),
                                                                                                          .M_AXI_BRESP   (M_AXI_BRESP),
                                                                                                          .M_AXI_BVALID  (M_AXI_BVALID),
                                                                                                          .M_AXI_BREADY  (M_AXI_BREADY),
                                                                                                          .M_AXI_ARID    (M_AXI_ARID),
                                                                                                          .M_AXI_ARADDR  (M_AXI_ARADDR),
                                                                                                          .M_AXI_ARLEN   (M_AXI_ARLEN),
                                                                                                          .M_AXI_ARSIZE  (M_AXI_ARSIZE),
                                                                                                          .M_AXI_ARBURST (M_AXI_ARBURST),
                                                                                                          .M_AXI_ARLOCK  (M_AXI_ARLOCK),
                                                                                                          .M_AXI_ARCACHE (M_AXI_ARCACHE),
                                                                                                          .M_AXI_ARPROT  (M_AXI_ARPROT),
                                                                                                          .M_AXI_ARQOS   (M_AXI_ARQOS),
                                                                                                          .M_AXI_ARVALID (M_AXI_ARVALID),
                                                                                                          .M_AXI_ARREADY (M_AXI_ARREADY),
                                                                                                          .M_AXI_RID     (M_AXI_RID),
                                                                                                          .M_AXI_RDATA   (M_AXI_RDATA),
                                                                                                          .M_AXI_RRESP   (M_AXI_RRESP),
                                                                                                          .M_AXI_RLAST   (M_AXI_RLAST),
                                                                                                          .M_AXI_RVALID  (M_AXI_RVALID),
                                                                                                          .M_AXI_RREADY  (M_AXI_RREADY),
                                                                                                          .i_writeAddress(MemMU_pointWriteAddress),
                                                                                                          .i_writePayload(MemMU_pointWritePayload),
                                                                                                          .i_readAddress (MemMU_pointReadAddress),
                                                                                                          .i_initWriteTxn(CU_INT_initWriteTx),
                                                                                                          .i_initReadTxn (CU_INT_initReadTx),
                                                                                                          .o_writeTxnDone(INT_writeTxnDone),
                                                                                                          .o_readTxnDone (INT_readTxnDone),
                                                                                                          .o_readPayload (INT_readPayload),
                                                                                                          .o_error       (INT_error)
      );

      AXI_single_word_burst #(
                                                                                                          .C_M_TARGET_SLAVE_BASE_ADDR(0),
                                                                                                          .C_M_AXI_ID_WIDTH(6),
                                                                                                          .C_M_AXI_ADDR_WIDTH(32),
                                                                                                          .C_M_AXI_DATA_WIDTH(64)
      ) AXI_EXT_MEM_inst (
                                                                                                          .M_AXI_ACLK    (i_SYSTEM_clk),
                                                                                                          .M_AXI_ARESETN (i_SYSTEM_rst),
                                                                                                          .M_AXI_AWID    (M1_AXI_AWID),
                                                                                                          .M_AXI_AWADDR  (M1_AXI_AWADDR),
                                                                                                          .M_AXI_AWLEN   (M1_AXI_AWLEN),
                                                                                                          .M_AXI_AWSIZE  (M1_AXI_AWSIZE),
                                                                                                          .M_AXI_AWBURST (M1_AXI_AWBURST),
                                                                                                          .M_AXI_AWLOCK  (M1_AXI_AWLOCK),
                                                                                                          .M_AXI_AWCACHE (M1_AXI_AWCACHE),
                                                                                                          .M_AXI_AWPROT  (M1_AXI_AWPROT),
                                                                                                          .M_AXI_AWQOS   (M1_AXI_AWQOS),
                                                                                                          .M_AXI_AWVALID (M1_AXI_AWVALID),
                                                                                                          .M_AXI_AWREADY (M1_AXI_AWREADY),
                                                                                                          .M_AXI_WDATA   (M1_AXI_WDATA),
                                                                                                          .M_AXI_WSTRB   (M1_AXI_WSTRB),
                                                                                                          .M_AXI_WLAST   (M1_AXI_WLAST),
                                                                                                          .M_AXI_WVALID  (M1_AXI_WVALID),
                                                                                                          .M_AXI_WREADY  (M1_AXI_WREADY),
                                                                                                          .M_AXI_BID     (M1_AXI_BID),
                                                                                                          .M_AXI_BRESP   (M1_AXI_BRESP),
                                                                                                          .M_AXI_BVALID  (M1_AXI_BVALID),
                                                                                                          .M_AXI_BREADY  (M1_AXI_BREADY),
                                                                                                          .M_AXI_ARID    (M1_AXI_ARID),
                                                                                                          .M_AXI_ARADDR  (M1_AXI_ARADDR),
                                                                                                          .M_AXI_ARLEN   (M1_AXI_ARLEN),
                                                                                                          .M_AXI_ARSIZE  (M1_AXI_ARSIZE),
                                                                                                          .M_AXI_ARBURST (M1_AXI_ARBURST),
                                                                                                          .M_AXI_ARLOCK  (M1_AXI_ARLOCK),
                                                                                                          .M_AXI_ARCACHE (M1_AXI_ARCACHE),
                                                                                                          .M_AXI_ARPROT  (M1_AXI_ARPROT),
                                                                                                          .M_AXI_ARQOS   (M1_AXI_ARQOS),
                                                                                                          .M_AXI_ARVALID (M1_AXI_ARVALID),
                                                                                                          .M_AXI_ARREADY (M1_AXI_ARREADY),
                                                                                                          .M_AXI_RID     (M1_AXI_RID),
                                                                                                          .M_AXI_RDATA   (M1_AXI_RDATA),
                                                                                                          .M_AXI_RRESP   (M1_AXI_RRESP),
                                                                                                          .M_AXI_RLAST   (M1_AXI_RLAST),
                                                                                                          .M_AXI_RVALID  (M1_AXI_RVALID),
                                                                                                          .M_AXI_RREADY  (M1_AXI_RREADY),
                                                                                                          .i_writeAddress(EXT_MEM_writeAddress + MonU_ExMU_parameter),
                                                                                                          .i_writePayload(EXT_MEM_writePayload),
                                                                                                          .i_readAddress (EXT_MEM_readAddress + MonU_ExMU_parameter),
                                                                                                          .i_initWriteTxn(EXT_MEM_initWriteTxn),
                                                                                                          .i_initReadTxn (EXT_MEM_initReadTxn),
                                                                                                          .o_writeTxnDone(EXT_MEM_writeTxnDone),
                                                                                                          .o_readTxnDone (EXT_MEM_readTxnDone),
                                                                                                          .o_readPayload (EXT_MEM_readPayload),
                                                                                                          .o_error       (EXT_MEM_error)
      );

      MonU #(
                                                                                                          .DATA_WIDTH(32),
                                                                                                          .ADDR_WIDTH(32)
      ) MonU_inst (
                                                                                                          .i_CU_status          (MonU_CU_status),
                                                                                                          .i_MemMU_status       (MonU_MemMU_status),
                                                                                                          .i_ExMU_status        (MonU_ExMU_status),
                                                                                                          .i_MonU_status        (MonU_status),
                                                                                                          .i_SIU_status         (MonU_SIU_status),
                                                                                                          .i_CU_pcReady         (CU_MonU_pcReady_hardware),
                                                                                                          .i_CU_processingDone  (CU_MonU_processingDone_hardware),
                                                                                                          .o_CU_pcReady         (CU_MonU_pcReady_software),
                                                                                                          .o_CU_processingDone  (CU_MonU_processingDone_software),
                                                                                                          .o_MonU_PointCloudSize(MonU_PointCloudSize),
                                                                                                          .i_EXT_status         (EXT_status),
                                                                                                          .o_CU_frameDone       (CU_MonU_frameDone),
                                                                                                          .i_CU_extReady        (CU_MonU_extReady),
                                                                                                          .i_DP_0               (EXT_debug_point_0),
                                                                                                          .i_DP_1               (EXT_debug_point_1),
                                                                                                          .i_DP_2               (EXT_debug_point_2),
                                                                                                          .i_DP_3               (EXT_debug_point_3),
                                                                                                          .i_DP_4               (EXT_debug_point_4),
                                                                                                          .i_DP_5               (EXT_debug_point_5),
                                                                                                          .i_DP_6               (EXT_debug_point_6),
                                                                                                          .i_DP_7               (EXT_debug_point_7),
                                                                                                          .i_DP_8               (EXT_debug_point_8),
                                                                                                          .i_DP_9               (EXT_debug_point_9),
                                                                                                          .i_DP_10              (EXT_debug_point_10),
                                                                                                          .i_DP_11              (EXT_debug_point_11),
                                                                                                          .i_DP_12              (EXT_debug_point_12),
                                                                                                          .i_DP_13              (EXT_debug_point_13),
                                                                                                          .i_DP_14              (EXT_debug_point_14),
                                                                                                          .i_DP_15              (EXT_debug_point_15),
                                                                                                          .i_DP_16              (EXT_debug_point_16),
                                                                                                          .i_DP_17              (EXT_debug_point_17),
                                                                                                          .i_DP_18              (EXT_debug_point_18),
                                                                                                          .i_DP_19              (EXT_debug_point_19),
                                                                                                          .o_USER_0             (EXT_user_define_0),
                                                                                                          .o_USER_1             (EXT_user_define_1),
                                                                                                          .o_USER_2             (EXT_user_define_2),
                                                                                                          .o_USER_3             (EXT_user_define_3),
                                                                                                          .o_USER_4             (EXT_user_define_4),
                                                                                                          .o_USER_5             (EXT_user_define_5),
                                                                                                          .o_USER_6             (EXT_user_define_6),
                                                                                                          .o_USER_7             (EXT_user_define_7),
                                                                                                          .o_USER_8             (EXT_user_define_8),
                                                                                                          .o_USER_9             (EXT_user_define_9),
                                                                                                          .o_CU_parameter       (MonU_CU_parameter),
                                                                                                          .o_MemMU_parameter    (MonU_MemMU_parameter),
                                                                                                          .o_ExMU_parameter     (MonU_ExMU_parameter),
                                                                                                          .o_MonU_parameter     (MonU_parameter),
                                                                                                          .o_SIU_parameter      (MonU_SIU_parameter),
                                                                                                          .o_status             (MonU_status),
                                                                                                          .S_AXI_ACLK           (i_SYSTEM_clk),
                                                                                                          .S_AXI_ARESETN        (i_SYSTEM_rst),
                                                                                                          .S_AXI_AWADDR         (S_AXI_AWADDR),
                                                                                                          .S_AXI_AWPROT         (S_AXI_AWPROT),
                                                                                                          .S_AXI_AWVALID        (S_AXI_AWVALID),
                                                                                                          .S_AXI_AWREADY        (S_AXI_AWREADY),
                                                                                                          .S_AXI_WDATA          (S_AXI_WDATA),
                                                                                                          .S_AXI_WSTRB          (S_AXI_WSTRB),
                                                                                                          .S_AXI_WVALID         (S_AXI_WVALID),
                                                                                                          .S_AXI_WREADY         (S_AXI_WREADY),
                                                                                                          .S_AXI_BRESP          (S_AXI_BRESP),
                                                                                                          .S_AXI_BVALID         (S_AXI_BVALID),
                                                                                                          .S_AXI_BREADY         (S_AXI_BREADY),
                                                                                                          .S_AXI_ARADDR         (S_AXI_ARADDR),
                                                                                                          .S_AXI_ARPROT         (S_AXI_ARPROT),
                                                                                                          .S_AXI_ARVALID        (S_AXI_ARVALID),
                                                                                                          .S_AXI_ARREADY        (S_AXI_ARREADY),
                                                                                                          .S_AXI_RDATA          (S_AXI_RDATA),
                                                                                                          .S_AXI_RRESP          (S_AXI_RRESP),
                                                                                                          .S_AXI_RVALID         (S_AXI_RVALID),
                                                                                                          .S_AXI_RREADY         (S_AXI_RREADY)
      );
    end

  endgenerate

endmodule
