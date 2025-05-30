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

`timescale 1 ns / 1 ps

/* Module: MemMU_P
    Memory Management Unit Pointcloud::

    The MemMU Representation configuration module makes ALFA handle multiple point clouds representations. *TODO*. 
*/
module MonU #(

    /* Parameters integer: Pointcloud.

        MEMORY_OFFSET = 0;
        POINTCLOUD_ID - Unique sensor ID.
        NUMBER_OF_REPRESENTATIONS - Number of representations configurated in ALFA.
        REPRESENTATION_TYPE - *TODO*
        REPRESENTATION_PARAM1 - *TODO*
        REPRESENTATION_PARAM2 - *TODO*
        REPRESENTATION_PARAM3 - *TODO*
        REPRESENTATION_PARAM4 - *TODO*
    */
    parameter integer DATA_WIDTH = 32,
    parameter integer ADDR_WIDTH = 32
) (
    input  wire [DATA_WIDTH-1:0] i_CU_status,
    input  wire [DATA_WIDTH-1:0] i_MemMU_status,
    input  wire [DATA_WIDTH-1:0] i_ExMU_status,
    input  wire [DATA_WIDTH-1:0] i_MonU_status,
    input  wire [DATA_WIDTH-1:0] i_SIU_status,
    input  wire [           0:0] i_CU_pcReady,
    output wire [           0:0] o_CU_pcReady,
    input  wire [           0:0] i_CU_processingDone,
    output wire [           0:0] o_CU_processingDone,
    output wire [DATA_WIDTH-1:0] o_MonU_PointCloudSize,
    input  wire [           0:0] i_CU_extReady,
    output wire [           0:0] o_CU_frameDone,
    input  wire [DATA_WIDTH-1:0] i_EXT_status,
    input  wire [DATA_WIDTH-1:0] i_DP_0,
    i_DP_1,
    i_DP_2,
    i_DP_3,
    i_DP_4,
    i_DP_5,
    i_DP_6,
    i_DP_7,
    i_DP_8,
    i_DP_9,
    i_DP_10,
    i_DP_11,
    i_DP_12,
    i_DP_13,
    i_DP_14,
    i_DP_15,
    i_DP_16,
    i_DP_17,
    i_DP_18,
    i_DP_19,
    output wire [DATA_WIDTH-1:0] o_USER_0,
    o_USER_1,
    o_USER_2,
    o_USER_3,
    o_USER_4,
    o_USER_5,
    o_USER_6,
    o_USER_7,
    o_USER_8,
    o_USER_9,
    output wire [DATA_WIDTH-1:0] o_CU_parameter,
    output wire [DATA_WIDTH-1:0] o_MemMU_parameter,
    output wire [DATA_WIDTH-1:0] o_ExMU_parameter,
    output wire [DATA_WIDTH-1:0] o_MonU_parameter,
    output wire [DATA_WIDTH-1:0] o_SIU_parameter,
    output reg  [DATA_WIDTH-1:0] o_status,

    // AXI interface
    input  wire [           0:0] S_AXI_ACLK,
    input  wire [           0:0] S_AXI_ARESETN,
    input  wire [ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire [           2:0] S_AXI_AWPROT,
    input  wire [           0:0] S_AXI_AWVALID,
    output wire [           0:0] S_AXI_AWREADY,

    // Write data (issued by master, acceped by Slave) 
    input  wire [    DATA_WIDTH-1:0] S_AXI_WDATA,
    input  wire [(DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire [               0:0] S_AXI_WVALID,
    output wire [               0:0] S_AXI_WREADY,
    output wire [               1:0] S_AXI_BRESP,
    output wire [               0:0] S_AXI_BVALID,
    input  wire [               0:0] S_AXI_BREADY,

    // Read address (issued by master, acceped by Slave)
    input  wire [ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire [           2:0] S_AXI_ARPROT,
    input  wire [           0:0] S_AXI_ARVALID,
    output wire [           0:0] S_AXI_ARREADY,

    // Read data (issued by slave)
    output wire [DATA_WIDTH-1 : 0] S_AXI_RDATA,
    output wire [             1:0] S_AXI_RRESP,
    output wire [             0:0] S_AXI_RVALID,
    input  wire [             0:0] S_AXI_RREADY
);


  // AXI4LITE signals
  reg [ADDR_WIDTH -1 : 0] axi_awaddr;
  reg                     axi_awready;
  reg                     axi_wready;
  reg [            1 : 0] axi_bresp;
  reg                     axi_bvalid;
  reg [ ADDR_WIDTH-1 : 0] axi_araddr;
  reg                     axi_arready;
  reg [ DATA_WIDTH-1 : 0] axi_rdata;
  reg [            1 : 0] axi_rresp;
  reg                     axi_rvalid;

  // Example-specific design signals
  // local parameter for addressing 32 bit / 64 bit DATA_WIDTH
  // ADDR_LSB is used for addressing 32/64 bit registers/memories
  // ADDR_LSB = 2 for 32 bits (n downto 2)
  // ADDR_LSB = 3 for 64 bits (n downto 3)
  localparam integer ADDR_LSB = (DATA_WIDTH / 32) + 1;
  localparam integer OPT_MEM_ADDR_BITS = 7;

  //----------------------------------------------
  //-- Signals for user logic register space example
  //------------------------------------------------
  //-- Number of Slave Registers 50
  reg     [DATA_WIDTH-1:0] slv_reg      [49:0];
  wire                     slv_reg_rden;
  wire                     slv_reg_wren;
  reg     [DATA_WIDTH-1:0] reg_data_out;
  integer                  byte_index;
  reg                      aw_en;

  // I/O Connections assignments
  assign S_AXI_AWREADY         = axi_awready;
  assign S_AXI_WREADY          = axi_wready;
  assign S_AXI_BRESP           = axi_bresp;
  assign S_AXI_BVALID          = axi_bvalid;
  assign S_AXI_ARREADY         = axi_arready;
  assign S_AXI_RDATA           = axi_rdata;
  assign S_AXI_RRESP           = axi_rresp;
  assign S_AXI_RVALID          = axi_rvalid;

  assign o_CU_parameter        = slv_reg[30];
  assign o_MemMU_parameter     = slv_reg[31];
  assign o_ExMU_parameter      = slv_reg[32];
  assign o_MonU_parameter      = slv_reg[33];
  assign o_SIU_parameter       = slv_reg[34];
  assign o_CU_pcReady          = (slv_reg[35] == 0) ? 1'b0 : 1'b1;
  assign o_CU_processingDone   = (slv_reg[36] == 0) ? 1'b0 : 1'b1;
  assign o_MonU_PointCloudSize = slv_reg[37];
  assign o_CU_frameDone        = (slv_reg[38] == 0) ? 1'b0 : 1'b1;
  assign o_USER_0              = slv_reg[40];
  assign o_USER_1              = slv_reg[41];
  assign o_USER_2              = slv_reg[42];
  assign o_USER_3              = slv_reg[43];
  assign o_USER_4              = slv_reg[44];
  assign o_USER_5              = slv_reg[45];
  assign o_USER_6              = slv_reg[46];
  assign o_USER_7              = slv_reg[47];
  assign o_USER_8              = slv_reg[48];
  assign o_USER_9              = slv_reg[49];


  // axi_awready generation
  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      axi_awready <= 1'b0;
      aw_en <= 1'b1;
    end else begin
      if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
        // slave is ready to accept write address when 
        // there is a valid write address and write data
        // on the write address and data bus. This design 
        // expects no outstanding transactions. 
        axi_awready <= 1'b1;
        aw_en <= 1'b0;
      end else if (S_AXI_BREADY && axi_bvalid) begin
        aw_en <= 1'b1;
        axi_awready <= 1'b0;
      end else begin
        axi_awready <= 1'b0;
      end
    end
  end

  // axi_awaddr latching
  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      axi_awaddr <= 0;
    end else begin
      if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
        // Write Address latching 
        axi_awaddr <= S_AXI_AWADDR;
      end
    end
  end

  // axi_wready generation
  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      axi_wready <= 1'b0;
    end else begin
      if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en) begin
        // slave is ready to accept write data when 
        // there is a valid write address and write data
        // on the write address and data bus. This design 
        // expects no outstanding transactions. 
        axi_wready <= 1'b1;
      end else begin
        axi_wready <= 1'b0;
      end
    end
  end

  // Memory mapped register select and write logic generation for status and user dubug
  assign slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

  integer i;

  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      for (i = 0; i < 30; i = i + 1) begin
        slv_reg[i] <= 0;
      end
      o_status <= 1'b0;
    end else begin
      slv_reg[0] <= i_CU_status;
      slv_reg[1] <= i_ExMU_status;
      slv_reg[2] <= i_MemMU_status;
      slv_reg[3] <= i_MonU_status;
      slv_reg[4] <= i_SIU_status;
      slv_reg[5] <= i_EXT_status;
      slv_reg[6] <= 32'b0;
      slv_reg[7] <= i_CU_extReady;
      slv_reg[8] <= i_CU_pcReady;
      slv_reg[9] <= i_CU_processingDone;
      slv_reg[10] <= i_DP_0;
      slv_reg[11] <= i_DP_1;
      slv_reg[12] <= i_DP_2;
      slv_reg[13] <= i_DP_3;
      slv_reg[14] <= i_DP_4;
      slv_reg[15] <= i_DP_5;
      slv_reg[16] <= i_DP_6;
      slv_reg[17] <= i_DP_7;
      slv_reg[18] <= i_DP_8;
      slv_reg[19] <= i_DP_9;
      slv_reg[20] <= i_DP_10;
      slv_reg[21] <= i_DP_11;
      slv_reg[22] <= i_DP_12;
      slv_reg[23] <= i_DP_13;
      slv_reg[24] <= i_DP_14;
      slv_reg[25] <= i_DP_15;
      slv_reg[26] <= i_DP_16;
      slv_reg[27] <= i_DP_17;
      slv_reg[28] <= i_DP_18;
      slv_reg[29] <= i_DP_19;
      o_status <= 1'b1;
    end
  end

  // Memory mapped register select and write logic generation for parameters and signals from software
  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      for (i = 30; i < 50; i = i + 1) slv_reg[i] <= 0;
    end else if (slv_reg_wren) begin
      case (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
        8'h1E:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[30][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h1F:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[31][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h20:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[32][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h21:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[33][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h22:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[34][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h23:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[35][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h24:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[36][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h25:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[37][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h26:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[38][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h27:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[39][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h28:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[40][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h29:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[41][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h2A:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[42][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h2B:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[43][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h2C:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[44][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h2D:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[45][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h2E:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[46][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h2F:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[47][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h30:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[48][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];
        8'h31:
        for (byte_index = 0; byte_index <= (DATA_WIDTH / 8) - 1; byte_index = byte_index + 1)
        if (S_AXI_WSTRB[byte_index] == 1)
          slv_reg[49][(byte_index*8)+:8] <= S_AXI_WDATA[(byte_index*8)+:8];

        default: begin
          for (i = 0; i < 50; i = i + 1) slv_reg[i] <= slv_reg[i];
        end
      endcase
    end
  end

  // Write response logic generation
  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      axi_bvalid <= 0;
      axi_bresp  <= 2'b0;
    end else begin
      if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID) begin
        // indicates a valid write response is available
        axi_bvalid <= 1'b1;
        axi_bresp  <= 2'b0;  // 'OKAY' response 
      end                   // work error responses in future
        else
        begin
        if (S_AXI_BREADY && axi_bvalid) 
            //check if bready is asserted while bvalid is high) 
            //(there is a possibility that bready is always asserted high)   
            begin
          axi_bvalid <= 1'b0;
        end
      end
    end
  end

  // axi_arready generation
  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      axi_arready <= 1'b0;
      axi_araddr  <= 32'b0;
    end else begin
      if (~axi_arready && S_AXI_ARVALID) begin
        // indicates that the slave has acceped the valid read address
        axi_arready <= 1'b1;
        // Read address latching
        axi_araddr  <= S_AXI_ARADDR;
      end else begin
        axi_arready <= 1'b0;
      end
    end
  end

  // axi_arvalid generation
  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      axi_rvalid <= 0;
      axi_rresp  <= 0;
    end else begin
      if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) begin
        // Valid read data is available at the read data bus
        axi_rvalid <= 1'b1;
        axi_rresp  <= 2'b0;  // 'OKAY' response
      end else if (axi_rvalid && S_AXI_RREADY) begin
        // Read data is accepted by the master
        axi_rvalid <= 1'b0;
      end
    end
  end

  // Memory mapped register select and read logic generation
  assign slv_reg_rden = axi_arready & S_AXI_ARVALID & ~axi_rvalid;
  always @(*) begin
    // Address decoding for reading registers
    case (axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
      8'h00:   reg_data_out <= slv_reg[0];
      8'h01:   reg_data_out <= slv_reg[1];
      8'h02:   reg_data_out <= slv_reg[2];
      8'h03:   reg_data_out <= slv_reg[3];
      8'h04:   reg_data_out <= slv_reg[4];
      8'h05:   reg_data_out <= slv_reg[5];
      8'h06:   reg_data_out <= slv_reg[6];
      8'h07:   reg_data_out <= slv_reg[7];
      8'h08:   reg_data_out <= slv_reg[8];
      8'h09:   reg_data_out <= slv_reg[9];
      8'h0A:   reg_data_out <= slv_reg[10];
      8'h0B:   reg_data_out <= slv_reg[11];
      8'h0C:   reg_data_out <= slv_reg[12];
      8'h0D:   reg_data_out <= slv_reg[13];
      8'h0E:   reg_data_out <= slv_reg[14];
      8'h0F:   reg_data_out <= slv_reg[15];
      8'h10:   reg_data_out <= slv_reg[16];
      8'h11:   reg_data_out <= slv_reg[17];
      8'h12:   reg_data_out <= slv_reg[18];
      8'h13:   reg_data_out <= slv_reg[19];
      8'h14:   reg_data_out <= slv_reg[20];
      8'h15:   reg_data_out <= slv_reg[21];
      8'h16:   reg_data_out <= slv_reg[22];
      8'h17:   reg_data_out <= slv_reg[23];
      8'h18:   reg_data_out <= slv_reg[24];
      8'h19:   reg_data_out <= slv_reg[25];
      8'h1A:   reg_data_out <= slv_reg[26];
      8'h1B:   reg_data_out <= slv_reg[27];
      8'h1C:   reg_data_out <= slv_reg[28];
      8'h1D:   reg_data_out <= slv_reg[29];
      8'h1E:   reg_data_out <= slv_reg[30];
      8'h1F:   reg_data_out <= slv_reg[31];
      8'h20:   reg_data_out <= slv_reg[32];
      8'h21:   reg_data_out <= slv_reg[33];
      8'h22:   reg_data_out <= slv_reg[34];
      8'h23:   reg_data_out <= slv_reg[35];
      8'h24:   reg_data_out <= slv_reg[36];
      8'h25:   reg_data_out <= slv_reg[37];
      8'h26:   reg_data_out <= slv_reg[38];
      8'h27:   reg_data_out <= slv_reg[39];
      8'h28:   reg_data_out <= slv_reg[40];
      8'h29:   reg_data_out <= slv_reg[41];
      8'h2A:   reg_data_out <= slv_reg[42];
      8'h2B:   reg_data_out <= slv_reg[43];
      8'h2C:   reg_data_out <= slv_reg[44];
      8'h2D:   reg_data_out <= slv_reg[45];
      8'h2E:   reg_data_out <= slv_reg[46];
      8'h2F:   reg_data_out <= slv_reg[47];
      8'h30:   reg_data_out <= slv_reg[48];
      8'h31:   reg_data_out <= slv_reg[49];
      default: reg_data_out <= 0;
    endcase
  end

  // Output register or memory read data
  always @(posedge S_AXI_ACLK) begin
    if (S_AXI_ARESETN == 1'b0) begin
      axi_rdata <= 0;
    end else begin
      // When there is a valid read address (S_AXI_ARVALID) with 
      // acceptance of read address by the slave (axi_arready), 
      // output the read dada 
      if (slv_reg_rden) begin
        axi_rdata <= reg_data_out;  // register read data
      end
    end
  end

endmodule
