`timescale 1ns / 1ps
/*
 * Copyright 2023 ALFA Project. All rights reserved.
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

module alib_extmem (
    input wire [20:0] i_blockAddress,
    input wire [63:0] i_blockPayload,
    input wire [31:0] i_alib_extmem_base_addr,
    input wire [0:0] i_initWriteTxn,
    output wire o_writeTxnDone,
    output o_error,

    input wire M_AXI_ACLK,
    input wire M_AXI_ARESETN,
    output wire [5:0] M_AXI_AWID,
    output wire [31:0] M_AXI_AWADDR,
    output wire [7:0] M_AXI_AWLEN,
    output wire [2:0] M_AXI_AWSIZE,
    output wire [1:0] M_AXI_AWBURST,
    output wire [0:0] M_AXI_AWLOCK,
    output wire [3:0] M_AXI_AWCACHE,
    output wire [2:0] M_AXI_AWPROT,
    output wire [3:0] M_AXI_AWQOS,
    output wire [0:0] M_AXI_AWVALID,
    input wire [0:0] M_AXI_AWREADY,
    output wire [63:0] M_AXI_WDATA,
    output wire [7:0] M_AXI_WSTRB,
    output wire [0:0] M_AXI_WLAST,
    output wire [0:0] M_AXI_WVALID,
    input wire [0:0] M_AXI_WREADY,
    input wire [0:0] M_AXI_BID,
    input wire [1:0] M_AXI_BRESP,
    input wire [0:0] M_AXI_BVALID,
    output wire [0:0] M_AXI_BREADY,
    output wire [5:0] M_AXI_ARID,
    output wire [31:0] M_AXI_ARADDR,
    output wire [7:0] M_AXI_ARLEN,
    output wire [2:0] M_AXI_ARSIZE,
    output wire [1:0] M_AXI_ARBURST,
    output wire [0:0] M_AXI_ARLOCK,
    output wire [3:0] M_AXI_ARCACHE,
    output wire [2:0] M_AXI_ARPROT,
    output wire [3:0] M_AXI_ARQOS,
    output wire [0:0] M_AXI_ARVALID,
    input wire [0:0] M_AXI_ARREADY,
    input wire [0:0] M_AXI_RID,
    input wire [63:0] M_AXI_RDATA,
    input wire [1:0] M_AXI_RRESP,
    input wire [0:0] M_AXI_RLAST,
    input wire [0:0] M_AXI_RVALID,
    output wire [0:0] M_AXI_RREADY
);

  function integer clogb2(input integer bit_depth);
    begin
      for (clogb2 = 0; bit_depth > 0; clogb2 = clogb2 + 1) bit_depth = bit_depth >> 1;
    end
  endfunction

  localparam [1:0] IDLE = 2'b00, INIT_TX = 2'b01;

  reg [1:0] mst_exec_state_write;

  localparam integer BURST_LEN = 1;  // Burst Length

  localparam integer C_TRANSACTIONS_NUM_WRITE = clogb2(BURST_LEN - 1);
  localparam integer C_MASTER_LENGTH_WRITE = 3;
  localparam integer C_NO_BURSTS_REQ_WRITE = C_MASTER_LENGTH_WRITE - clogb2(
      (BURST_LEN * 64 / 8) - 1
  );

  reg [31:0] axi_awaddr;
  reg axi_awvalid;
  reg [63:0] axi_wdata;
  reg axi_wlast;
  reg axi_wvalid;
  reg axi_bready;
  reg [31:0] axi_araddr;
  reg axi_arvalid;
  reg axi_rready;
  reg [C_TRANSACTIONS_NUM_WRITE:0] write_index;
  wire [C_TRANSACTIONS_NUM_WRITE+2:0] burst_size_bytes_write;
  reg [C_NO_BURSTS_REQ_WRITE:0] write_burst_counter;
  reg start_single_burst_write;
  reg writes_done;
  reg error_reg;
  reg burst_write_active;
  wire write_resp_error;
  wire wnext;

  assign M_AXI_AWID = 'b0;
  assign M_AXI_AWADDR = axi_awaddr + i_alib_extmem_base_addr;
  assign M_AXI_AWLEN = BURST_LEN - 1;
  assign M_AXI_AWSIZE = clogb2((64 / 8) - 1);
  assign M_AXI_AWBURST = 2'b00;
  assign M_AXI_AWLOCK = 1'b0;
  assign M_AXI_AWCACHE = 4'b0000;
  assign M_AXI_AWPROT = 3'h0;
  assign M_AXI_AWQOS = 4'h0;
  assign M_AXI_AWVALID = axi_awvalid;
  assign M_AXI_WDATA = axi_wdata;
  assign M_AXI_WSTRB = {(64 / 8) {1'b1}};
  assign M_AXI_WLAST = axi_wlast;
  assign M_AXI_WVALID = axi_wvalid;
  assign M_AXI_BREADY = axi_bready;

  assign M_AXI_ARID = 'b0;
  assign M_AXI_ARADDR = 0;
  assign M_AXI_ARLEN = 0;
  assign M_AXI_ARSIZE = clogb2((64 / 8) - 1);
  assign M_AXI_ARBURST = 2'b01;
  assign M_AXI_ARLOCK = 1'b0;
  assign M_AXI_ARCACHE = 4'b0000;
  assign M_AXI_ARPROT = 3'h0;
  assign M_AXI_ARQOS = 4'h0;
  assign M_AXI_ARVALID = 0;
  assign M_AXI_RREADY = 0;

  assign burst_size_bytes_write = BURST_LEN * 64 / 8;
  assign o_writeTxnDone = writes_done;
  assign o_error = error_reg;

  wire [31:0] id_to_write;

  assign id_to_write[3:0]   = 4'b00;
  assign id_to_write[24:4]  = i_blockAddress;
  assign id_to_write[31:22] = 0;

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_awvalid <= 1'b0;
    else if (~axi_awvalid && start_single_burst_write) axi_awvalid <= 1'b1;
    else if (M_AXI_AWREADY && axi_awvalid) axi_awvalid <= 1'b0;
    else axi_awvalid <= axi_awvalid;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_awaddr <= id_to_write;
    else if (M_AXI_AWREADY && axi_awvalid) axi_awaddr <= axi_awaddr;
    else axi_awaddr <= axi_awaddr;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_wdata <= i_blockPayload;
    else axi_wdata <= axi_wdata;
  end

  assign wnext = M_AXI_WREADY & axi_wvalid;

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_wvalid <= 1'b0;
    else if (~axi_wvalid && start_single_burst_write) axi_wvalid <= 1'b1;
    else if (wnext && axi_wlast) axi_wvalid <= 1'b0;
    else axi_wvalid <= axi_wvalid;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_wlast <= 1'b0;
    else if (((write_index == BURST_LEN - 2 && BURST_LEN >= 2) && wnext) || (BURST_LEN == 1))
      axi_wlast <= 1'b1;
    else if (wnext) axi_wlast <= 1'b0;
    else if (axi_wlast && BURST_LEN == 1) axi_wlast <= 1'b0;
    else axi_wlast <= axi_wlast;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1 || start_single_burst_write == 1'b1)
      write_index <= 0;
    else if (wnext && (write_index != BURST_LEN - 1)) write_index <= write_index + 1;
    else write_index <= write_index;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_bready <= 1'b0;
    else if (M_AXI_BVALID && ~axi_bready) axi_bready <= 1'b1;
    else if (axi_bready) axi_bready <= 1'b0;
    else axi_bready <= axi_bready;
  end

  assign write_resp_error = axi_bready & M_AXI_BVALID;

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) error_reg <= 1'b0;
    else if (write_resp_error) error_reg <= 1'b1;
    else error_reg <= error_reg;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) write_burst_counter <= 'b0;
    else if (M_AXI_AWREADY && axi_awvalid)
      if (write_burst_counter[C_NO_BURSTS_REQ_WRITE] == 1'b0)
        write_burst_counter <= write_burst_counter + 1'b1;
      else write_burst_counter <= write_burst_counter;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 1'b0) begin
      mst_exec_state_write <= IDLE;
      start_single_burst_write <= 1'b0;
    end else begin
      case (mst_exec_state_write)
        IDLE:
        if (i_initWriteTxn == 1'b1) mst_exec_state_write <= INIT_TX;
        else mst_exec_state_write <= IDLE;
        INIT_TX: begin
          if (writes_done) mst_exec_state_write <= IDLE;
          else mst_exec_state_write <= INIT_TX;
          if (~axi_awvalid && ~start_single_burst_write && ~burst_write_active)
            start_single_burst_write <= 1'b1;
          else start_single_burst_write <= 1'b0;
        end
        default: mst_exec_state_write <= IDLE;
      endcase
    end
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) burst_write_active <= 1'b0;
    else if (start_single_burst_write) burst_write_active <= 1'b1;
    else if (M_AXI_BVALID && axi_bready) burst_write_active <= 0;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) writes_done <= 1'b0;
    else if (M_AXI_BVALID && (write_burst_counter[C_NO_BURSTS_REQ_WRITE]) && axi_bready)
      writes_done <= 1'b1;
    else writes_done <= writes_done;
  end

endmodule

