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

module AXI #(
    parameter C_M_TARGET_SLAVE_BASE_ADDR = 32'h00000000,
    // Burst Length. Supports 1, 2, 4, 8, 16, 32, 64, 128, 256 burst lengths
    parameter integer C_M_AXI_BURST_LEN_WRITE = 32,
    parameter integer C_M_AXI_BURST_LEN_READ = 32,
    // Thread ID Width
    parameter integer C_M_AXI_ID_WIDTH = 6,
    // Width of Address Bus
    parameter integer C_M_AXI_ADDR_WIDTH = 32,
    // Width of Data Bus
    parameter integer C_M_AXI_DATA_WIDTH = 64
    // Width of User Write Address Bus
) (

    input wire [31:0] i_writeAddress,
    input wire [(64*C_M_AXI_BURST_LEN_WRITE)-1:0] i_writePayload,
    input wire [31:0] i_readAddress,
    output wire [(64*C_M_AXI_BURST_LEN_READ)-1:0] o_readPayload,

    // Initiate AXI transactions
    input wire i_initWriteTxn,
    input wire i_initReadTxn,
    // Asserts when transaction is complete
    output wire o_writeTxnDone,
    output wire o_readTxnDone,
    // Asserts when ERROR is detected
    output o_error,
    // Global Clock Signal.
    input wire M_AXI_ACLK,
    // Global Reset Singal. This Signal is Active Low
    input wire M_AXI_ARESETN,
    // Master Interface Write Address ID
    output wire [C_M_AXI_ID_WIDTH-1 : 0] M_AXI_AWID,
    // Master Interface Write Address
    output wire [C_M_AXI_ADDR_WIDTH-1 : 0] M_AXI_AWADDR,
    // Burst length. The burst length gives the exact number of transfers in a burst
    output wire [7 : 0] M_AXI_AWLEN,
    // Burst size. This signal indicates the size of each transfer in the burst
    output wire [2 : 0] M_AXI_AWSIZE,
    // Burst type. The burst type and the size information, 
    // determine how the address for each transfer within the burst is calculated.
    output wire [1 : 0] M_AXI_AWBURST,
    // Lock type. Provides additional information about the
    // atomic characteristics of the transfer.
    output wire M_AXI_AWLOCK,
    // Memory type. This signal indicates how transactions
    // are required to progress through a system.
    output wire [3 : 0] M_AXI_AWCACHE,
    // Protection type. This signal indicates the privilege
    // and security level of the transaction, and whether
    // the transaction is a data access or an instruction access.
    output wire [2 : 0] M_AXI_AWPROT,
    // Quality of Service, QoS identifier sent for each write transaction.
    output wire [3 : 0] M_AXI_AWQOS,
    // Write address valid. This signal indicates that
    // the channel is signaling valid write address and control information.
    output wire M_AXI_AWVALID,
    // Write address ready. This signal indicates that
    // the slave is ready to accept an address and associated control signals
    input wire M_AXI_AWREADY,
    // Master Interface Write Data.
    output wire [C_M_AXI_DATA_WIDTH-1 : 0] M_AXI_WDATA,
    // Write strobes. This signal indicates which byte
    // lanes hold valid data. There is one write strobe
    // bit for each eight bits of the write data bus.
    output wire [C_M_AXI_DATA_WIDTH/8-1 : 0] M_AXI_WSTRB,
    // Write last. This signal indicates the last transfer in a write burst.
    output wire M_AXI_WLAST,
    // Write valid. This signal indicates that valid write
    // data and strobes are available
    output wire M_AXI_WVALID,
    // Write ready. This signal indicates that the slave
    // can accept the write data.
    input wire M_AXI_WREADY,
    // Master Interface Write Response.
    input wire [C_M_AXI_ID_WIDTH-1 : 0] M_AXI_BID,
    // Write response. This signal indicates the status of the write transaction.
    input wire [1 : 0] M_AXI_BRESP,
    // Write response valid. This signal indicates that the
    // channel is signaling a valid write response.
    input wire M_AXI_BVALID,
    // Response ready. This signal indicates that the master
    // can accept a write response.
    output wire M_AXI_BREADY,
    // Master Interface Read Address.
    output wire [C_M_AXI_ID_WIDTH-1 : 0] M_AXI_ARID,
    // Read address. This signal indicates the initial
    // address of a read burst transaction.
    output wire [C_M_AXI_ADDR_WIDTH-1 : 0] M_AXI_ARADDR,
    // Burst length. The burst length gives the exact number of transfers in a burst
    output wire [7 : 0] M_AXI_ARLEN,
    // Burst size. This signal indicates the size of each transfer in the burst
    output wire [2 : 0] M_AXI_ARSIZE,
    // Burst type. The burst type and the size information, 
    // determine how the address for each transfer within the burst is calculated.
    output wire [1 : 0] M_AXI_ARBURST,
    // Lock type. Provides additional information about the
    // atomic characteristics of the transfer.
    output wire M_AXI_ARLOCK,
    // Memory type. This signal indicates how transactions
    // are required to progress through a system.
    output wire [3 : 0] M_AXI_ARCACHE,
    // Protection type. This signal indicates the privilege
    // and security level of the transaction, and whether
    // the transaction is a data access or an instruction access.
    output wire [2 : 0] M_AXI_ARPROT,
    // Quality of Service, QoS identifier sent for each read transaction
    output wire [3 : 0] M_AXI_ARQOS,
    // Write address valid. This signal indicates that
    // the channel is signaling valid read address and control information
    output wire M_AXI_ARVALID,
    // Read address ready. This signal indicates that
    // the slave is ready to accept an address and associated control signals
    input wire M_AXI_ARREADY,
    // Read ID tag. This signal is the identification tag
    // for the read data group of signals generated by the slave.
    input wire [C_M_AXI_ID_WIDTH-1 : 0] M_AXI_RID,
    // Master Read Data
    input wire [C_M_AXI_DATA_WIDTH-1 : 0] M_AXI_RDATA,
    // Read response. This signal indicates the status of the read transfer
    input wire [1 : 0] M_AXI_RRESP,
    // Read last. This signal indicates the last transfer in a read burst
    input wire M_AXI_RLAST,
    // Read valid. This signal indicates that the channel
    // is signaling the required read data.
    input wire M_AXI_RVALID,
    // Read ready. This signal indicates that the master can
    // accept the read data and response information.
    output wire M_AXI_RREADY
);

  function integer clogb2(input integer bit_depth);
    begin
      for (clogb2 = 0; bit_depth > 0; clogb2 = clogb2 + 1) bit_depth = bit_depth >> 1;
    end
  endfunction


  localparam integer C_TRANSACTIONS_NUM_WRITE = clogb2(C_M_AXI_BURST_LEN_WRITE - 1);
  localparam integer C_TRANSACTIONS_NUM_READ = clogb2(C_M_AXI_BURST_LEN_READ - 1);
  //0 Burst length for transactions, in C_M_AXI_DATA_WIDTHs.
  // Non-2^n lengths will eventually cause bursts across 4K address boundaries.
  localparam integer C_MASTER_LENGTH_WRITE = 8;
  localparam integer C_MASTER_LENGTH_READ = 8;
  // total number of burst transfers is master length divided by burst length and burst size
  localparam integer C_NO_BURSTS_REQ_WRITE = C_MASTER_LENGTH_WRITE - clogb2(
      (C_M_AXI_BURST_LEN_WRITE * C_M_AXI_DATA_WIDTH / 8) - 1
  );
  localparam integer C_NO_BURSTS_REQ_READ = C_MASTER_LENGTH_READ - clogb2(
      (C_M_AXI_BURST_LEN_READ * C_M_AXI_DATA_WIDTH / 8) - 1
  );

  // Example State machine to initialize write transactions
  localparam [1:0] IDLE = 2'b00,  // This state initiates AXI4Lite transaction 
  // after the state machine changes state to INIT_WRITE   
  // when there is 0 to 1 transition on INIT_AXI_TXN
  INIT_TX = 2'b01;  // This state initializes transaction,

  reg     [                           1:0] mst_exec_state_write;
  reg     [                           1:0] mst_exec_state_read;
  reg     [                          63:0] point_reg                [0:(C_M_AXI_BURST_LEN_READ-1)];

  ///AXI4 internal temp signals
  reg     [      C_M_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
  reg                                      axi_awvalid;
  reg     [      C_M_AXI_DATA_WIDTH-1 : 0] axi_wdata;
  reg                                      axi_wlast;
  reg                                      axi_wvalid;
  reg                                      axi_bready;
  reg     [      C_M_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
  reg                                      axi_arvalid;
  reg                                      axi_rready;
  //write beat count in a burst
  reg     [  C_TRANSACTIONS_NUM_WRITE : 0] write_index;
  //read beat count in a burst
  reg     [   C_TRANSACTIONS_NUM_READ : 0] read_index;
  //size of C_M_AXI_BURST_LEN length burst in bytes
  wire    [C_TRANSACTIONS_NUM_WRITE+2 : 0] burst_size_bytes_write;
  wire    [ C_TRANSACTIONS_NUM_READ+2 : 0] burst_size_bytes_read;
  //The burst counters are used to track the number of burst transfers of C_M_AXI_BURST_LEN burst length needed to transfer 2^C_MASTER_LENGTH bytes of data.
  reg     [     C_NO_BURSTS_REQ_WRITE : 0] write_burst_counter;
  reg     [      C_NO_BURSTS_REQ_READ : 0] read_burst_counter;
  reg                                      start_single_burst_write;
  reg                                      start_single_burst_read;
  reg                                      writes_done;
  reg                                      reads_done;
  reg                                      error_reg;
  reg                                      burst_write_active;
  reg                                      burst_read_active;
  //Interface response error flags
  wire                                     write_resp_error;
  wire                                     read_resp_error;
  wire                                     wnext;

  integer                                  i;

  //I/O Connections. Write Address (AW)
  assign M_AXI_AWID = 'b0;
  //The AXI address is a concatenation of the target base address + active offset range
  assign M_AXI_AWADDR = C_M_TARGET_SLAVE_BASE_ADDR + axi_awaddr;
  //Burst LENgth is number of transaction beats, minus 1
  assign M_AXI_AWLEN = C_M_AXI_BURST_LEN_WRITE - 1;
  //Size should be C_M_AXI_DATA_WIDTH, in 2^SIZE bytes, otherwise narrow bursts are used
  assign M_AXI_AWSIZE = clogb2((C_M_AXI_DATA_WIDTH / 8) - 1);
  //INCR burst type is usually used, except for keyhole bursts
  assign M_AXI_AWBURST = 2'b01;
  assign M_AXI_AWLOCK = 1'b0;
  //Update value to 4'b0011 if coherent accesses to be used via the Zynq ACP port. Not Allocated, Modifiable, not Bufferable. Not Bufferable since this example is meant to test memory, not intermediate cache. 
  assign M_AXI_AWCACHE = 4'b0000;
  assign M_AXI_AWPROT = 3'h0;
  assign M_AXI_AWQOS = 4'h0;
  assign M_AXI_AWVALID = axi_awvalid;
  //Write Data(W)
  assign M_AXI_WDATA = axi_wdata;
  //All bursts are complete and aligned in this example
  assign M_AXI_WSTRB = {(C_M_AXI_DATA_WIDTH / 8) {1'b1}};
  assign M_AXI_WLAST = axi_wlast;
  assign M_AXI_WVALID = axi_wvalid;
  //Write Response (B)
  assign M_AXI_BREADY = axi_bready;

  //Read Address (AR)
  assign M_AXI_ARID = 'b0;
  assign M_AXI_ARADDR = axi_araddr;
  //Burst LENgth is number of transaction beats, minus 1
  assign M_AXI_ARLEN = C_M_AXI_BURST_LEN_READ - 1;
  //Size should be C_M_AXI_DATA_WIDTH, in 2^n bytes, otherwise narrow bursts are used
  assign M_AXI_ARSIZE = clogb2((C_M_AXI_DATA_WIDTH / 8) - 1);
  //INCR burst type is usually used, except for keyhole bursts
  assign M_AXI_ARBURST = 2'b01;
  assign M_AXI_ARLOCK = 1'b0;
  //Update value to 4'b0011 if coherent accesses to be used via the Zynq ACP port. Not Allocated, Modifiable, not Bufferable. Not Bufferable since this example is meant to test memory, not intermediate cache. 
  assign M_AXI_ARCACHE = 4'b0000;
  assign M_AXI_ARPROT = 3'h0;
  assign M_AXI_ARQOS = 4'h0;
  assign M_AXI_ARVALID = axi_arvalid;
  //Read and Read Response (R)
  assign M_AXI_RREADY = axi_rready;

  //Burst size in bytes
  assign burst_size_bytes_read = C_M_AXI_BURST_LEN_READ * C_M_AXI_DATA_WIDTH / 8;
  assign burst_size_bytes_write = C_M_AXI_BURST_LEN_WRITE * C_M_AXI_DATA_WIDTH / 8;

  assign o_writeTxnDone = writes_done;
  assign o_readTxnDone = reads_done;

  assign o_error = error_reg;

  wire [63:0] point_write[0:(C_M_AXI_BURST_LEN_WRITE-1)];

  generate
    genvar vargen;
    for (vargen = 0; vargen < C_M_AXI_BURST_LEN_READ; vargen = vargen + 1) begin
      assign o_readPayload[64*vargen+63:64*vargen] = point_reg[vargen];
    end
    for (vargen = 0; vargen < C_M_AXI_BURST_LEN_WRITE; vargen = vargen + 1) begin
      assign point_write[vargen] = i_writePayload[64*vargen+63:64*vargen];
    end
  endgenerate

  //--------------------
  //Write Address Channel
  //--------------------

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_awvalid <= 1'b0;
    else if (~axi_awvalid && start_single_burst_write)
      axi_awvalid <= 1'b1;    		// If previously not valid , start next transaction                                                                                                                                           
    else if (M_AXI_AWREADY && axi_awvalid)
      axi_awvalid <= 1'b0;       // Once asserted, VALIDs cannot be deasserted, so axi_awvalid must wait until transaction is accepted                                                                                                                    
    else axi_awvalid <= axi_awvalid;
  end

  // Next address after AWREADY indicates previous address acceptance    
  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_awaddr <= i_writeAddress;
    else if (M_AXI_AWREADY && axi_awvalid) axi_awaddr <= axi_awaddr;
    else axi_awaddr <= axi_awaddr;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_wdata <= point_write[0];
    else if (wnext) axi_wdata <= point_write[write_index+1];
    else axi_wdata <= axi_wdata;
  end

  assign wnext = M_AXI_WREADY & axi_wvalid;

  // WVALID logic, similar to the axi_awvalid always block above                      
  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_wvalid <= 1'b0;
    else if (~axi_wvalid && start_single_burst_write)
      axi_wvalid <= 1'b1;   // If previously not valid, start next transaction                                                                                                                                                                 
    else if (wnext && axi_wlast)
      axi_wvalid <= 1'b0;  // Once asserted, VALIDs cannot be deasserted, so WVALID must wait until burst is complete with WLAST                                                        
    else axi_wvalid <= axi_wvalid;
  end

  // WLAST generation on the MSB of a counter underflow                                                    
  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_wlast <= 1'b0;
    else if (((write_index == C_M_AXI_BURST_LEN_WRITE-2 && C_M_AXI_BURST_LEN_WRITE >= 2) && wnext) || (C_M_AXI_BURST_LEN_WRITE == 1 ))
      axi_wlast <= 1'b1;
    else if (wnext) axi_wlast <= 1'b0;
    else if (axi_wlast && C_M_AXI_BURST_LEN_WRITE == 1) axi_wlast <= 1'b0;
    else axi_wlast <= axi_wlast;
  end


  // Burst length counter. Uses extra counter register bit to indicate terminal count to reduce decode logic                                                   
  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1 || start_single_burst_write == 1'b1)
      write_index <= 0;
    else if (wnext && (write_index != C_M_AXI_BURST_LEN_WRITE - 1)) write_index <= write_index + 1;
    else write_index <= write_index;
  end


  //----------------------------
  // Write Response (B) Channel
  //----------------------------

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) axi_bready <= 1'b0;
    else if (M_AXI_BVALID && ~axi_bready) axi_bready <= 1'b1;
    else if (axi_bready) axi_bready <= 1'b0;
    else axi_bready <= axi_bready;
  end


  //Flag any write response errors                                        
  assign write_resp_error = axi_bready & M_AXI_BVALID;

  //----------------------------
  // Read Address Channel
  //----------------------------

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initReadTxn == 1'b1) axi_arvalid <= 1'b0;
    else if (~axi_arvalid && start_single_burst_read) // If previously not valid , start next transaction
      axi_arvalid <= 1'b1;
    else if (M_AXI_ARREADY && axi_arvalid) axi_arvalid <= 1'b0;
    else axi_arvalid <= axi_arvalid;
  end

  // Next address after ARREADY indicates previous address acceptance  
  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initReadTxn == 1'b1) axi_araddr <= i_readAddress;
    else if (M_AXI_ARREADY && axi_arvalid) axi_araddr <= axi_araddr + burst_size_bytes_read;
    else axi_araddr <= axi_araddr;
  end


  //--------------------------------
  // Read Data (and Response) Channel
  //--------------------------------

  assign rnext = M_AXI_RVALID && axi_rready;

  // Burst length counter. Uses extra counter register bit to indicate terminal count to reduce decode logic                                
  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initReadTxn == 1'b1 || start_single_burst_read) read_index <= 0;
    else if (rnext && (read_index != C_M_AXI_BURST_LEN_READ - 1)) read_index <= read_index + 1;
    else read_index <= read_index;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initReadTxn == 1'b1) axi_rready <= 1'b0;
    else if (M_AXI_RVALID) begin // accept/acknowledge rdata/rresp with axi_rready by the master when M_AXI_RVALID is asserted by slave                                                                 
      if (M_AXI_RLAST && axi_rready) axi_rready <= 1'b0;
      else axi_rready <= 1'b1;
    end else axi_rready <= axi_rready;  // retain the previous value                 
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initReadTxn == 1'b1)
      for (i = 0; i < C_M_AXI_BURST_LEN_READ; i = i + 1) point_reg[i] <= 0;
    else if (rnext)
      point_reg[read_index] <= M_AXI_RDATA; //Only check data when RVALID is active                                                                                                                                               
  end

  assign read_resp_error = axi_rready & M_AXI_RVALID & M_AXI_RRESP[1]; //Flag any read response errors                                        

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1 || i_initReadTxn == 1'b1) error_reg <= 1'b0;
    else if (write_resp_error || read_resp_error) error_reg <= 1'b1;
    else error_reg <= error_reg;
  end

  //--------------------------------
  // Throttling
  //--------------------------------

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) write_burst_counter <= 'b0;
    else if (M_AXI_AWREADY && axi_awvalid) begin
      if (write_burst_counter[C_NO_BURSTS_REQ_WRITE] == 1'b0)
        write_burst_counter <= write_burst_counter + 1'b1;
    end else write_burst_counter <= write_burst_counter;
  end


  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initReadTxn == 1'b1) read_burst_counter <= 'b0;
    else if (M_AXI_ARREADY && axi_arvalid) begin
      if (read_burst_counter[C_NO_BURSTS_REQ_READ] == 1'b0)
        read_burst_counter <= read_burst_counter + 1'b1;
    end else read_burst_counter <= read_burst_counter;
  end

  always @ ( posedge M_AXI_ACLK) begin  //MASTER_EXECUTION_PROC                                                                                                     
    if (M_AXI_ARESETN == 1'b0) begin
      // reset condition                                                                                  
      // All the signals are assigned default values under reset condition                                
      mst_exec_state_write     <= IDLE;
      mst_exec_state_read      <= IDLE;
      start_single_burst_write <= 1'b0;
      start_single_burst_read  <= 1'b0;
    end else begin
      case (mst_exec_state_write) // state write transition                                                                                

        IDLE: //This state is responsible to wait for user defined C_M_START_COUNT number of clock cycles.                                                                                    
        if (i_initWriteTxn == 1'b1) mst_exec_state_write <= INIT_TX;
        else mst_exec_state_write <= IDLE;

        INIT_TX: // This state is responsible to issue start_single_write pulse to initiate a write transaction. Write transactions will be issued until burst_write_active signal is asserted.                                                                                                                      
        if (writes_done)
          mst_exec_state_write <= IDLE;//                                                                                                                                                    
        else begin
          mst_exec_state_write <= INIT_TX;
          if (~axi_awvalid && ~start_single_burst_write && ~burst_write_active)
            start_single_burst_write <= 1'b1;
          else
            start_single_burst_write <= 1'b0; //Negate to generate a pulse                          																
        end

        default: mst_exec_state_write <= IDLE;
      endcase

      case (mst_exec_state_read)  // state read transition    
        IDLE: //This state is responsible to wait for user defined C_M_START_COUNT number of clock cycles.                                                                                                                                                          
        if (i_initReadTxn == 1'b1) mst_exec_state_read <= INIT_TX;
        else mst_exec_state_read <= IDLE;

        INIT_TX:
        if (reads_done)
          mst_exec_state_read <= IDLE;//                                                                                                                                                       
        else begin
          mst_exec_state_read <= INIT_TX;
          if (~axi_arvalid && ~start_single_burst_read && ~burst_read_active)
            start_single_burst_read <= 1'b1;
          else
            start_single_burst_read <= 1'b0; //Negate to generate a pulse                                                                                                               
        end

        default: mst_exec_state_read <= IDLE;
      endcase
    end
  end



  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initWriteTxn == 1'b1) burst_write_active <= 1'b0;
    else if (start_single_burst_write)
      burst_write_active <= 1'b1; //The burst_write_active is asserted when a write burst transaction is initiated                                                                                                                                              
    else if (M_AXI_BVALID && axi_bready) burst_write_active <= 0;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0) writes_done <= 1'b1;
    else if (i_initWriteTxn == 1'b1) writes_done <= 1'b0;
    else if (M_AXI_BVALID && (write_burst_counter[C_NO_BURSTS_REQ_WRITE]) && axi_bready)
      writes_done <= 1'b1;
    else writes_done <= writes_done;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0 || i_initReadTxn == 1'b1) burst_read_active <= 1'b0;
    else if (start_single_burst_read) burst_read_active <= 1'b1;
    else if (M_AXI_RVALID && axi_rready && M_AXI_RLAST) burst_read_active <= 0;
  end

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0) reads_done <= 1'b1;
    else if (i_initReadTxn == 1'b1) reads_done <= 1'b0;
    else if (M_AXI_RVALID && axi_rready && (read_index == C_M_AXI_BURST_LEN_READ-1) && (read_burst_counter[C_NO_BURSTS_REQ_READ]))
      reads_done <= 1'b1;
    else reads_done <= reads_done;
  end

endmodule

module AXI_single_word_burst #(
    parameter C_M_TARGET_SLAVE_BASE_ADDR = 32'h00000000,
    // Thread ID Width
    parameter integer C_M_AXI_ID_WIDTH = 6,
    // Width of Address Bus
    parameter integer C_M_AXI_ADDR_WIDTH = 32,
    // Width of Data Bus
    parameter integer C_M_AXI_DATA_WIDTH = 64
) (
    input  wire [31:0] i_writeAddress,
    input  wire [63:0] i_writePayload,
    input  wire [31:0] i_readAddress,
    output wire [63:0] o_readPayload,

    // Initiate AXI transactions
    input wire i_initWriteTxn,
    input wire i_initReadTxn,
    // Asserts when transaction is complete
    output reg o_writeTxnDone,
    output reg o_readTxnDone,
    // Asserts when ERROR is detected
    output o_error,
    // Global Clock Signal.
    input wire M_AXI_ACLK,
    // Global Reset Singal. This Signal is Active Low
    input wire M_AXI_ARESETN,
    // Master Interface Write Address ID
    output wire [C_M_AXI_ID_WIDTH-1 : 0] M_AXI_AWID,
    // Master Interface Write Address
    output wire [C_M_AXI_ADDR_WIDTH-1 : 0] M_AXI_AWADDR,
    // Burst length. The burst length gives the exact number of transfers in a burst
    output wire [7 : 0] M_AXI_AWLEN,
    // Burst size. This signal indicates the size of each transfer in the burst
    output wire [2 : 0] M_AXI_AWSIZE,
    // Burst type. The burst type and the size information, 
    // determine how the address for each transfer within the burst is calculated.
    output wire [1 : 0] M_AXI_AWBURST,
    // Lock type. Provides additional information about the
    // atomic characteristics of the transfer.
    output wire M_AXI_AWLOCK,
    // Memory type. This signal indicates how transactions
    // are required to progress through a system.
    output wire [3 : 0] M_AXI_AWCACHE,
    // Protection type. This signal indicates the privilege
    // and security level of the transaction, and whether
    // the transaction is a data access or an instruction access.
    output wire [2 : 0] M_AXI_AWPROT,
    // Quality of Service, QoS identifier sent for each write transaction.
    output wire [3 : 0] M_AXI_AWQOS,
    // Write address valid. This signal indicates that
    // the channel is signaling valid write address and control information.
    output wire M_AXI_AWVALID,
    // Write address ready. This signal indicates that
    // the slave is ready to accept an address and associated control signals
    input wire M_AXI_AWREADY,
    // Master Interface Write Data.
    output wire [C_M_AXI_DATA_WIDTH-1 : 0] M_AXI_WDATA,
    // Write strobes. This signal indicates which byte
    // lanes hold valid data. There is one write strobe
    // bit for each eight bits of the write data bus.
    output wire [C_M_AXI_DATA_WIDTH/8-1 : 0] M_AXI_WSTRB,
    // Write last. This signal indicates the last transfer in a write burst.
    output wire M_AXI_WLAST,
    // Write valid. This signal indicates that valid write
    // data and strobes are available
    output wire M_AXI_WVALID,
    // Write ready. This signal indicates that the slave
    // can accept the write data.
    input wire M_AXI_WREADY,
    // Master Interface Write Response.
    input wire [C_M_AXI_ID_WIDTH-1 : 0] M_AXI_BID,
    // Write response. This signal indicates the status of the write transaction.
    input wire [1 : 0] M_AXI_BRESP,
    // Write response valid. This signal indicates that the
    // channel is signaling a valid write response.
    input wire M_AXI_BVALID,
    // Response ready. This signal indicates that the master
    // can accept a write response.
    output wire M_AXI_BREADY,
    // Master Interface Read Address.
    output wire [C_M_AXI_ID_WIDTH-1 : 0] M_AXI_ARID,
    // Read address. This signal indicates the initial
    // address of a read burst transaction.
    output wire [C_M_AXI_ADDR_WIDTH-1 : 0] M_AXI_ARADDR,
    // Burst length. The burst length gives the exact number of transfers in a burst
    output wire [7 : 0] M_AXI_ARLEN,
    // Burst size. This signal indicates the size of each transfer in the burst
    output wire [2 : 0] M_AXI_ARSIZE,
    // Burst type. The burst type and the size information, 
    // determine how the address for each transfer within the burst is calculated.
    output wire [1 : 0] M_AXI_ARBURST,
    // Lock type. Provides additional information about the
    // atomic characteristics of the transfer.
    output wire M_AXI_ARLOCK,
    // Memory type. This signal indicates how transactions
    // are required to progress through a system.
    output wire [3 : 0] M_AXI_ARCACHE,
    // Protection type. This signal indicates the privilege
    // and security level of the transaction, and whether
    // the transaction is a data access or an instruction access.
    output wire [2 : 0] M_AXI_ARPROT,
    // Quality of Service, QoS identifier sent for each read transaction
    output wire [3 : 0] M_AXI_ARQOS,
    // Write address valid. This signal indicates that
    // the channel is signaling valid read address and control information
    output wire M_AXI_ARVALID,
    // Read address ready. This signal indicates that
    // the slave is ready to accept an address and associated control signals
    input wire M_AXI_ARREADY,
    // Read ID tag. This signal is the identification tag
    // for the read data group of signals generated by the slave.
    input wire [C_M_AXI_ID_WIDTH-1 : 0] M_AXI_RID,
    // Master Read Data
    input wire [C_M_AXI_DATA_WIDTH-1 : 0] M_AXI_RDATA,
    // Read response. This signal indicates the status of the read transfer
    input wire [1 : 0] M_AXI_RRESP,
    // Read last. This signal indicates the last transfer in a read burst
    input wire M_AXI_RLAST,
    // Read valid. This signal indicates that the channel
    // is signaling the required read data.
    input wire M_AXI_RVALID,
    // Read ready. This signal indicates that the master can
    // accept the read data and response information.
    output wire M_AXI_RREADY
);

  reg                            axi_awvalid;
  reg                            axi_wvalid;
  reg                            axi_bready;
  reg                            axi_arvalid;
  reg                            axi_rready;
  reg [C_M_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
  reg [C_M_AXI_DATA_WIDTH-1 : 0] axi_wdata;
  reg [C_M_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
  reg [C_M_AXI_DATA_WIDTH-1 : 0] read_data;
  reg                            last_initWriteTxn;
  reg                            error_reg;

  assign M_AXI_AWID = 'b0;
  assign M_AXI_AWADDR = axi_awaddr;
  assign M_AXI_AWLEN = 8'b00000000;  // Single word
  assign M_AXI_AWSIZE = 3'b011;  // log2(8 bytes) = 3
  assign M_AXI_AWBURST = 2'b01;  // INCR burst type
  assign M_AXI_AWLOCK = 1'b0;
  assign M_AXI_AWCACHE = 4'b0000;
  assign M_AXI_AWPROT = 3'b000;
  assign M_AXI_AWQOS = 4'b0000;
  assign M_AXI_AWVALID = axi_awvalid;

  assign M_AXI_WDATA = axi_wdata;
  assign M_AXI_WSTRB = {(C_M_AXI_DATA_WIDTH / 8) {1'b1}};
  assign M_AXI_WLAST = 1'b1;  // Single word burst
  assign M_AXI_WVALID = axi_wvalid;

  assign M_AXI_BREADY = axi_bready;

  assign M_AXI_ARID = 'b0;
  assign M_AXI_ARADDR = axi_araddr;
  assign M_AXI_ARLEN = 8'b00000000;  // Single word
  assign M_AXI_ARSIZE = 3'b011;  // log2(8 bytes) = 3
  assign M_AXI_ARBURST = 2'b01;  // INCR burst type
  assign M_AXI_ARLOCK = 1'b0;
  assign M_AXI_ARCACHE = 4'b0000;
  assign M_AXI_ARPROT = 3'b000;
  assign M_AXI_ARQOS = 4'b0000;
  assign M_AXI_ARVALID = axi_arvalid;

  assign M_AXI_RREADY = axi_rready;

  assign o_readPayload = read_data;
  assign o_error = error_reg;

  always @(posedge M_AXI_ACLK) begin
    if (M_AXI_ARESETN == 0) begin
      axi_awvalid <= 1'b0;
      axi_wvalid <= 1'b0;
      axi_bready <= 1'b0;
      axi_arvalid <= 1'b0;
      axi_rready <= 1'b0;
      o_writeTxnDone <= 1'b0;
      o_readTxnDone <= 1'b0;
      last_initWriteTxn <= 1'b0;
      error_reg <= 1'b0;
    end else begin
      // Write transaction
      last_initWriteTxn <= i_initWriteTxn;
      if (i_initWriteTxn && !o_writeTxnDone) begin
        if (!axi_awvalid && !axi_wvalid && !last_initWriteTxn) begin
          axi_awaddr  <= i_writeAddress;
          axi_wdata   <= i_writePayload;
          axi_awvalid <= 1'b1;
          axi_wvalid  <= 1'b1;
        end

        if (M_AXI_AWREADY && axi_awvalid) axi_awvalid <= 1'b0;
        if (M_AXI_WREADY && axi_wvalid) axi_wvalid <= 1'b0;

        if (M_AXI_BVALID && !axi_bready) axi_bready <= 1'b1;
        if (axi_bready && M_AXI_BVALID) begin
          axi_bready <= 1'b0;
          o_writeTxnDone <= 1'b1;
        end
      end else begin
        o_writeTxnDone <= 1'b0;
      end

      // Read transaction
      if (i_initReadTxn && !o_readTxnDone) begin
        axi_araddr  <= i_readAddress;
        axi_arvalid <= 1'b1;
        if (M_AXI_ARREADY && axi_arvalid) axi_arvalid <= 1'b0;
        if (M_AXI_RVALID && !axi_rready) begin
          axi_rready <= 1'b1;
          read_data  <= M_AXI_RDATA;
        end
        if (axi_rready && M_AXI_RVALID) begin
          axi_rready <= 1'b0;
          o_readTxnDone <= 1'b1;
        end
      end else begin
        o_readTxnDone <= 1'b0;
      end

      // Error detection
      if (M_AXI_BVALID && M_AXI_BRESP[1]) error_reg <= 1'b1;
      if (M_AXI_RVALID && M_AXI_RRESP[1]) error_reg <= 1'b1;
    end
  end

endmodule
