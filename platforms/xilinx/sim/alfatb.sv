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

// Import the AXI VIP module
import axi_vip_pkg::*;
import design_1_axi_vip_0_0_pkg::*;
import design_1_axi_vip_1_0_pkg::*;

`define MONU_REGS 32'h8000_0000 // 1G
`define DDR_EMULATION 32'h0000_0000 // 64k, axi_vip_0 
`define BRAM_EMULATION 32'h0000_0000 // 64k, axi_vip_0 
`define POINTCLOUD_MAX_SIZE_DDR 32'h0010_0000 // 1G
`define POINTCLOUD_MAX_SIZE_BRAM 32'h0002_0000 // 128k
`define POINTCLOUD_NUMBER_POINTS 1000
`define POINTCLOUD_SIZE_DDR `POINTCLOUD_NUMBER_POINTS*8 
`define POINTCLOUD_SIZE_BRAM `POINTCLOUD_NUMBER_POINTS

module ALFA_tb ();
  xil_axi_resp_t resp;
  xil_axi_uint mst_agent_verbosity = XIL_AXI_VERBOSITY_NONE;
  xil_axi_uint slv_agent_verbosity = XIL_AXI_VERBOSITY_NONE;

  bit [design_1_axi_vip_0_0_VIP_DATA_WIDTH - 1:0] mem_data;
  bit [design_1_axi_vip_1_0_VIP_DATA_WIDTH - 1:0] reg_data;

  design_1_axi_vip_1_0_mst_t mst_agent;
  design_1_axi_vip_0_0_slv_mem_t slv_agent;

  bit tb_reset;
  bit tb_clk;

  design_1_wrapper DUT (
                                                                                                      .i_SYSTEM_clk_0(tb_clk),
                                                                                                      .i_SYSTEM_rst_0(tb_reset)
  );

  initial begin
    tb_reset <= 1'b0;
    repeat (20) @(negedge tb_clk);
    tb_reset <= 1'b1;
  end

  always #5 tb_clk <= ~tb_clk;

  // Main process
  initial begin
    $display("Running ALFA_tb");

    // new the agents
    mst_agent = new("MasterVIP", DUT.design_1_i.axi_vip_1.inst.IF);
    slv_agent = new("SlaveVIP", DUT.design_1_i.axi_vip_0.inst.IF);
    $timeformat(-12, 1, "ps", 1);
    // Set tags for easier debug
    mst_agent.set_agent_tag("Master VIP");
    slv_agent.set_agent_tag("Slave VIP");
    // Set verbosity
    mst_agent.set_verbosity(mst_agent_verbosity);
    slv_agent.set_verbosity(slv_agent_verbosity);

    // start the agents
    mst_agent.start_master();
    slv_agent.start_slave();

    // initialize first 1G of slave VIP memory with data from file
    backdoor_mem_write(`DDR_EMULATION, `DDR_EMULATION + `POINTCLOUD_SIZE_DDR);

    // Set up MonU that Point Cloud is X size
    $display("Set PC_size_ready with `POINTCLOUD_SIZE_DDR");
    mst_agent.AXI4LITE_WRITE_BURST(`MONU_REGS + 37 * 4, 0, `POINTCLOUD_NUMBER_POINTS, resp);

    // Set up MonU that Filter is between 5 and 10 meters
    $display("Set PC_size_ready with `POINTCLOUD_SIZE_DDR");
    mst_agent.AXI4LITE_WRITE_BURST(`MONU_REGS + 40 * 4, 0, 500, resp);
    $display("Set PC_size_ready with `POINTCLOUD_SIZE_DDR");
    mst_agent.AXI4LITE_WRITE_BURST(`MONU_REGS + 41 * 4, 0, 1000, resp);

    // Set up MonU that Point Cloud is ready.
    $display("Active trigger PC_ready");
    mst_agent.AXI4LITE_WRITE_BURST(`MONU_REGS + 35 * 4, 0, 0, resp);
    mst_agent.AXI4LITE_WRITE_BURST(`MONU_REGS + 35 * 4, 0, 1, resp);

    // Set up slave wready with desired policy
    $display("Set slave wready");
    gen_wready();

    begin
      do begin
        mst_agent.AXI4LITE_READ_BURST(`MONU_REGS + 9 * 4, 0, reg_data, resp);
      end while ((reg_data) != 1);
    end

    backdoor_mem_read(`DDR_EMULATION, `DDR_EMULATION + `POINTCLOUD_SIZE_DDR);
    $display("Successful Test");
    $finish;
  end

  longint
                                                                                                      data,
                                                                                                      i,
                                                                                                      x,
                                                                                                      y,
                                                                                                      z,
                                                                                                      label;
  longint inside_i = 0;
  int     fd1;

  task backdoor_mem_write(input xil_axi_ulong start_addr, input xil_axi_ulong stop_addr);
    bit           [    design_1_axi_vip_0_0_VIP_ADDR_WIDTH-1:0] mem_wr_addr;
    bit           [    design_1_axi_vip_0_0_VIP_DATA_WIDTH-1:0] write_data;
    bit           [(design_1_axi_vip_0_0_VIP_DATA_WIDTH/8)-1:0] write_strb;
    xil_axi_ulong                                               addr_offset;

    slv_agent.mem_model.set_memory_fill_policy(
                                                                                                        XIL_AXI_MEMORY_FILL_FIXED);        // Determines what policy to use when memory model encounters an empty entry
    slv_agent.mem_model.set_default_memory_value(
                                                                                                        32'hFFFFFFFF);        // Determines what policy to use when memory model encounters an empty entry

    write_strb = ($pow(
                                                                                                        2,
                                                                                                        (design_1_axi_vip_0_0_VIP_DATA_WIDTH/8)
    ) - 1);  // All strobe bits asserted
    fd1 = $fopen(
                                                                                                        "/home/ricardororiz/ALFA_Workspace/dev/alfa-unit/sim/mem_generator/mem_cart.txt",
                                                                                                        "r"
    );

    for (
                                                                                                        mem_wr_addr = start_addr
                                                                                                        ,
                                                                                                        i = 0 ;
                                                                                                        mem_wr_addr < stop_addr;
                                                                                                        mem_wr_addr += 8
                                                                                                        ,
                                                                                                        i++
    ) begin
      //WRITE_DATA_FAIL: assert(std::randomize(write_data)); 
      //write_data = data read from the file with the input data
      $fscanf(fd1, "%d\n", data);
      write_data = data;
      x = (data & 64'h0000_0000_0000_FFFF);
      y = ((data & 64'h0000_0000_FFFF_0000) >> 16);
      z = ((data & 64'h0000_FFFF_0000_0000) >> 32);
      label = (data >> 56) & 64'hFF;
      $display("Write: 0x%H", data);
      $display("-> X %d", x);
      $display("-> Y %d", y);
      $display("-> Z %d", z);
      $display("-> label %d", label);

      slv_agent.mem_model.backdoor_memory_write(mem_wr_addr, write_data, write_strb);
    end
    $display("-> Points %d", i);
    $display("Successful backdoor_mem_write");

    $fclose(fd1);
  endtask : backdoor_mem_write

  task backdoor_mem_read(input xil_axi_ulong start_addr, input xil_axi_ulong stop_addr);
    bit           [design_1_axi_vip_0_0_VIP_ADDR_WIDTH-1:0] mem_rd_addr;
    bit           [design_1_axi_vip_0_0_VIP_DATA_WIDTH-1:0] read_data;
    xil_axi_ulong                                           addr_offset;

    for (
                                                                                                        mem_rd_addr = start_addr
                                                                                                        ,
                                                                                                        i = 0 ;
                                                                                                        mem_rd_addr < stop_addr;
                                                                                                        mem_rd_addr += 8
                                                                                                        ,
                                                                                                        i++
    ) begin
      read_data = slv_agent.mem_model.backdoor_memory_read(mem_rd_addr);
      x = (read_data & 64'h0000_0000_0000_FFFF);
      y = ((read_data & 64'h0000_0000_FFFF_0000) >> 16);
      z = ((read_data & 64'h0000_FFFF_0000_0000) >> 32);
      label = (read_data >> 56) & 64'hFF;

      if (label == 1) begin
        $display("Write: 0x%H", read_data);
        $display("-> X %d", x);
        $display("-> Y %d", y);
        $display("-> Z %d", z);
        $display("-> label %d", label);
        $display("-> id %d", i);
        inside_i = inside_i + 1;

      end
    end
    $display("-> Points Inside %d", inside_i);
    $display("Successful backdoor_mem_read");
  endtask : backdoor_mem_read



  // task to set slave AXI VIP wready policy to always asserted
  task gen_wready();
    axi_ready_gen wready_gen;
    wready_gen = slv_agent.wr_driver.create_ready("wready");
    wready_gen.set_ready_policy(XIL_AXI_READY_GEN_NO_BACKPRESSURE);
    slv_agent.wr_driver.send_wready(wready_gen);
  endtask : gen_wready

endmodule
