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

module alib_bram #(
    parameter DATA_WIDTH = 8,  // Width of the data
    parameter DEPTH = 1024     // Depth of the memory
) (
    input wire clk,
    input wire rst,  // RAM reset
    input wire [$clog2(DEPTH-1)-1:0] addr,  // Address width determined by DEPTH
    input wire [DATA_WIDTH-1:0] din,
    input wire we,
    output wire [DATA_WIDTH-1:0] dout
);

  // Memory declaration
  (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram_mem[0:DEPTH-1];

  reg [DATA_WIDTH-1:0] ram_data = {DATA_WIDTH{1'b0}};
  integer ram_index;

  // Read and write operations
  always @(posedge clk) begin
    if (rst) begin
      if (we) ram_mem[addr] <= din;  // Write operation
      ram_data <= ram_mem[addr];  // Read operation
    end else ram_data <= {DATA_WIDTH{1'b0}};
  end

  assign dout = ram_data;
endmodule

module alib_bram_r_w #(
    parameter DATA_WIDTH = 8,  // Width of the data
    parameter DEPTH = 1024     // Depth of the memory
) (
    // Port A (Write Only)
    input wire                       clk,    // Write clock
    input wire [$clog2(DEPTH-1)-1:0] addra,  // Write address bus
    input wire [     DATA_WIDTH-1:0] din,    // RAM input data
    input wire                       we,     // Write enable

    // Port B (Read Only)
    input  wire [$clog2(DEPTH-1)-1:0] addrb,  // Read address bus
    input  wire                       rst,    // Output reset
    output wire [     DATA_WIDTH-1:0] dout    // RAM output data
);

  // Memory declaration
  (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram_mem[0:DEPTH-1];

  reg [DATA_WIDTH-1:0] ram_data = {DATA_WIDTH{1'b0}};

  // Port A (Write Only)
  always @(posedge clk) begin
    if (rst) begin
      if (we) ram_mem[addra] <= din;  // Write operation
    end
  end

  // Port B (Read Only)
  always @(posedge clk) begin
    if (rst) ram_data <= ram_mem[addrb];  // Read operation
    else ram_data <= {DATA_WIDTH{1'b0}};
  end

  assign dout = ram_data;

endmodule

module alib_uram_r_w #(
    parameter DATA_WIDTH = 8,  // Width of the data
    parameter DEPTH = 1024     // Depth of the memory
) (
    // Port A (Write Only)
    input wire                       clk,    // Write clock
    input wire [$clog2(DEPTH-1)-1:0] addra,  // Write address bus
    input wire [     DATA_WIDTH-1:0] din,    // RAM input data
    input wire                       we,     // Write enable

    // Port B (Read Only)
    input  wire [$clog2(DEPTH-1)-1:0] addrb,  // Read address bus
    input  wire                       rst,    // Output reset
    output wire [     DATA_WIDTH-1:0] dout    // RAM output data
);

  // Memory declaration
  (* ram_style = "ultra" *) reg [DATA_WIDTH-1:0] ram_mem[0:DEPTH-1];

  reg [DATA_WIDTH-1:0] ram_data = {DATA_WIDTH{1'b0}};

    // Port A (Write Only)
  always @(posedge clk) begin
    if (rst) begin
      if (we) ram_mem[addra] <= din;  // Write operation
    end
  end

  // Port B (Read Only)
  always @(posedge clk) begin
    if (rst) ram_data <= ram_mem[addrb];  // Read operation
    else ram_data <= {DATA_WIDTH{1'b0}};
  end

  assign dout = ram_data;

endmodule

module alib_uram #(
    parameter DATA_WIDTH = 8,  // Width of the data
    parameter DEPTH = 1024     // Depth of the memory
) (
    input wire clk,
    input wire rst,  // RAM reset
    input wire [$clog2(DEPTH-1)-1:0] addr,  // Address width determined by DEPTH
    input wire [DATA_WIDTH-1:0] din,
    input wire we,
    output wire [DATA_WIDTH-1:0] dout
);

  // Memory declaration
  (* ram_style = "ultra" *) reg [DATA_WIDTH-1:0] ram_mem[0:DEPTH-1];

  reg [DATA_WIDTH-1:0] ram_data = {DATA_WIDTH{1'b0}};

  // Read and write operations
  always @(posedge clk) begin
    if (rst) begin
      if (we) ram_mem[addr] <= din;  // Write operation
      ram_data <= ram_mem[addr];  // Read operation
    end else ram_data <= {DATA_WIDTH{1'b0}};
  end

  assign dout = ram_data;
endmodule

module alib_dram #(
    parameter DATA_WIDTH = 8,  // Width of the data
    parameter DEPTH = 1024     // Depth of the memory
) (
    input wire clk,
    input wire rst,  // RAM reset
    input wire [$clog2(DEPTH-1)-1:0] addr,  // Address width determined by DEPTH
    input wire [DATA_WIDTH-1:0] din,
    input wire we,
    output wire [DATA_WIDTH-1:0] dout
);

  // Memory declaration
  (* ram_style = "distributed" *) reg [DATA_WIDTH-1:0] ram_mem[0:DEPTH-1];

  reg [DATA_WIDTH-1:0] ram_data = {DATA_WIDTH{1'b0}};

  // Read and write operations
  always @(posedge clk) begin
    if (rst) begin
      if (we) ram_mem[addr] <= din;  // Write operation
      ram_data <= ram_mem[addr];  // Read operation
    end else ram_data <= {DATA_WIDTH{1'b0}};
  end

  assign dout = ram_data;
endmodule