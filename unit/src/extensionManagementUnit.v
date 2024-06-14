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
 
`timescale 1ns / 1ps

/* Module: MemMU
	Memory Management Unit

    The MMU module is the main . Here the interfaces, point clouds setups and representations are configured
    and connected to the remaining of ALFA Units. This module is composed of four main sections:

    - Representation instantiation
    - Point Cloud instantiation
    - Frame management
    - State machine for each interface:

        -  - ;
        - Read point cloud for algorithm processing.  
*/
module ExMU #( 
    /* Parameters integer: Point Clouds, Representations, Frames and States.
    */ 
    parameter   [7:0]       REPRESENTATION_TYPE = 1'b0
    )(

    /* Input: Input ports

    */
    input       [0:0]       i_SYSTEM_clk,
    input       [0:0]       i_SYSTEM_rst,
    input       [2047:0]    i_INT_readPayload,
    input       [0:0]       i_CU_ExMU_readCache,
    input       [0:0]       i_CU_ExMU_writeCache,
    input       [0:0]       i_CU_ExMU_readWriteID,
    input       [0:0]       i_CU_ExMU_readPoint,
    input       [0:0]       i_CU_ExMU_writePoint,
    input       [0:0]       i_CU_ExMU_writeMem,
    output      [0:0]       o_ExMU_readInCache,
    output      [0:0]       o_ExMU_writeInCache,
    output reg  [2047:0]    o_ExMU_writePayload,
    output reg  [18:0]      o_ExMU_writeID,
    output reg  [18:0]      o_ExMU_readID, 
    output reg  [31:0]      o_status,    

    //Extension interface
    input       [15:0]      i_EXT_writeCustomField,
    input       [18:0]      i_EXT_writeID,
    input       [18:0]      i_EXT_readID,         
    output reg  [15:0]      o_EXT_pointAngleH,
    output reg  [15:0]      o_EXT_pointAngleV,
    output reg  [15:0]      o_EXT_pointRadius,
    output reg  [15:0]      o_EXT_readCustomField,
    output reg  [15:0]      o_EXT_pointX,
    output reg  [15:0]      o_EXT_pointY, 
    output reg  [15:0]      o_EXT_pointZ
    );

    reg [63:0] write_cache [31:0];
    reg [63:0] read_cache [31:0];

    reg [13:0] addr_read_cache;
    reg [13:0] addr_write_cache;

    wire [13:0] id_read;
    wire [13:0] id_write;

    wire [63:0] ExMU_writePayload;
    wire [4:0] EXT_writeID_temp , EXT_readID_temp;
    wire [63:0] write_cache_ptr, read_cache_ptr;
    
    assign id_read = i_EXT_readID[18:5];
    assign id_write = i_EXT_writeID[18:5];

    assign EXT_writeID_temp = i_EXT_writeID[4:0];
    assign EXT_readID_temp = i_EXT_readID[4:0];

    assign write_cache_ptr = write_cache[EXT_writeID_temp];
    assign read_cache_ptr = read_cache[EXT_readID_temp];

    assign ExMU_writePayload[47:0] = write_cache_ptr[47:0];
    assign ExMU_writePayload[63:48] = i_EXT_writeCustomField;

    assign o_ExMU_readInCache = addr_read_cache==id_read? 1'b1 : 1'b0;
    assign o_ExMU_writeInCache = addr_write_cache==id_write? 1'b1 : 1'b0;

    integer i;

    //HANDLE CACHES
    always @(posedge i_SYSTEM_clk) begin
            if (i_SYSTEM_rst) begin
                if(i_CU_ExMU_readWriteID)
                    o_ExMU_readID <= (id_write << 5);
                else
                    o_ExMU_readID <= (id_read << 5);
            end
            else o_ExMU_readID <= 0;
    end

    //UPDATE CACHE READ
    always @(posedge i_SYSTEM_clk) begin
        if (i_SYSTEM_rst) begin
            if(i_CU_ExMU_readCache) begin
                for (i = 0; i < 32; i = i + 1) begin  
                    read_cache[i] <= i_INT_readPayload[i*64 +:64];   
                end
                addr_read_cache <= o_ExMU_readID >> 5;
            end   
        end
        else begin
            for (i = 0; i < 32; i = i + 1) begin  
                    read_cache[i] <= 0;    
                end
            addr_read_cache <= 1;     
        end
    end

    //UPDATE CACHE WRITE
    always @(posedge i_SYSTEM_clk) begin
        if (i_SYSTEM_rst) begin
            if(i_CU_ExMU_writeCache) begin
                for (i = 0; i < 32; i = i + 1) begin  
                    write_cache[i] <= i_INT_readPayload[i*64 +:64];   
                end
                addr_write_cache <= o_ExMU_readID >> 5;
                o_ExMU_writeID <= o_ExMU_readID;
            end
            else if(i_CU_ExMU_writePoint) 
                write_cache[EXT_writeID_temp][63:48] <= i_EXT_writeCustomField;   
        end
        else begin
            for (i = 0; i < 32; i = i + 1) begin  
                    write_cache[i] <= 0;    
                end
            addr_write_cache <= 1;
            o_ExMU_writeID <= 0;
        end
    end
 
    //WRITE
    always @(posedge i_SYSTEM_clk) begin
        if (i_SYSTEM_rst) begin
            if(i_CU_ExMU_writeMem)
                for (i = 0; i < 32; i = i + 1) begin  
                    o_ExMU_writePayload[i*64 +:64] <= write_cache[i];   
                end
        end
        else begin
            for (i = 0; i < 32; i = i + 1) begin  
                    o_ExMU_writePayload[i*64 +:64] <= 0;    
            end
        end
    end

    generate

        //READ
        if(REPRESENTATION_TYPE==1'b0) begin
            always @(posedge i_SYSTEM_clk) begin
                if (i_SYSTEM_rst) begin
                     o_status <= 1'b1;
                    if(i_CU_ExMU_readPoint) begin
                        o_EXT_pointAngleH <= read_cache_ptr[15:0];
                        o_EXT_pointAngleV <= read_cache_ptr[31:16];
                        o_EXT_pointRadius <= read_cache_ptr[47:32];
                        o_EXT_readCustomField  <= read_cache_ptr[63:48];                      
                        end
                end   
                else begin
                    o_EXT_pointZ <= 0;
                    o_EXT_pointY <= 0;	
                    o_EXT_pointX <= 0;
                    o_EXT_pointAngleH <= 0;
                    o_EXT_pointAngleV <= 0;
                    o_EXT_pointRadius <= 0;
                    o_status <= 1'b0;
                    o_EXT_readCustomField <= 0;
                end
            end
        end
        else if(REPRESENTATION_TYPE==1'b1) begin
            always @(posedge i_SYSTEM_clk) begin
                if (i_SYSTEM_rst) begin
                     o_status <= 1'b1;
                    if(i_CU_ExMU_readPoint) begin
                        o_EXT_pointX <= read_cache_ptr[15:0];
                        o_EXT_pointY <= read_cache_ptr[31:16];	
                        o_EXT_pointZ <= read_cache_ptr[47:32];
                        o_EXT_readCustomField  <= read_cache_ptr[63:48];
                    end
                end   
                else begin
                    o_EXT_pointZ <= 0;
                    o_EXT_pointY <= 0;	
                    o_EXT_pointX <= 0;
                    o_EXT_pointAngleH <= 0;
                    o_EXT_pointAngleV <= 0;
                    o_EXT_pointRadius <= 0;
                    o_status <= 1'b0;
                    o_EXT_readCustomField <= 0;
                end
            end
        
        end
    endgenerate
endmodule
