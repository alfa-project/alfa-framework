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

/* Module: CU
    Control Unit::

    *TODO*. 
*/
module CU #(

    /* Parameters integer: TODO.

        TODO = TODO;
    */
    parameter [0:0]     SIU_SUPPORT = 1'h0,
    parameter [0:0]     EXMU_SUPPORT = 1'h0
    )(

    /* Input: Input ports

        <System::clk> - System clock. 
        <System::rst> - System reset.
    */
    input wire [0:0]    i_SYSTEM_clk,
    input wire [0:0]    i_SYSTEM_rst,	
    input wire [0:0]    i_SIU_newPoint,
    input wire [0:0]    i_SIU_newFrame,
    input wire [0:0]    i_ExMU_readInCache,
    input wire [0:0]    i_ExMU_writeInCache,
    input wire [0:0]    i_EXT_doneProcessing,
    input wire [0:0]    i_EXT_writeValid,
    input wire [0:0]    i_EXT_readReady,
    input wire [31:0]   i_MemMU_status,
    input wire [31:0]   i_ExMU_status,
    input wire [31:0]   i_MonU_status,
    input wire [31:0]   i_SIU_status, 
    input wire [0:0]    i_INT_writeTxnDone,
    input wire [0:0]    i_INT_readTxnDone,
    input wire [0:0]    i_CU_MonU_pcReady,
    input wire [0:0]    i_CU_MonU_frameDone,          
    output wire [0:0]   o_CU_MonU_extReady,
    output reg [0:0]    o_CU_EXT_enable,
    output     [0:0]    o_CU_EXT_writeReady,
    output     [0:0]    o_CU_EXT_readValid,
    output reg [0:0]    o_CU_INT_initWriteTx,
    output reg [0:0]    o_CU_INT_initReadTx,
    output reg [0:0]    o_CU_ExMU_readCache,
    output reg [0:0]    o_CU_ExMU_writeCache,
    output     [0:0]    o_CU_ExMU_readWriteID,
    output     [0:0]    o_CU_ExMU_readPoint,
    output     [0:0]    o_CU_ExMU_writePoint,
    output reg [0:0]    o_CU_ExMU_writeMem,
    output reg [0:0]    o_CU_MonU_pcReady,
    output reg [0:0]    o_CU_MonU_processingDone,
    output reg [0:0]    o_CU_SIU_ExMU,
    output reg [31:0]   o_status
    ); 

	localparam [4:0] 
        IDLE                    = 5'b00001, 
        SIU_WRITE_PC            = 5'b00010, 
        SIU_WRITE_END           = 5'b00011, 
        EXT_READ_POINT          = 5'b00100,
        EXT_READ_CACHE          = 5'b00101,
        EXT_READ_CACHE_T_OFF    = 5'b00110,
        EXT_READ_CACHE_END      = 5'b00111, 
        EXT_WRITE_POINT         = 5'b01000, 
        EXT_WRITE_CACHE         = 5'b01001,
        EXT_WRITE_CACHE_T_OFF   = 5'b01010,
        EXT_WRITE_CACHE_END     = 5'b01011,
        EXT_WRITE_MEM           = 5'b01100,
        EXT_WRITE_MEM_T_OFF      = 5'b01101,
        EXT_WRITE_MEM_END       = 5'b01110,
        EXT_LAST_WRITE          = 5'b01111,
        EXT_LAST_WRITE_T_OFF    = 5'b10000,
        EXT_END                 = 5'b10001, 
        ERROR                   = 5'b11111, 
        RESET                   = 5'b00000;

    wire [3:0] error;
    wire finishing_process;
    reg first_time, readReady, CU_EXT_writeReady, CU_EXT_readValid;

    assign error[0]   = (i_MemMU_status == 32'hFFFFFFFF)? 1'b1 : 1'b0;
    assign error[1]   = (i_ExMU_status  == 32'hFFFFFFFF)? 1'b1 : 1'b0;
    assign error[2]   = (i_MonU_status  == 32'hFFFFFFFF)? 1'b1 : 1'b0;
    assign error[3]   = (i_SIU_status   == 32'hFFFFFFFF)? 1'b1 : 1'b0;
    
    assign finishing_process = (o_status==EXT_LAST_WRITE || o_status==EXT_LAST_WRITE_T_OFF || o_status==EXT_END)? 1'b1 : 1'b0;
    assign o_CU_ExMU_writePoint = (~finishing_process && i_ExMU_writeInCache && CU_EXT_writeReady && i_EXT_writeValid)? 1'b1 : 1'b0;
    assign o_CU_ExMU_readPoint = (~finishing_process && i_ExMU_readInCache && i_EXT_readReady && ~readReady) ? 1'b1 : 1'b0;
    assign o_CU_EXT_writeReady = i_ExMU_writeInCache? CU_EXT_writeReady : 1'b0;
    assign o_CU_EXT_readValid = i_ExMU_readInCache? CU_EXT_readValid : 1'b0;
    
    assign o_CU_MonU_extReady = ~o_CU_EXT_enable;
    assign o_CU_ExMU_readWriteID = ~i_ExMU_writeInCache? 1'b1 : 1'b0;
    reg [0:0] pcReady_rise_up;
    
    always @(posedge i_SYSTEM_clk) begin
        if(i_SYSTEM_rst) begin
            if(o_CU_EXT_enable && i_EXT_doneProcessing) o_CU_EXT_enable <= 1'b0;
            else if(pcReady_rise_up==0 && i_CU_MonU_pcReady==1) o_CU_EXT_enable <= 1'b1;
            else o_CU_EXT_enable <= o_CU_EXT_enable;
            pcReady_rise_up <= i_CU_MonU_pcReady;
        end
        else begin
            o_CU_EXT_enable <= 0;
            pcReady_rise_up <= 0;
        end
    end

    always @(posedge i_SYSTEM_clk) begin
        if(i_SYSTEM_rst) begin
            if(~finishing_process) begin
                readReady <= i_ExMU_readInCache & i_EXT_readReady;
                CU_EXT_readValid = o_CU_ExMU_readPoint;
                CU_EXT_writeReady <= i_ExMU_writeInCache & ~o_CU_ExMU_writePoint;  
            end
            else begin
                CU_EXT_writeReady <= 1'b0;
                CU_EXT_readValid <= 1'b0; 
                readReady <= 1'b0;
            end
        end
        else begin
            CU_EXT_writeReady <= 1'b0;
            CU_EXT_readValid  <= 1'b0;
            readReady <= 1'b0;
        end
    end

    always @(posedge i_SYSTEM_clk) 
    begin
        if(i_SYSTEM_rst) begin
            case (o_status)       

                RESET:
                    if (error!=0)
                        o_status  <= ERROR;
                    else 
                        o_status  <= IDLE;    

                ERROR:
                        o_status  <= ERROR;

				IDLE:
                    if (error!=0)
                        o_status  <= ERROR;
                    else if (i_EXT_doneProcessing == 1) 
						o_status  <= EXT_LAST_WRITE;
                    else if (SIU_SUPPORT == 1'b1 && i_SIU_newPoint)
                        o_status  <= SIU_WRITE_PC;                                                                                                                                                     
					else if (i_CU_MonU_pcReady && EXMU_SUPPORT==1'b1) begin
                        if(i_EXT_doneProcessing)
                            o_status  <= EXT_END;
                        else if(i_INT_readTxnDone && i_INT_writeTxnDone) begin
                            if(~i_ExMU_writeInCache) begin
                                if(~first_time)
                                    o_status  <= EXT_WRITE_MEM;
                                else
                                    o_status  <= EXT_WRITE_CACHE;
                            end
                            else if(~i_ExMU_readInCache) 
                                o_status  <= EXT_READ_CACHE;
                            else
                                o_status  <= IDLE;
                        end
                        else 
                            o_status  <= IDLE;
                    end
                    else
                        o_status  <= IDLE;
				                                                                                                                                                 																												
				SIU_WRITE_PC: 
					if (error!=0)
                        o_status  <= ERROR;
                    else if (i_SIU_newFrame == 1)
                        o_status  <= SIU_WRITE_END;                                                                                                                                                
					else 
                        o_status  <= SIU_WRITE_PC;  

                SIU_WRITE_END:
                    if (error!=0)
                        o_status  <= ERROR;
                    else
                        o_status  <= IDLE;  

                EXT_WRITE_MEM:
                    if (error!=0)
                        o_status  <= ERROR;
                    else 
                        o_status  <= EXT_WRITE_MEM_T_OFF; 

                EXT_WRITE_MEM_T_OFF:
                    if (error!=0)
                        o_status  <= ERROR;
                    else
                        o_status  <= EXT_WRITE_MEM_END;        

                EXT_WRITE_MEM_END:
                    if (error!=0)
                        o_status  <= ERROR;
                    else
                        o_status  <= EXT_WRITE_CACHE;  

                EXT_WRITE_CACHE:
                    if (error!=0)
                            o_status  <= ERROR;
                    else
                        o_status  <= EXT_WRITE_CACHE_T_OFF;

                EXT_WRITE_CACHE_T_OFF:
                    if (error!=0)
                            o_status  <= ERROR;
                    else if(i_INT_readTxnDone && ~o_CU_INT_initReadTx)
                        o_status  <= EXT_WRITE_CACHE_END; 
                    else 
                        o_status  <= EXT_WRITE_CACHE_T_OFF; 

                EXT_WRITE_CACHE_END:
                    if (error!=0)
                            o_status  <= ERROR;
                    else if(o_CU_ExMU_writeCache)
                        o_status  <= IDLE;
                    else 
                        o_status  <= EXT_WRITE_CACHE_END;

                EXT_READ_CACHE:
                    if (error!=0)
                            o_status  <= ERROR;
                    else
                        o_status  <= EXT_READ_CACHE_T_OFF;

                EXT_READ_CACHE_T_OFF:
                    if (error!=0)
                            o_status  <= ERROR;
                    else if(i_INT_readTxnDone && ~o_CU_INT_initReadTx)
                        o_status  <= EXT_READ_CACHE_END; 
                    else 
                        o_status  <= EXT_READ_CACHE_T_OFF; 

                EXT_READ_CACHE_END:
                    if (error!=0)
                            o_status  <= ERROR;
                    else if(o_CU_ExMU_readCache)
                        o_status  <= IDLE;
                    else 
                        o_status  <= EXT_READ_CACHE_END;   

                EXT_LAST_WRITE:
                    if (error!=0)
                            o_status  <= ERROR;
                    else
                        o_status  <= EXT_LAST_WRITE_T_OFF;

                EXT_LAST_WRITE_T_OFF:
                    if (error!=0)
                            o_status  <= ERROR;
                    else if(i_INT_writeTxnDone && ~o_CU_INT_initWriteTx)
                        o_status  <= EXT_END; 
                    else 
                        o_status  <= EXT_LAST_WRITE_T_OFF;

                EXT_END:
                    if (error!=0)
                        o_status  <= ERROR;
                    else if (o_CU_EXT_enable) 
                        o_status  <= IDLE; 
                    else 
                        o_status <= EXT_END;

				default :                                                                                         
					o_status  <= IDLE;                                                                                                                                                        
			endcase  
        end
        else 
            o_status <= RESET;   
    end  

    always @(posedge i_SYSTEM_clk) 
    begin
        if(i_SYSTEM_rst) begin
            case (o_status)       

                RESET:
                begin
                    o_CU_INT_initWriteTx        <= 1'b0;
                    o_CU_INT_initReadTx         <= 1'b0;
                    o_CU_ExMU_readCache         <= 1'b0;
                    o_CU_ExMU_writeCache        <= 1'b0;
                    o_CU_MonU_pcReady           <= 1'b0;
                    o_CU_MonU_processingDone    <= 1'b0;
                    o_CU_SIU_ExMU               <= 1'b0; 
                    o_CU_ExMU_writeMem          <= 1'b0;
                    first_time                  <= 1'b1;
                end    

				IDLE:
                begin
                    o_CU_INT_initWriteTx        <= 1'b0;
                    o_CU_INT_initReadTx         <= 1'b0;
                    o_CU_ExMU_readCache         <= 1'b0;
                    o_CU_ExMU_writeCache        <= 1'b0;
                    o_CU_MonU_processingDone    <= 1'b0;
                    o_CU_SIU_ExMU               <= 1'b1;
                end 

                ERROR:
                begin
                    o_CU_MonU_pcReady           <= 1'b0;
                    o_CU_MonU_processingDone    <= 1'b0;
                    o_CU_SIU_ExMU               <= 1'b0; 
                end                                                                                                                                                    
																												
				SIU_WRITE_PC: 
                begin
                    //SIU IMPLEMENTATION CONTROL
                    o_CU_SIU_ExMU               <= 1'b0; 
                end  

                SIU_WRITE_END:
                begin
                    //SIU IMPLEMENTATION CONTROL  
                    o_CU_SIU_ExMU               <= 1'b0;   
                end

                EXT_WRITE_MEM:
                begin
                    o_CU_ExMU_writeMem          <= 1'b1;
                end

                EXT_WRITE_MEM_T_OFF:
                begin
                    o_CU_ExMU_writeMem          <= 1'b0;
                end

                EXT_WRITE_MEM_END:
                begin
                    o_CU_INT_initWriteTx        <= 1'b1; 
                end

                EXT_WRITE_CACHE:
                begin
                    o_CU_INT_initWriteTx        <= 1'b0;
                    o_CU_INT_initReadTx         <= 1'b1;
                    o_CU_ExMU_writeCache        <= 1'b0;
                    first_time                  <= 1'b0;  
                end

                EXT_WRITE_CACHE_T_OFF:
                begin
                    o_CU_INT_initReadTx         <= 1'b0; 
                end

                EXT_WRITE_CACHE_END:
                begin
                    o_CU_ExMU_writeCache        <= 1'b1;
                end

                EXT_READ_CACHE:
                begin
                    o_CU_INT_initReadTx         <= 1'b1;
                    o_CU_ExMU_readCache         <= 1'b0; 
                end

                EXT_READ_CACHE_T_OFF:
                begin
                    o_CU_INT_initReadTx         <= 1'b0; 
                end

                EXT_READ_CACHE_END:
                begin
                    o_CU_ExMU_readCache         <= 1'b1; 
                end

                EXT_LAST_WRITE:
                begin
                    o_CU_INT_initWriteTx        <= 1'b1;
                end

                EXT_LAST_WRITE_T_OFF:
                begin
                    o_CU_INT_initWriteTx        <= 1'b0;
                end

                EXT_END:
                begin
                    o_CU_MonU_processingDone    <= 1'b1;    
                end 

				default :                                                                                         
                begin
                    o_CU_INT_initWriteTx        <= 1'b0;
                    o_CU_INT_initReadTx         <= 1'b0;
                    o_CU_ExMU_readCache         <= 1'b0;
                    o_CU_ExMU_writeCache        <= 1'b0;
                    o_CU_MonU_pcReady           <= 1'b0;
                    o_CU_MonU_processingDone    <= 1'b0;
                    o_CU_SIU_ExMU               <= 1'b0; 
                    o_CU_ExMU_writeMem          <= 1'b0;
                    first_time                  <= 1'b1;
                end                                                                                                                                                        
			endcase  
        end
        else 
            begin
                o_CU_INT_initWriteTx            <= 1'b0;
                o_CU_INT_initReadTx             <= 1'b0;
                o_CU_ExMU_readCache             <= 1'b0;
                o_CU_ExMU_writeCache            <= 1'b0;
                first_time                      <= 1'b1;
                o_CU_MonU_pcReady               <= 1'b0;
                o_CU_MonU_processingDone        <= 1'b0;  
                o_CU_SIU_ExMU                   <= 1'b0;  
                o_CU_ExMU_writeMem              <= 1'b0; 
            end
    end  
endmodule