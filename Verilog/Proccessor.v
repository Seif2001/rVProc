`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/13/2023 05:36:23 PM
// Design Name: 
// Module Name: Proccessor
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
/*

module ALUControl(
    input[1:0] op,
    input[31:0] inst,
    output reg[3:0] sel
);*/
`default_nettype none

//DATAMEM Address
//CHANGE INSTMEM
//data?
//load?
//Qsize?
// modify rca
//when to assign
//first value of pc
// input pClk, input [1:0] ledSel, input [4:0] ssdSel, output[15:0]leds, output[12:0] ssd 

`include "defines.v"
//,input pClk, input [1:0] ledSel, input [3:0] ssdsel, output reg [15:0]leds, output [3:0] anode,output [6:0]ledout
module Proccessor(input wire clk, input wire reset);
    wire [31:0] pc,data;
    wire [3:0] ALUcontrol;
    wire [31:0] instruction, WriteData, Read1, Read2, imm, ALU2muxOut, ALU1muxOut, ALUresult,shiftout, branchaddOut,pcadderOUT, writeToReg, AUIPC_add_out;
    wire jalr, branch, MemRead, MemWrite, ALUsrc1, ALUsrc2, Regwrite, zf, cf, vf, sf;
    wire [1:0] pcSel;
    wire [1:0] ALUOP, Memtoreg;
    wire [3:0] readwrite;
    reg [12:0]  sevensegin;
    

    Nbit_reg #( .N(32) ) PC(.clk(clk), .load(1), .reset(reset), .data(data), .Q(pc));
    RCA pcadder (.A(pc),.B(4),.Sum(pcadderOUT));
    InstMem Instructions(.addr({pc[7:2]}), .data_out(instruction ));
    mux4by2 muxpc(.i0(pcadderOUT),.i1(0), .i2(EX_MEM_BranchAddOut), .i3(EX_MEM_ALU_out), .s(pcSel),.out(data));

    
    
     ///////
   
       wire [31:0] IF_ID_PC, IF_ID_PCP4;
       wire[31:0] IF_ID_Inst;
       Register #(200) IF_ID (clk,reset,1'b1, {pc, instruction, pcadderOUT }, {IF_ID_PC,IF_ID_Inst, IF_ID_PCP4} );
    
    controlUnit control(.IR(IF_ID_Inst), .MemRead(MemRead), .Memtoreg(Memtoreg), .MemWrite(MemWrite), .ALUsrc1(ALUsrc1), .ALUsrc2(ALUsrc2),.j(jalr), .Regwrite(Regwrite), .ALUOP(ALUOP), .read_write(readwrite));
    
    NbitReg RegFile(.clk(clk), .rst(reset),.RS1(IF_ID_Inst[19:15]), .RS2(IF_ID_Inst[24:20]), .RD(MEM_WB_Rd),.WData(writeToReg),.regWrite(MEM_WB_Ctrl[6]),.R1(Read1), .R2(Read2));
    rv32_ImmGen ImmediateGenerator(.IR(IF_ID_Inst), .Imm(imm));

     wire [31:0] ID_EX_PC, ID_EX_PCP4;
       wire[31:0] ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm,ID_EX_Inst, forAout, forBout;
       wire [13:0] ID_EX_Ctrl;
       wire [4:0] ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;
       
       Register #(350) ID_EX (clk,reset,1'b1,
           {{MemRead, Memtoreg,MemWrite, ALUsrc1, ALUsrc2, jalr, Regwrite, ALUOP, readwrite  }, IF_ID_PC,IF_ID_PCP4, Read1, Read2, imm, 
               IF_ID_Inst[19:15], IF_ID_Inst[24:20],IF_ID_Inst[11:7], IF_ID_Inst },
           {ID_EX_Ctrl,ID_EX_PC,ID_EX_PCP4, ID_EX_RegR1,ID_EX_RegR2,
           ID_EX_Imm,ID_EX_Rs1,ID_EX_Rs2,ID_EX_Rd, ID_EX_Inst} );
       // Rs1 and Rs2 are needed later for the forwarding unit
    
    //output reg Branch, MemRead ,MemWrite, ALUsrc1,ALUsrc2, Regwrite, j,
       //output reg [1:0] ALUOP,Memtoreg,
       //output reg [3:0] read_write
    
   //[`IR_shamt]
     
    
    mux2by1 muxr2(.in1(ID_EX_RegR2), .in2(ID_EX_Imm), .sel(ID_EX_Ctrl[8]), .out( ALU2muxOut));
    mux2by1 muxr1(.in1(ID_EX_RegR1), .in2(ID_EX_PC), .sel(ID_EX_Ctrl[9]), .out( ALU1muxOut));
    ALUControl ALUcon(.op({ID_EX_Ctrl[5], ID_EX_Ctrl[4]}), .inst(ID_EX_Inst), .sel(ALUcontrol));
    RCA beqadder (.A(ID_EX_Imm),.B(IF_ID_PC),.Sum(branchaddOut));
    prv32_ALU Alu(.a(ALU1muxOut), .b(ALU2muxOut),. shamt(ID_EX_Inst[`IR_shamt]),.r(ALUresult),.zf(zf), .cf(cf), .vf(vf), .sf(sf),.alufn(ALUcontrol));
    
    
      wire [31:0] EX_MEM_BranchAddOut, EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_Inst, EX_MEM_PCP4;
      wire [13:0] EX_MEM_Ctrl;
      wire [4:0] EX_MEM_Rd;
      wire EX_MEM_Zf, EX_MEM_Cf, EX_MEM_Vf, EX_MEM_Sf;
      Register #(300) EX_MEM (clk,reset,1'b1,
      { ID_EX_Ctrl, branchaddOut, zf,cf, vf, sf , ALUresult,ALU2muxOut , ID_EX_Rd, ID_EX_Inst, ID_EX_PCP4},
      {EX_MEM_Ctrl, EX_MEM_BranchAddOut,EX_MEM_Zf,EX_MEM_Cf,EX_MEM_Vf, EX_MEM_Sf,
      EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_Rd, EX_MEM_Inst, EX_MEM_PCP4} );
      
    branchUnit bu(.zf(EX_MEM_Zf), .cf(EX_MEM_Cf), .vf(EX_MEM_Vf), .sf(EX_MEM_Sf), .funct3(EX_MEM_Inst[`IR_funct3]),.IR(EX_MEM_Inst),.flag(branch));
    assign pcSel[0] = EX_MEM_Ctrl[7];
    assign pcSel[1] = branch;
    
    DataMem DataMemory(.clk(clk), .MemRead(EX_MEM_Ctrl[13]), .MemWrite(EX_MEM_Ctrl[12]), .addr({EX_MEM_ALU_out}), .data_in(EX_MEM_RegR2),.read_write(EX_MEM_Ctrl[3:0]), .data_out(WriteData));
    
        
    wire MEM_WB_andout;
    wire [31:0] MEM_WB_Mem_out, MEM_WB_ALU_out, MEM_WB_PCP4;
    wire [13:0] MEM_WB_Ctrl;
    wire [4:0] MEM_WB_Rd;
    wire [1:0] MEM_WB_PC_Sel;
    Register #(300) MEM_WB (clk,reset,1'b1,
    {pcSel, EX_MEM_Ctrl, WriteData, EX_MEM_ALU_out, EX_MEM_Rd, EX_MEM_PCP4},
    { MEM_WB_PC_Sel, MEM_WB_Ctrl,MEM_WB_Mem_out, MEM_WB_ALU_out,
    MEM_WB_Rd, MEM_WB_PCP4} );
    
    mux4by2 MemOrRes(.i0(MEM_WB_ALU_out), .i1(MEM_WB_Mem_out),.i2(MEM_WB_PCP4), .i3(0), .s({MEM_WB_Ctrl[12], MEM_WB_Ctrl[11]}), .out(writeToReg));
    
    
//    always@(ledSel)begin
//    if (ledSel==2'b00)
//    leds=instruction[15:0];
    
//    else if (ledSel==2'b01)
//    leds= instruction[31:16];
//    else if(ledSel==2'b10)
  
//    leds= {8'b00000000,ALUOP,ALUcontrol,zeroFlag,andout};
//    end
//    always @(ssdsel)begin
//    if (ssdsel==4'b0000)
//    sevensegin=pc;
//    else if(ssdsel==4'b0001)
//    sevensegin=pcadderOUT;
//    else if(ssdsel==4'b0010)
//    sevensegin=beqaddOut;
//    else if(ssdsel==4'b0011)
//    sevensegin=data;
//     else if(ssdsel==4'b0100)
//    sevensegin= Read1;
//      else if(ssdsel==4'b0101)
//    sevensegin= Read2;
//     else if(ssdsel==4'b0110)
//    sevensegin= writeToReg;
//     else if(ssdsel==4'b0111)
//    sevensegin= imm;
//      else if(ssdsel==4'b1000)
//    sevensegin= shiftout;
//     else if(ssdsel==4'b1001)
//    sevensegin= ALUmuxOut;
//       else if(ssdsel==4'b1010)
//    sevensegin= ALUresult;
//        else if(ssdsel==4'b1011)
//    sevensegin= WriteData;   
//    end
    
//    Four_Digit_Seven_Segment_Driver disp(.clk(clk),.num(sevensegin), .Anode(anode), .LED_out(ledout));
      
         
    
endmodule
`default_nettype wire
