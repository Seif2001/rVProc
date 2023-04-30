`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/28/2023 10:05:17 AM
// Design Name: 
// Module Name: controlUnit
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
`include "defines.v"


module controlUnit(
input [31:0] IR,
output reg Branch, MemRead ,MemWrite, ALUsrc1,ALUsrc2, Regwrite, j,
output reg [1:0] ALUOP,Memtoreg,
output reg [3:0] read_write


);

always @(*) begin
    case (`OPCODE)
        `OPCODE_Arith_I   :begin
            Branch= 0;
            MemRead=0;
            MemWrite=0;
            ALUsrc1=0;
            ALUsrc2 = 1;
            ALUOP=2'b10;
            Memtoreg=2'b00;
            Regwrite=1;
            read_write=0;
            j = 0;    
         end
        `OPCODE_Store     :begin
            Branch= 0;
            MemRead=0;
            MemWrite=1;
            ALUsrc1=0;
            ALUsrc2 = 1;
            ALUOP=2'b00;
            Regwrite=0;
            if (IR[14:12]==3'b000)//sb
                read_write= 4'b1011;
            else if (IR[14:12]==3'b001)//sh
                read_write= 4'b1110;
            else if (IR[14:12]==3'b010)//sw
                read_write= 4'b1111;
            
            j = 0;      
          end
        `OPCODE_LUI       :     begin
            Branch= 0;
            MemRead=0;
            MemWrite=0;
            ALUsrc1=0;
            ALUsrc2 = 1;
            ALUOP=2'b10;
            Memtoreg=2'b00;
            Regwrite=1;
            read_write=0;
            j = 0;  
         end
        `OPCODE_AUIPC     :     begin
           Branch= 0;
           MemRead=0;
           MemWrite=0;
           ALUsrc1=1;
           ALUsrc2 = 1;
           ALUOP=2'b00;
           Memtoreg=2'b00;
           Regwrite=1;
           j = 0;      
         end
        `OPCODE_JAL       :    begin
            Branch= 1;
            MemRead=0;
            MemWrite=0;
            ALUsrc1=1;
            ALUsrc2 = 1;
            ALUOP=2'b00;
            Memtoreg=2'b10;
            Regwrite=1;
            j = 0;     
         end
        `OPCODE_JALR      :     begin
            Branch= 1;
            MemRead=0;
            MemWrite=0;
            ALUsrc1=0;
            ALUsrc2 = 1;
            ALUOP=2'b00;
            Memtoreg=2'b10;
            Regwrite=1;
            read_write=0;
            j = 1;  
         end
        `OPCODE_Branch    :     begin
            Branch= 1;
            MemRead=0;
            MemWrite=0;
            ALUsrc1=0;
            ALUsrc2=0;
            ALUOP=2'b01;
            Regwrite=0;
            read_write=0;
            j = 0;      
         end
        `OPCODE_Arith_R           :        begin
            Branch= 0;
            MemRead=0;
            MemWrite=0;
            ALUsrc1=0;
            ALUsrc2 = 0;
            ALUOP=2'b10;
            Memtoreg=2'b00;
            Regwrite=1;
            read_write=0;
            j = 0;    
          end
        `OPCODE_Load    :     begin
            Branch= 0;
            MemRead=1;
            MemWrite=0;
            ALUsrc1=0;
            ALUsrc2 = 1;
            ALUOP=2'b00;
            Memtoreg=2'b01;
            Regwrite=1;
            if (IR[14:12]==3'b000)//lb
                read_write= 4'b1000;
            else if (IR[14:12]==3'b001)//lh
                read_write= 4'b1001;
            else if (IR[14:12]==3'b010)//lw
                read_write= 4'b1010;
            else if (IR[14:12]==3'b100)//lbu
                read_write= 4'b1100;
            else if (IR[14:12]==3'b101)//lhu
                read_write= 4'b1101;
                
            j = 0;     
          end
          default:begin
                      Branch= 0;
                      MemRead=0;
                      MemWrite=0;
                      ALUsrc1=0;
                      ALUsrc2 = 0;
                      ALUOP=2'b00;
                      Memtoreg=2'b00;
                      Regwrite=0;
                      read_write=0;
                      j = 0;     
                    end
          
    endcase 
end

endmodule
