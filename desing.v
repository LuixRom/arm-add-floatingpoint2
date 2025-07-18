`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//  Módulo de display de 4 dígitos (usa CLKdivider, hFSM y HexTo7Segment)
//////////////////////////////////////////////////////////////////////////////////
/*
module hex_display(
    input  wire       clk,
    input  wire       reset,
    input  wire [15:0] data,
    output wire [7:0] catode,
    output wire [3:0] anode
);
    wire scl_clk;
    wire [3:0] digit;

    CLKdivider u_div (
        .clk   (clk),
        .reset (reset),
        .t     (scl_clk)
    );

    hFSM u_fsm (
        .clk   (scl_clk),
        .reset (reset),
        .data  (data),
        .digit (digit),
        .anode (anode)
    );

    HexTo7Segment u_hex (
        .digit  (digit),
        .catode (catode)
    );
endmodule

*/
//Nivel 2: Memoria
module mem (
    input wire clk,
    input wire we,
    input wire [31:0] a,
    input wire [31:0] wd,
    output wire [31:0] rd
);
    reg [31:0] RAM [0:63];

    initial begin
    
    RAM[0]  = 32'hE04F000F;
    RAM[1]  = 32'hE2802005;
    RAM[2]  = 32'hE2403002;
    RAM[3]  = 32'hE0C45392;
    RAM[4]  = 32'hE5805064;
    RAM[5]  = 32'hE5804068;

        /*
        RAM[0]  = 32'hE3A00080;
        RAM[1]  = 32'hE3A0143D;
        RAM[2]  = 32'hE3A02607;
        RAM[3]  = 32'hE0811002;
        RAM[4]  = 32'hE3A02101;
        RAM[5]  = 32'hE3A03843;
        RAM[6]  = 32'hE3A04903;
        RAM[7]  = 32'hE0822003;
        RAM[8]  = 32'hE0822004;
        RAM[9]  = 32'hE3A0343D;
        RAM[10] = 32'hE3A0473E;
        RAM[11] = 32'hE0833004;
        RAM[12] = 32'hE3A044BE;
        RAM[13] = 32'hE3A0589E;
        RAM[14] = 32'hE0844005;
        RAM[15] = 32'hE1A05003;
        RAM[16] = 32'hE3A06C01;
        RAM[17] = 32'hE5865000;
        RAM[18] = 32'h00000000;
        RAM[19] = 32'h00000000;*/
        
    end

    // Lectura
    assign rd = RAM[a[31:2]];

    // Escritura
    always @(posedge clk) begin
        if (we)
            RAM[a[31:2]] <= wd;
    end
endmodule

//Nivel 2: Une el controller y el datapath
module arm (
clk,
reset,
MemWrite,
Adr,
WriteData,
ReadData
);
input wire clk;
input wire reset;
output wire MemWrite;
output wire [31:0] Adr;
output wire [31:0] WriteData;
input wire [31:0] ReadData;
wire [31:0] Instr;
wire [3:0] ALUFlags;
wire PCWrite;
  wire [1:0] RegWrite;
wire IRWrite;
wire AdrSrc;
wire [1:0] RegSrc;
wire [1:0] ALUSrcA;
wire [1:0] ALUSrcB;
wire [1:0] ImmSrc;
  wire [3:0] ALUControl;
wire [1:0] ResultSrc;
  wire lmulFlag;
  wire RegSrcMul;
  wire mullargo;
 
controller c(
.clk(clk),
.reset(reset),
.Instr(Instr),
.ALUFlags(ALUFlags),
.PCWrite(PCWrite),
.MemWrite(MemWrite),
.RegWrite(RegWrite),
.IRWrite(IRWrite),
.AdrSrc(AdrSrc),
.RegSrc(RegSrc),
.ALUSrcA(ALUSrcA),
.ALUSrcB(ALUSrcB),
.ResultSrc(ResultSrc),
.ImmSrc(ImmSrc),
      .ALUControl(ALUControl),
      .lmulFlag(lmulFlag),
      .RegSrcMul(RegSrcMul),
      .mullargo(mullargo)
);
datapath dp(
.clk(clk),
.reset(reset),
.Adr(Adr),
.WriteData(WriteData),
.ReadData(ReadData),
.Instr(Instr),
.ALUFlags(ALUFlags),
.PCWrite(PCWrite),
      .RegWrite(RegWrite),
.IRWrite(IRWrite),
.AdrSrc(AdrSrc),
.RegSrc(RegSrc),
.ALUSrcA(ALUSrcA),
.ALUSrcB(ALUSrcB),
.ResultSrc(ResultSrc),
.ImmSrc(ImmSrc),
      .ALUControl(ALUControl),
      .lmulFlag(lmulFlag),
      .RegSrcMul(RegSrcMul),
      .mullargo(mullargo)
);
endmodule

//Contiene la lógica condlogic, decode
module controller (
clk,
reset,
Instr,
ALUFlags,
PCWrite,
MemWrite,
RegWrite,
IRWrite,
AdrSrc,
RegSrc,
ALUSrcA,
ALUSrcB,
ResultSrc,
ImmSrc,
ALUControl,
  lmulFlag,
  RegSrcMul,
  mullargo
);
input wire clk;
input wire reset;
  input wire [31:0] Instr;
input wire [3:0] ALUFlags;
output wire PCWrite;
output wire MemWrite;
output wire RegWrite;
output wire IRWrite;
output wire AdrSrc;
output wire [1:0] RegSrc;
output wire [1:0] ALUSrcA;
output wire [1:0] ALUSrcB;
output wire [1:0] ResultSrc;
output wire [1:0] ImmSrc;
  output wire [3:0] ALUControl;
  output wire lmulFlag;
  output wire RegSrcMul;
  output wire mullargo;
 
wire [1:0] FlagW;
wire PCS;
wire NextPC;
wire RegW;
  wire RegW2;
wire MemW;
 
 
decode dec(
.clk(clk),
.reset(reset),
.Op(Instr[27:26]),
      .Mop(Instr[7:4]),
.Funct(Instr[25:20]),
.Rd(Instr[15:12]),
.FlagW(FlagW),
.PCS(PCS),
.NextPC(NextPC),
.RegW(RegW),
      .RegW2(RegW2),
.MemW(MemW),
.IRWrite(IRWrite),
.AdrSrc(AdrSrc),
.ResultSrc(ResultSrc),
.ALUSrcA(ALUSrcA),
.ALUSrcB(ALUSrcB),
.ImmSrc(ImmSrc),
.RegSrc(RegSrc),
      .ALUControl(ALUControl),
      .lmulFlag(lmulFlag),
      .RegSrcMul(RegSrcMul),
      .mullargo(mullargo)
);
condlogic cl(
.clk(clk),
.reset(reset),
.Cond(Instr[31:28]),
.ALUFlags(ALUFlags),
.FlagW(FlagW),
.PCS(PCS),
.NextPC(NextPC),
.RegW(RegW),
      .RegW2(RegW2),
.MemW(MemW),
.PCWrite(PCWrite),
.RegWrite(RegWrite),
.MemWrite(MemWrite)
);
endmodule

module decode (
clk,
reset,
Op,
  Mop,
Funct,
Rd,
FlagW,
PCS,
NextPC,
RegW,
  RegW2,
MemW,
IRWrite,
AdrSrc,
ResultSrc,
ALUSrcA,
ALUSrcB,
ImmSrc,
RegSrc,
ALUControl,
  lmulFlag,
  RegSrcMul,
  mullargo
);
input wire clk;
input wire reset;
input wire [1:0] Op;
input wire [5:0] Funct;
  input wire [3:0] Mop;
input wire [3:0] Rd;
output reg [1:0] FlagW;
output wire PCS;
output wire NextPC;
output wire RegW;
  output wire RegW2;
output wire MemW;
output wire IRWrite;
output wire AdrSrc;
output wire [1:0] ResultSrc;
output wire [1:0] ALUSrcA;
output wire [1:0] ALUSrcB;
output wire [1:0] ImmSrc;
output wire [1:0] RegSrc;
  output wire lmulFlag;
  output reg [3:0] ALUControl;
wire Branch;
wire ALUOp;
  output wire RegSrcMul;//Cambios en los operandos
  output wire mullargo;
  reg mullargo_reg;
 
  assign mullargo = mullargo_reg;

// Main FSM
mainfsm fsm(
.clk(clk),
.reset(reset),
.Op(Op),
.Funct(Funct),
.IRWrite(IRWrite),
.AdrSrc(AdrSrc),
.ALUSrcA(ALUSrcA),
.ALUSrcB(ALUSrcB),
.ResultSrc(ResultSrc),
.NextPC(NextPC),
.RegW(RegW),
      .RegW2(RegW2),
.MemW(MemW),
.Branch(Branch),
      .ALUOp(ALUOp),
      .mullargo(mullargo),
        .lmulFlag(lmulFlag)
);

// ADD CODE BELOW
// Add code for the ALU Decoder and PC Logic.
// Remember, you may reuse code from previous labs.
// ALU Decoder
always @(*) begin
      mullargo_reg = 0;
    if (ALUOp) begin
        if (Mop[3:0] == 4'b1001) begin
            case (Funct[4:1])
                4'b0000: ALUControl = 4'b0101;       // MUL
                  4'b0100: begin
                         ALUControl = 4'b0110; //UMULL
                      mullargo_reg = 1;
                    end
                 
                  4'b0110: begin
                         ALUControl = 4'b0111; //SMULL
                      mullargo_reg = 1;
                    end
            endcase
        end
          else if(Mop[3:0] == 4'b0111) begin
            if (Funct[4:1] == 4'b0001)
              ALUControl= 4'b1000;  //UDIV
          end
          else begin
            case (Funct[4:1])
                4'b0100: ALUControl = 4'b0000;       // ADD
                4'b0010: ALUControl = 4'b0001;       // SUB
                4'b0000: ALUControl = 4'b0010;       // AND
                4'b1100: ALUControl = 4'b0011;       // ORR
                4'b0001: ALUControl = 4'b0100;       // EOR
                  4'b1101: ALUControl = 4'b1001;  // MOV
                  4'b1111: ALUControl = 4'b1010;  // MVN
                default: ALUControl = 3'bxxx;
            endcase
        end
        FlagW[1] = Funct[0];
        FlagW[0] = Funct[0] & ((ALUControl == 4'b0000) | (ALUControl == 4'b0001));
    end
     
      //
    else begin
        ALUControl = 4'b0000;
        FlagW = 2'b00;
    end
end
// PC Logic
assign PCS = ((Rd == 4'b1111) & RegW) | Branch;
// Add code for the Instruction Decoder (Instr Decoder) below.
// Recall that the input to Instr Decoder is Op, and the outputs are
// ImmSrc and RegSrc. We've completed the ImmSrc logic for you.

// Instr Decoder
assign ImmSrc = Op;
  assign RegSrc[1] = Op == 2'b01; // RegSrc1 is 1 for STR, 0 for DP and the rest don't care
assign RegSrc[0] = Op == 2'b10; // RegSrc0 is only 1 for B instructions
         
    assign RegSrcMul = Mop[3:0] == 4'b1001;
endmodule

module mainfsm (
clk,
reset,
Op,
Funct,
IRWrite,
AdrSrc,
ALUSrcA,
ALUSrcB,
ResultSrc,
NextPC,
RegW,
  RegW2,
MemW,
Branch,
ALUOp,
  mullargo,
  lmulFlag
);
input wire clk;
input wire reset;
input wire [1:0] Op;
input wire [5:0] Funct;
output wire IRWrite;
output wire AdrSrc;
output wire [1:0] ALUSrcA;
output wire [1:0] ALUSrcB;
output wire [1:0] ResultSrc;
output wire NextPC;
output wire RegW;
  output wire RegW2;
output wire MemW;
output wire Branch;
output wire ALUOp;
  output wire lmulFlag;
reg [3:0] state;
reg [3:0] nextstate;
  reg [13:0] controls;
    input wire mullargo;
 
localparam [3:0] FETCH = 0;
localparam [3:0] DECODE = 1;
localparam [3:0] MEMADR = 2;
localparam [3:0] MEMRD = 3;
localparam [3:0] MEMWB = 4;
localparam [3:0] MEMWR = 5;
localparam [3:0] EXECUTER = 6;
localparam [3:0] EXECUTEI = 7;
localparam [3:0] ALUWB = 8;
localparam [3:0] BRANCH = 9;
localparam [3:0] UNKNOWN = 10;
  localparam [3:0] ALUWB2= 11;

// state register
always @(posedge clk or posedge reset)
if (reset)
state <= FETCH;
else
state <= nextstate;


// ADD CODE BELOW
  // Finish entering the next state logic below.  We've completed the
  // first two states, FETCH and DECODE, for you.

  // next state logic
always @(*)
casex (state)
FETCH: nextstate = DECODE;
DECODE:
case (Op)
2'b00:
if (Funct[5])
nextstate = EXECUTEI;
else
nextstate = EXECUTER;
2'b01: nextstate = MEMADR;
2'b10: nextstate = BRANCH;
default: nextstate = UNKNOWN;
endcase
MEMADR:
if (Funct[0])
nextstate = MEMRD;
else
nextstate = MEMWR;
         
MEMRD: nextstate = MEMWB;
MEMWB: nextstate = FETCH;
MEMWR: nextstate = FETCH;
EXECUTER: nextstate = mullargo == 1? ALUWB2 : ALUWB;
EXECUTEI: nextstate = mullargo == 1? ALUWB2 : ALUWB;
ALUWB: nextstate = FETCH;
BRANCH: nextstate = FETCH;
          ALUWB2:   nextstate = FETCH;
default: nextstate = FETCH;
endcase

// ADD CODE BELOW
// Finish entering the output logic below.  We've entered the
// output logic for the first two states, FETCH and DECODE, for you.

// state-dependent output logic
always @(*) begin
case (state)
FETCH: controls =    15'b010001010011000;
DECODE: controls =   15'b000000010011000;
MEMADR: controls =   15'b000000000000100;
MEMRD: controls =    15'b000000100000000;
MEMWB: controls =    15'b000010001000000;
MEMWR: controls =    15'b000100100000000;
EXECUTER: controls = 15'b000000000000010;
EXECUTEI: controls = 15'b000000000000110;
ALUWB: controls =    15'b000010000000000;
          ALUWB2: controls = 15'b100010000000001;
BRANCH: controls =   15'b001000010100100;
default: controls =  15'bxxxxxxxxxxxxxxx;
endcase
end
  assign {RegW2, NextPC, Branch, MemW, RegW, IRWrite, AdrSrc,
        ResultSrc, ALUSrcA, ALUSrcB, ALUOp, lmulFlag} = controls;
endmodule

// ADD CODE BELOW
// Add code for the condlogic and condcheck modules. Remember, you may
// reuse code from prior labs.
module condlogic (
clk,
reset,
Cond,
ALUFlags,
FlagW,
PCS,
NextPC,
RegW,
  RegW2,
MemW,
PCWrite,
RegWrite,
MemWrite
);
input wire clk;
input wire reset;
input wire [3:0] Cond;
input wire [3:0] ALUFlags;
input wire [1:0] FlagW;
input wire PCS;
input wire NextPC;
input wire RegW;
  input wire RegW2;
input wire MemW;
output wire PCWrite;
  output wire [1:0] RegWrite;
output wire MemWrite;
wire [1:0] FlagWrite;
wire [3:0] Flags;
wire CondEx;
 
  wire actualCondEx;
  wire PCSrc;


// ADD CODE HERE
flopenr #(2) flagreg1 (     // N y Z
        .clk(clk),
.reset(reset),
.en(FlagWrite[1]),
.d(ALUFlags[3:2]),
.q(Flags[3:2])          
    );
 
  flopenr #(2) flagreg0 (    
        .clk(clk),
.reset(reset),
.en(FlagWrite[0]),
.d(ALUFlags[1:0]),
.q(Flags[1:0])
    );
 
  flopr #(1) condexreg (
        .clk(clk),
.reset(reset),
.d(CondEx),
.q(actualCondEx)
    );
 
  condcheck cc (
        .Cond(Cond),
        .Flags(Flags),
        .CondEx(CondEx)
    );
 
  assign FlagWrite = FlagW & {2 {CondEx}};
  assign RegWrite[0] = RegW & actualCondEx;
  assign RegWrite[1]  = RegW2  & actualCondEx;
    assign MemWrite = MemW & actualCondEx;
    assign PCSrc   = PCS & actualCondEx;
    assign PCWrite = PCSrc | NextPC;
 
endmodule

module condcheck (
Cond,
Flags,
CondEx
);
input wire [3:0] Cond;
input wire [3:0] Flags;
output reg CondEx;

// ADD CODE HERE
  wire neg;
wire zero;
wire carry;
wire overflow;
wire ge;
assign {neg, zero, carry, overflow} = Flags;
assign ge = neg == overflow;
always @(*)
case (Cond)
4'b0000: CondEx = zero;
4'b0001: CondEx = ~zero;
4'b0010: CondEx = carry;
4'b0011: CondEx = ~carry;
4'b0100: CondEx = neg;
4'b0101: CondEx = ~neg;
4'b0110: CondEx = overflow;
4'b0111: CondEx = ~overflow;
4'b1000: CondEx = carry & ~zero;
4'b1001: CondEx = ~(carry & ~zero);
4'b1010: CondEx = ge;
4'b1011: CondEx = ~ge;
4'b1100: CondEx = ~zero & ge;
4'b1101: CondEx = ~(~zero & ge);
4'b1110: CondEx = 1'b1;
default: CondEx = 1'bx;
endcase
endmodule

// ADD CODE BELOW
// Complete the datapath module below for Lab 11.
// You do not need to complete this module for Lab 10.
// The datapath unit is a structural SystemVerilog module. That is,
// it is composed of instances of its sub-modules. For example,
// the instruction register is instantiated as a 32-bit flopenr.
// The other submodules are likewise instantiated.
module datapath (
clk,
reset,
Adr,
WriteData,
ReadData,
Instr,
ALUFlags,
PCWrite,
RegWrite,
IRWrite,
AdrSrc,
RegSrc,
ALUSrcA,
ALUSrcB,
ResultSrc,
ImmSrc,
ALUControl,
  lmulFlag,
  RegSrcMul,
  mullargo,
);
input wire clk;
input wire reset;
output wire [31:0] Adr;
output wire [31:0] WriteData;
input wire [31:0] ReadData;
output wire [31:0] Instr;
output wire [3:0] ALUFlags;
input wire PCWrite;
input wire RegWrite;
input wire IRWrite;
input wire AdrSrc;
input wire [1:0] RegSrc;
input wire [1:0] ALUSrcA;
input wire [1:0] ALUSrcB;
input wire [1:0] ResultSrc;
input wire [1:0] ImmSrc;
  input wire [3:0] ALUControl;
 
  input wire lmulFlag;
  input wire RegSrcMul;
  input wire mullargo;
 
wire [31:0] PCNext;
wire [31:0] PC;
wire [31:0] ExtImm;
wire [31:0] SrcA;
wire [31:0] SrcB;
wire [31:0] Result;
wire [31:0] Data;
wire [31:0] RD1;
wire [31:0] RD2;
wire [31:0] A;
wire [31:0] ALUResult;
    wire [31:0] ALUResult2;
wire [31:0] ALUOut;
  wire [31:0] ALUOut2;
wire [3:0] RA1;
wire [3:0] RA2;
 
 
  wire [3:0] _RA1, _RA2, A3;

// Your datapath hardware goes below. Instantiate each of the
// submodules that you need. Remember that you can reuse hardware
// from previous labs. Be sure to give your instantiated modules
// applicable names such as pcreg (PC register), adrmux
// (Address Mux), etc. so that your code is easier to understand.

// ADD CODE HERE
  flopenr #(32) pcreg(
.clk(clk),
.reset(reset),
.en(PCWrite),
.d(Result),
.q(PC)
);
mux2 #(32) adrmux(
.d0(PC),
.d1(Result),
.s(AdrSrc),
.y(Adr)
);
// here goes (implicitly) the instruction/data memory
flopenr #(32) instrreg(
.clk(clk),
.reset(reset),
.en(IRWrite),
.d(ReadData),
.q(Instr)
);
 
flopr #(32) readdatareg(
.clk(clk),
.reset(reset),
.d(ReadData),
.q(Data)
);
 
 
 

  mux2 #(4) ra1mulmux(
      .d0(Instr[19:16]),
      .d1(Instr[3:0]),
      .s(RegSrcMul),
      .y(_RA1)
    );
 
    mux2 #(4) ra1mux(
    .d0(_RA1),
    .d1(4'd15),
    .s(RegSrc[0]),
    .y(RA1)
);
 
  mux2 #(4) ra2mulmux(
    .d0(Instr[3:0]),    // Rm normal
    .d1(Instr[11:8]),   // Rs para MUL
    .s(RegSrcMul),
    .y(_RA2)
);
 
 
  mux2 #(4) ra2mux(
    .d0(_RA2),
    .d1(Instr[15:12]),
    .s(RegSrc[1]),
    .y(RA2)
);
 
  mux2 #(4) a3mux(
    .d0(Instr[15:12]),  // Rd normal
    .d1(Instr[19:16]),  // Rn como destino en MUL
    .s(RegSrcMul),
    .y(A3)
);
 
  regfile rf(
.clk(clk),
.we3(RegWrite),
.ra1(RA1),
.ra2(RA2),
      .wa3(A3),
      .wa4(Instr[15:12]),
.wd3(Result),
      .wd4(ALUOut2),
      .mullargo(lmulFlag),
.r15(Result),
.rd1(RD1),
.rd2(RD2)
);
 
 
  extend ext(
.Instr(Instr[23:0]),
.ImmSrc(ImmSrc),
.ExtImm(ExtImm)
    );
 
 
  flopr #(64) rdreg(
      .clk(clk),
      .reset(reset),
      .d({RD1, RD2}),
      .q({A, WriteData})
    );
 
  mux2 #(32) srcamux(
.d0(A),
.d1(PC),
      .s(ALUSrcA[0]),
.y(SrcA)
);
 
  mux3 #(32) srcbmux(
.d0(WriteData),
.d1(ExtImm),
      .d2(32'd4),
.s(ALUSrcB),
.y(SrcB)
);
 
 

  alu alu(
SrcA,
SrcB,
ALUControl,
      mullargo,
ALUResult,
      ALUResult2,
ALUFlags
);
 


flopr #(32) aluresultreg(
.clk(clk),
.reset(reset),
.d(ALUResult),
.q(ALUOut)
);
 
  flopr #(32) aluresultreg2(
.clk(clk),
.reset(reset),
    .d(ALUResult2),
    .q(ALUOut2)
);
 
mux3 #(32) resmux(
.d0(ALUOut),
.d1(Data),
.d2(ALUResult),
.s(ResultSrc),
.y(Result)
);
endmodule

// ADD CODE BELOW
// Add needed building blocks below (i.e., parameterizable muxes,
// registers, etc.). Remember, you can reuse code from previous labs.
// We've also provided a parameterizable 3:1 mux below for your
// convenience.

module mux3 (
d0,
d1,
d2,
s,
y
);
parameter WIDTH = 8;
input wire [WIDTH - 1:0] d0;
input wire [WIDTH - 1:0] d1;
input wire [WIDTH - 1:0] d2;
input wire [1:0] s;
output wire [WIDTH - 1:0] y;
assign y = (s[1] ? d2 : (s[0] ? d1 : d0));
endmodule


module flopr (
clk,
reset,
d,
q
);
parameter WIDTH = 8;
input wire clk;
input wire reset;
input wire [WIDTH - 1:0] d;
output reg [WIDTH - 1:0] q;
always @(posedge clk or posedge reset)
if (reset)
q <= 0;
else
q <= d;
endmodule


module flopr2 (
clk,
reset,
d0,
    d1,
q0,
    q1
);
parameter WIDTH = 8;
input wire clk;
input wire reset;
input wire [WIDTH - 1:0] d0;
    input wire [WIDTH - 1:0] d1;
output reg [WIDTH - 1:0] q0;
    output reg [WIDTH - 1:0] q1;
always @(posedge clk or posedge reset) begin
if (reset) begin
q0 <= 0;
            q1 <= 0;
        end
else begin
q0 <= d0;
            q1 <= d1;
        end
    end
endmodule

module extend (
Instr,
ImmSrc,
ExtImm
);
input wire [23:0] Instr;
input wire [1:0] ImmSrc;
output reg [31:0] ExtImm;
  wire [31:0] extend={24'b000000000000000000000000, Instr[7:0]};
  wire[31:0] rotado=(extend >> Instr[11:8]*2) | (extend << (32 -Instr[11:8]*2));

   
always @(*)
case (ImmSrc)
2'b00: ExtImm = rotado;
2'b01: ExtImm = {20'b00000000000000000000, Instr[11:0]};
2'b10: ExtImm = {{6 {Instr[23]}}, Instr[23:0], 2'b00};
default: ExtImm = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
endcase
endmodule

module regfile (
clk,
we3,
ra1,
ra2,
wa3,
  wa4,
wd3,
  wd4,
  mullargo,
r15,
rd1,
rd2
);
input wire clk;
input wire we3;
input wire [3:0] ra1;
input wire [3:0] ra2;
input wire [3:0] wa3;
  input wire [3:0] wa4;
input wire [31:0] wd3;
  input wire [31:0] wd4;
  input wire mullargo;
input wire [31:0] r15;
output wire [31:0] rd1;
output wire [31:0] rd2;
reg [31:0] rf [14:0];
always @(posedge clk)
if (we3) begin
rf[wa3] <= wd3;
          if(mullargo) begin
            rf[wa4] <=wd4;
          end
end
assign rd1 = (ra1 == 4'b1111 ? r15 : rf[ra1]);
assign rd2 = (ra2 == 4'b1111 ? r15 : rf[ra2]);
endmodule

module mux2 (
d0,
d1,
s,
y
);
parameter WIDTH = 8;
input wire [WIDTH - 1:0] d0;
input wire [WIDTH - 1:0] d1;
input wire s;
output wire [WIDTH - 1:0] y;
assign y = (s ? d1 : d0);
endmodule

module flopenr (
clk,
reset,
en,
d,
q
);
parameter WIDTH = 8;
input wire clk;
input wire reset;
input wire en;
input wire [WIDTH - 1:0] d;
output reg [WIDTH - 1:0] q;
always @(posedge clk or posedge reset)
if (reset)
q <= 0;
else if (en)
q <= d;
endmodule


module alu (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [3:0]  ALUControl,   // ahora de 4 bits
    input  wire        mullargo,    
    output reg  [31:0] Result,      
    output reg  [31:0] Result2,      
    output wire [3:0]  ALUFlags
);

    // Preparaciones auxiliares
    wire [31:0] condinvb = ALUControl[0] ? ~b : b;
    wire [32:0] sum = a + condinvb + ALUControl[0];

    // Multiplicaciones
    wire [63:0] umul_result = a * b;
    wire signed [31:0] sa = a;
    wire signed [31:0] sb = b;
    wire signed [63:0] smul_result = sa * sb;

    // Punto flotante
    wire [31:0] sumaFP;
    wire [31:0] mulFP;
    wire [63:0] sumaFP64;
    wire [63:0] mulFP64;
  	wire [63:0] a_double = {Result2, a};  
    wire [63:0] b_double = {Result2, b};
    AddFP fpadd(.a(a), .b(b), .f(sumaFP));
    MulFP fpmul(.a(a), .b(b), .f(mulFP));
    AddFP_Double fpadd64(.a(a_double), .b(b_double), .f(sumaFP64));
    MulFP_Double fpmul64(.a(a_double), .b(b_double), .f(mulFP64));  	
  

    always @(*) begin
        Result  = 32'd0;
        Result2 = 32'd0;

        case (ALUControl)

            4'b0000,  // ADD
            4'b0001:  // SUB o CMP
                Result = sum[31:0];

            4'b0010:  Result = a & b;       // AND
            4'b0011:  Result = a | b;       // ORR
            4'b0100:  Result = a ^ b;       // EOR
            4'b0101:  Result = a * b;       // MUL (32-bit)

            4'b0110: begin                  // UMULL o UDIV
                if (mullargo) begin
                    Result  = umul_result[31:0];
                    Result2 = umul_result[63:32];
                end else begin
                    Result  = (b != 0) ? a / b : 32'd0;
                    Result2 = (b != 0) ? a % b : 32'd0;
                end
            end

            4'b0111: begin                  // SMULL
                Result  = smul_result[31:0];
                Result2 = smul_result[63:32];
            end

            4'b1001: Result = b;            // MOV
            4'b1010: Result = ~b;           // MVN

            4'b1101: Result = sumaFP;       // FP ADD
          
            4'b1111: begin  // FP ADD DOUBLE
                Result  = sumaFP64[31:0];     
                Result2 = sumaFP64[63:32];    
            end

            4'b1110: begin
    		if (mullargo) begin  // reutilizando mullargo como double_flag si lo defines así
        			Result  = mulFP64[31:0];      
        			Result2 = mulFP64[63:32];
    			end else begin
       	 			Result = mulFP;
        			Result2 = 32'd0;
    			end
			end      

            default: begin
                Result  = 32'd0;
                Result2 = 32'd0;
            end

        endcase
    end

    wire isFPAdd    = (ALUControl == 4'b1101);
    wire isFPMul    = (ALUControl == 4'b1110) && !mullargo;
    wire isFPAdd64  = (ALUControl == 4'b1111);
    wire isFPMul64  = (ALUControl == 4'b1110) && mullargo;

    wire uses64bits = (ALUControl == 4'b0110 || ALUControl == 4'b0111 ||
                       isFPAdd64 || isFPMul64);

    wire N = uses64bits ? Result2[31] : Result[31];

    wire Z = uses64bits ? ((Result == 32'd0) && (Result2 == 32'd0))
                        : (Result == 32'd0);

    wire C = (!mullargo && ALUControl[1:0] == 2'b00) ? sum[32] : 1'b0;

    wire V = (!mullargo && ALUControl[1:0] == 2'b00)
             ? ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31])
             : 1'b0;

    assign ALUFlags = {N, Z, C, V};

endmodule


module AddFP(
  input  [31:0] a,         // IEEE-754 float
  input  [31:0] b,
  output reg [31:0] f
);
  wire sign1 = a[31];
  wire sign2 = b[31];
  wire [7:0] exponent1 = a[30:23];
  wire [7:0] exponent2 = b[30:23];
  wire [22:0] fraction1 = a[22:0];
  wire [22:0] fraction2 = b[22:0];

  wire [23:0] mantissa1 = (exponent1 == 0) ? {1'b0, fraction1} : {1'b1, fraction1};
  wire [23:0] mantissa2 = (exponent2 == 0) ? {1'b0, fraction2} : {1'b1, fraction2};

  reg [23:0] mantissa_a, mantissa_b;
  reg [7:0] exponent_result;
  reg [24:0] sum;
  reg sign_result;
  reg [22:0] result_fraction;

  always @(*) begin
    if (a == 0) begin f = b; return; end
    if (b == 0) begin f = a; return; end

    if (exponent1 > exponent2) begin
      mantissa_a = mantissa1;
      mantissa_b = mantissa2 >> (exponent1 - exponent2);
      exponent_result = exponent1;
      sign_result = sign1;
    end else begin
      mantissa_a = mantissa2;
      mantissa_b = mantissa1 >> (exponent2 - exponent1);
      exponent_result = exponent2;
      sign_result = sign2;
    end

    if (sign1 == sign2)
      sum = mantissa_a + mantissa_b;
    else begin
      if (mantissa_a >= mantissa_b) begin
        sum = mantissa_a - mantissa_b;
        sign_result = (exponent1 >= exponent2) ? sign1 : sign2;
      end else begin
        sum = mantissa_b - mantissa_a;
        sign_result = (exponent2 >= exponent1) ? sign2 : sign1;
      end
    end

    if (sum[24]) begin
      sum = sum >> 1;
      exponent_result = exponent_result + 1;
    end else begin
      while (sum[23] == 0 && exponent_result > 0 && sum != 0) begin
        sum = sum << 1;
        exponent_result = exponent_result - 1;
      end
    end

    result_fraction = sum[22:0];
    f = {sign_result, exponent_result, result_fraction};
  end
endmodule


module MulFP(
  input  [31:0] a,         // IEEE-754 float
  input  [31:0] b,
  output reg [31:0] f
);
  wire sign1 = a[31];
  wire sign2 = b[31];
  wire [7:0] exponent1 = a[30:23];
  wire [7:0] exponent2 = b[30:23];
  wire [22:0] fraction1 = a[22:0];
  wire [22:0] fraction2 = b[22:0];

  wire [23:0] mantissa1 = (exponent1 == 0) ? {1'b0, fraction1} : {1'b1, fraction1};
  wire [23:0] mantissa2 = (exponent2 == 0) ? {1'b0, fraction2} : {1'b1, fraction2};

  reg [47:0] product;
  reg [7:0] exponent_result;
  reg result_sign;
  reg [22:0] result_fraction;

  always @(*) begin
    if (a == 0 || b == 0) begin f = 32'd0; return; end

    result_sign = sign1 ^ sign2;
    exponent_result = exponent1 + exponent2 - 8'd127;
    product = mantissa1 * mantissa2;

    if (product[47]) begin
      product = product >> 1;
      exponent_result = exponent_result + 1;
    end else begin
      while (!product[46] && exponent_result > 0) begin
        product = product << 1;
        exponent_result = exponent_result - 1;
      end
    end

    result_fraction = product[45:23];
    f = {result_sign, exponent_result, result_fraction};
  end
endmodule

module AddFP_Double(
  input  [63:0] a,
  input  [63:0] b,
  output reg [63:0] f
);
  wire sign1 = a[63];
  wire sign2 = b[63];
  wire [10:0] exponent1 = a[62:52];
  wire [10:0] exponent2 = b[62:52];
  wire [51:0] fraction1 = a[51:0];
  wire [51:0] fraction2 = b[51:0];

  wire [52:0] mantissa1 = (exponent1 == 0) ? {1'b0, fraction1} : {1'b1, fraction1};
  wire [52:0] mantissa2 = (exponent2 == 0) ? {1'b0, fraction2} : {1'b1, fraction2};

  reg [52:0] mantissa_a, mantissa_b;
  reg [10:0] exponent_result;
  reg [53:0] sum;
  reg result_sign;
  reg [51:0] result_fraction;

  always @(*) begin
    if (a == 0) begin f = b; return; end
    if (b == 0) begin f = a; return; end

    if (exponent1 > exponent2) begin
      mantissa_a = mantissa1;
      mantissa_b = mantissa2 >> (exponent1 - exponent2);
      exponent_result = exponent1;
      result_sign = sign1;
    end else begin
      mantissa_a = mantissa2;
      mantissa_b = mantissa1 >> (exponent2 - exponent1);
      exponent_result = exponent2;
      result_sign = sign2;
    end

    if (sign1 == sign2)
      sum = mantissa_a + mantissa_b;
    else begin
      if (mantissa_a >= mantissa_b) begin
        sum = mantissa_a - mantissa_b;
        result_sign = (exponent1 >= exponent2) ? sign1 : sign2;
      end else begin
        sum = mantissa_b - mantissa_a;
        result_sign = (exponent2 >= exponent1) ? sign2 : sign1;
      end
    end

    while (sum[53] == 0 && exponent_result > 0 && sum != 0) begin
      sum = sum << 1;
      exponent_result = exponent_result - 1;
    end

    if (sum[53]) begin
      sum = sum >> 1;
      exponent_result = exponent_result + 1;
    end

    result_fraction = sum[51:0];
    f = {result_sign, exponent_result, result_fraction};
  end
endmodule

module MulFP_Double(
  input  [63:0] a,
  input  [63:0] b,
  output reg [63:0] f
);
  wire sign1 = a[63];
  wire sign2 = b[63];
  wire [10:0] exponent1 = a[62:52];
  wire [10:0] exponent2 = b[62:52];
  wire [51:0] fraction1 = a[51:0];
  wire [51:0] fraction2 = b[51:0];

  wire [52:0] mantissa1 = (exponent1 == 0) ? {1'b0, fraction1} : {1'b1, fraction1};
  wire [52:0] mantissa2 = (exponent2 == 0) ? {1'b0, fraction2} : {1'b1, fraction2};

  reg [105:0] product;
  reg [10:0] exponent_result;
  reg result_sign;
  reg [51:0] result_fraction;

  always @(*) begin
    if (a == 0 || b == 0) begin f = 64'd0; return; end

    result_sign = sign1 ^ sign2;
    exponent_result = exponent1 + exponent2 - 11'd1023;
    product = mantissa1 * mantissa2;

    if (product[105]) begin
      product = product >> 1;
      exponent_result = exponent_result + 1;
    end else begin
      while (!product[104] && exponent_result > 0) begin
        product = product << 1;
        exponent_result = exponent_result - 1;
      end
    end

    result_fraction = product[104:53];
    f = {result_sign, exponent_result, result_fraction};
  end
endmodule


module clockdivider2(input in_clk , output reg out_clk);
reg [23:0] counter = 1;
always @(posedge in_clk) begin
counter <= counter + 1;
if (counter == 0) out_clk <= ~out_clk;
end
endmodule

//Listo
module HexTo7Segment (
    input [3:0] digit,  // 4-bit hexadecimal input
    output reg [7:0] catode  // 7-segment output (7 segments + DP)
);
    always @(*) begin
        case(digit)
            4'h0: catode = 8'b00000011;
            4'h1: catode = 8'b10011111;
            4'h2: catode = 8'b00100101;
            4'h3: catode = 8'b00001101;
            4'h4: catode = 8'b10011001;
            4'h5: catode = 8'b01001001;
            4'h6: catode = 8'b01000001;
            4'h7: catode = 8'b00011111;
            4'h8: catode = 8'b00000001;
            4'h9: catode = 8'b00011001;
            4'hA: catode = 8'b00010001;
            4'hB: catode = 8'b11000001;
            4'hC: catode = 8'b01100011;
            4'hD: catode = 8'b10000101;
            4'hE: catode = 8'b01100001;
            4'hF: catode = 8'b01110001;
            default: catode = 8'b11111111;  // Blank display
        endcase
    end
endmodule
module hFSM (
    input clk,
    input reset,
    input [15:0] data,
    output reg [3:0] digit,
    output reg [3:0] anode
);

    reg [1:0] digit_counter;

    always @(posedge clk or posedge reset) begin
        if (reset)
            digit_counter <= 2'b00;
        else
            digit_counter <= digit_counter + 1;
    end

    always @(*) begin
        // Valores por defecto
        digit = 4'b0000;
        anode = 4'b1111;

        case(digit_counter)
            2'b00: begin
                digit = data[15:12];  // Dígito más significativo
                anode = 4'b0111;      // Display de la izquierda
            end
            2'b01: begin
                digit = data[11:8];
                anode = 4'b1011;
            end
            2'b10: begin
                digit = data[7:4];
                anode = 4'b1101;
            end
            2'b11: begin
                digit = data[3:0];    // Dígito menos significativo
                anode = 4'b1110;      // Display de la derecha
            end
        endcase
    end

endmodule



module hex_display(
    input clk,
    input reset,
    input [15:0] data,
    output wire [7:0] catode,
    output wire [3:0] anode
);

    wire scl_clk;
    wire [3:0] digit;

    clockdivider sc (
        .in_clk(clk),
        .out_clk(scl_clk)
    );

    hFSM m (
        .clk(scl_clk),
        .reset(reset),
        .data(data),
        .digit(digit),
        .anode(anode)
    );

    HexTo7Segment decoder (
        .digit(digit),
        .catode(catode)
    );
    
endmodule
