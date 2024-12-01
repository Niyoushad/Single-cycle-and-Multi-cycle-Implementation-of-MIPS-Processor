//===========================================================//
//
//			Name & Student ID:  Niyousha Dadkhah 401101687
//
//			Implemented Instructions are:
//			R format:  add(u), sub(u), and, or, xor, nor, slt, sltu;
//			I format:  beq, bne, lw, sw, addi(u), slti, sltiu, andi, ori, xori, lui.
//
//===========================================================//

`timescale 1ns/1ns

   `define ADD  4'h0
   `define SUB  4'h1
   `define SLT  4'h2
   `define SLTU 4'h3
   `define AND  4'h4
   `define OR   4'h5
   `define NOR  4'h6
   `define XOR  4'h7
   `define LUI  4'h8

   // op field lables
   `define R_type 6'b000000
   `define beq    6'b000100
   `define bne    6'b000101
   `define lw     6'b100011
   `define sw     6'b101011
   `define addi   6'b001000
   `define addiu  6'b001001
   `define slti   6'b001010
   `define sltiu  6'b001011
   `define andi   6'b001100
   `define ori    6'b001101
   `define xori   6'b001110
   `define lui    6'b001111

   `define add    6'b100000
   `define addu   6'b100001
   `define sub    6'b100010
   `define subu   6'b100011
   `define and    6'b100100
   `define or     6'b100101
   `define xor    6'b100110
   `define nor    6'b100111
   `define slt    6'b101010
   `define sltu   6'b101011


module single_cycle_mips 
(
	input clk,
	input reset
   //output SZEn, ALUSrc, RegDst, MemtoReg, RegWrite, MemWrite
);
 
	initial begin
		$display("Single Cycle MIPS Implemention");
		$display("Name & Student ID:  Niyousha Dadkhah 401101687");
	end
reg SZEn, ALUSrc, RegDst, MemtoReg, RegWrite, MemWrite;
	reg [31:0] PC;          // Keep PC as it is, its name is used in higher level test bench

   wire [31:0] instr;
   wire [ 5:0] op   = instr[31:26];
   wire [ 5:0] func = instr[ 5: 0];

   wire [31:0] RD1, RD2, AluResult, MemReadData;

   wire AluZero;

   // Control Signals

   wire PCSrc, PCSrc1;
   //***************I added*******************
   reg Branch, BranchNEQ;
   assign PCSrc = AluZero && Branch; //CORRECT??
   assign PCSrc1 = BranchNEQ && ~AluZero;



   
   //***************I added*******************
   


   reg [3:0] AluOP;

	
	// CONTROLLER COMES HERE

   always @(*) begin
      SZEn = 1'bx; // Zero / sign extend
      AluOP = 4'hx;
      ALUSrc = 1'bx;
      RegDst = 1'bx;
      MemtoReg = 1'bx;
      RegWrite = 1'b0;
      MemWrite = 1'b0;
      Branch = 1'bx;
      BranchNEQ = 1'bx;

      case(op)
         `R_type: begin 
            //SZEn STAY X
            RegWrite = 1;
            RegDst = 1;
            ALUSrc = 0;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            case (func) 
               `add  : AluOP = `ADD;
               `addu : AluOP = `ADD;
               `sub  : AluOP = `SUB;
               `subu : AluOP = `SUB;
               `and  : AluOP = `AND;
               `or   : AluOP = `OR;
               `xor  : AluOP = `XOR;
               `nor  : AluOP = `NOR;
               `slt  : AluOP = `SLT;
               `sltu : AluOP = `SLTU;
            endcase
         end 
         `beq: begin
            SZEn = 1;
            RegWrite = 0;
            //RegDst stay x 
            ALUSrc = 0;
            Branch = 1;
            BranchNEQ = 0;
            MemWrite = 0;
            //MemtoReg stay x
            AluOP = `SUB;
         end 
         `bne:begin
            SZEn = 1;
            RegWrite = 0;
            //RegDst stay x
            ALUSrc = 0;
            Branch = 0; //correct??
            BranchNEQ = 1;
            MemWrite = 0;
            //MemtoReg STAY X
            AluOP = `SUB;
         end
         `lw:begin
            SZEn = 1;
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 1;
            AluOP = `ADD;
         end 
         `sw:begin
            SZEn = 1;
            RegWrite = 0;
            //RegDst stay x
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 1;
            //MemtoReg stay x
            AluOP = `ADD;
         end 
         `addi:begin
            SZEn = 1;
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            AluOP = `ADD;
         end 
         `addiu:begin
            SZEn = 0;
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            AluOP = `ADD;
         end 
         `slti:begin
            SZEn = 1;
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            AluOP = `SLT;
         end 
         `sltiu:begin
            SZEn = 0;
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            AluOP = `SLTU; 
         end 
         `andi:begin
            SZEn = 0;
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            AluOP = `AND;
         end 
         `ori:begin
            SZEn = 0;
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            AluOP = `OR; 
         end 
         `xori:begin
            SZEn = 0;
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            AluOP = `XOR;
         end 
         `lui: begin
            //SZEn STAY X
            RegWrite = 1;
            RegDst = 0;
            ALUSrc = 1;
            Branch = 0;
            BranchNEQ = 0;
            MemWrite = 0;
            MemtoReg = 0;
            AluOP = `LUI;
         end  

      endcase
      


   end


	// DATA PATH STARTS HERE

   wire [31:0] Imm32 = SZEn ? {{16{instr[15]}},instr[15:0]} : {16'h0, instr[15:0]};     // ZSEn: 1 sign extend, 0 zero extend

   wire [31:0] PCplus4 = PC + 4'h4;

   wire [31:0] PCbranch = PCplus4 + (Imm32 << 2);

   always @(posedge clk)
      if(reset)
         PC <= 32'h0;
      else
         PC <= (PCSrc || PCSrc1) ? PCbranch : PCplus4;


//==========================================================//
//	instantiated modules
//==========================================================//

// Register File

   reg_file rf
   (
      .clk   ( clk ),
      .write ( RegWrite ),
      .WR    ( RegDst   ? instr[15:11] : instr[20:16]),
      .WD    ( MemtoReg ? MemReadData  : AluResult),
      .RR1   ( instr[25:21] ),
      .RR2   ( instr[20:16] ),
      .RD1   ( RD1 ),
      .RD2   ( RD2 )
	);

   my_alu alu
   (
      .Op( AluOP ),
      .A ( RD1 ),
      .B ( ALUSrc ? Imm32 : RD2),
      .X ( AluResult ),
      .Z ( AluZero )
   );
   


//	Instruction Memory
	async_mem imem			// keep the exact instance name
	(
		.clk		   (1'b0),
		.write		(1'b0),		// no write for instruction memory
		.address	   ( PC ),		   // address instruction memory with pc
		.write_data	(32'bx),
		.read_data	( instr )
	);
	
// Data Memory
	async_mem dmem			// keep the exact instance name
	(
		.clk		   ( clk ),
		.write		( MemWrite ),
		.address	   ( AluResult ),
		.write_data	( RD2 ),
		.read_data	( MemReadData )
	);

endmodule


//==============================================================================//
// ------ ALU -----
module my_alu(
   input  [3:0] Op,
   input  [31:0] A,
   input  [31:0] B,
   output [31:0] X,
   output        Z
   );

   wire sub = Op != `ADD;
   wire [31:0] bb = sub ? ~B : B;
   wire [32:0] sum = A + bb + sub;
   wire sltu = ! sum[32];

   wire v = sub ?
            ( A[31] != B[31] && A[31] != sum[31] )
          : ( A[31] == B[31] && A[31] != sum[31] );

   wire slt = v ^ sum[31];

   reg [31:0] x;

   always @( * )
      case( Op )
         `ADD : x = sum;
         `SUB : x = sum;
         `SLT : x = slt;
         `SLTU: x = sltu;
         `AND : x = A & B; //why bitwise and??
         `OR  : x = A | B;
         `NOR : x = ~(A | B);
         `XOR : x = A ^ B;
         `LUI : x = {B[15:0], 16'h0};
         default : x = 32'hxxxxxxxx;
      endcase

   assign X = x;
   assign Z = x == 32'h00000000; // IF X IS EQUALE TO 0 THEN Z IS 1

endmodule

//============================================================================//
