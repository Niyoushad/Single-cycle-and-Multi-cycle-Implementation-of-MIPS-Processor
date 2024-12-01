//j problem version 7:09 PM
`timescale 1ns/100ps

   `define ADD  3'b000
   `define SUB  3'b001
   `define SLT  3'b010
   `define SLTU 3'b011
   `define AND  3'b100
   `define XOR  3'b101
   `define OR   3'b110
   `define NOR  3'b111

module multi_cycle_mips(

   input clk,
   input reset,

   // Memory Ports
   output  [31:0] mem_addr,
   input   [31:0] mem_read_data,
   output  [31:0] mem_write_data,
   output         mem_read,
   output         mem_write
);

   // Data Path Registers
   reg MRE, MWE;
   reg [31:0] A, B, PC, HI, LO, IR, MDR, MAR;

   // Data Path Control Lines, don't forget, regs are not always register !!
   reg setMRE, clrMRE, setMWE, clrMWE;
   reg Awrt, Bwrt, RFwrt, PCwrt, HIwrt, LOwrt, IRwrt, MDRwrt, MARwrt;
 
 //fojewoifheoihvwe
 reg counter;

   // Multiplier wires and reg:
    wire [31:0] Multiplier = B;
    wire [31:0] Multiplicand = A;
    wire [63:0] MultResult;
    reg start;
    wire ready;

   // Memory Ports Bindings
   assign mem_addr = MAR;
   assign mem_read = MRE;
   assign mem_write = MWE;
   assign mem_write_data = B;

   // Mux & ALU Control Line
   reg [2:0] aluSelB, aluOp, MemtoReg;
   reg [1:0] RegDst, PCSrc;
   reg SgnExt, aluSelA, IorD;


   // Wiring
   wire aluZero;
   wire [31:0] aluResult, rfRD1, rfRD2;
   // JumpPC Address Making
   wire [31:0] PCJump = {PC[31:28], IR[25:0], 2'b00};


   // PC Mux with PCSrc enable
   reg [31:0] local_PC;
   always @(*)
   case (PCSrc)
      2'b00: local_PC = aluResult;
      2'b01: local_PC = A; // محتوای وجیستر اول در پی سی مینشیند
      2'b10: ;
      2'b11: local_PC = PCJump;
   endcase

   // Clocked Registers
   always @( posedge clk ) begin // مقداردهی بر حسب کنترلی
      if( reset )
         PC <= #0.1 32'h00000000;
      else if( PCwrt )
         PC <= #0.1 local_PC; // I ADDED 

      //----------------- I ADDED --------------:
      
      if( reset )
         HI <= #0.1 'bx;
      else if( HIwrt )
         HI <= #0.1 MultResult[63:32];

      //----------------- I ADDED --------------:
      if( reset )
         LO <= #0.1 'bx;
      else if( LOwrt )
         LO <= #0.1 MultResult[31:0]; 

      

      if( Awrt ) A <= #0.1 rfRD1;
      if( Bwrt ) B <= #0.1 rfRD2;

      if( MARwrt ) MAR <= #0.1 IorD ? aluResult : PC;

      if( IRwrt ) IR <= #0.1 mem_read_data;
      if( MDRwrt ) MDR <= #0.1 mem_read_data;

      if( reset | clrMRE ) MRE <= #0.1 1'b0;
          else if( setMRE) MRE <= #0.1 1'b1;

      if( reset | clrMWE ) MWE <= #0.1 1'b0;
          else if( setMWE) MWE <= #0.1 1'b1;
   end


   // Register File
   reg_file rf(
      .clk( clk ),
      .write( RFwrt ),

      .RR1( IR[25:21] ),
      .RR2( IR[20:16] ),
      .RD1( rfRD1 ),
      .RD2( rfRD2 ),

      .WR( (RegDst == 2'b11) ? IR[15:11] :
           (RegDst == 2'b00) ? IR[20:16] :
           (RegDst == 2'b01) ? 5'b11111 : PC), 
      .WD( (MemtoReg == 3'b111) ? MDR :
           (MemtoReg == 3'b000) ? aluResult :
           (MemtoReg == 3'b001) ? {IR[15:0], 16'h0000} :
           (MemtoReg == 3'b010) ? HI:
           (MemtoReg == 3'b011) ? LO: 'bx)

   );
   

   // Sign/Zero Extension
   wire [31:0] SZout = SgnExt ? {{16{IR[15]}}, IR[15:0]} : {16'h0000, IR[15:0]};

   // ALU-A Mux
   wire [31:0] aluA = aluSelA ? A : PC;

   // ALU-B Mux
   reg [31:0] aluB;
   always @(*)
   case (aluSelB)
      3'b000: aluB = B;
      3'b001: aluB = 32'h4;
      3'b010: aluB = SZout;
      3'b011: aluB = SZout << 2;
      3'b100: aluB = 32'h0000_0000;
      3'b101,3'b110,3'b111:;
   endcase

 


   my_alu alu(
      .A( aluA ),
      .B( aluB ),
      .Op( aluOp ),

      .X( aluResult ),
      .Z( aluZero )
   );


   my_multiplier multiplier (
      .clk( clk ),
      .start(start), 
      .A( Multiplicand ),
      .B( Multiplier ),
      .Product( MultResult ),
      .ready(ready)
   );

   // Controller Starts Here

   // Controller State Registers
   reg [4:0] state, nxt_state;

   // State Names & Numbers
   localparam
      RESET = 0,      FETCH1 = 1,     FETCH2 = 2,   FETCH3 = 3,   DECODE = 4,
      EX_ALU_R = 7,   EX_ALU_I = 8,
      EX_LW_1 = 11,   EX_LW_2 = 12,   EX_LW_3 = 13, EX_LW_4 = 14, EX_LW_5 = 15,
      EX_SW_1 = 21,   EX_SW_2 = 22,   EX_SW_3 = 23,
      EX_BRA_1 = 25,  EX_BRA_2 = 26,
      EX_J = 27,      EX_JAL = 28,  EX_JALR = 29,
      EX_MULT = 30, 
      EX_LUI_2 = 31,
      EX_LUI_3 = 32; 

   // State Clocked Register 
   always @(posedge clk)
      if(reset)
         state <= #0.1 RESET;
      else
         state <= #0.1 nxt_state;

   task PrepareFetch;
      begin
         IorD = 0;
         setMRE = 1;
         MARwrt = 1;
         nxt_state = FETCH1;
      end
   endtask

   // State Machine Body Starts Here
   always @( * ) begin

      nxt_state = 'bx;

      SgnExt = 'bx;   IorD = 'bx;
      MemtoReg = 'bx; RegDst = 'bx;
      aluSelA = 'bx;  aluSelB = 'bx; aluOp = 'bx;
      PCSrc = 'bx;

      PCwrt = 0;
      HIwrt = 0;  LOwrt = 0;
      Awrt = 0;   Bwrt = 0;
      RFwrt = 0;  IRwrt = 0;
      MDRwrt = 0; MARwrt = 0;
      setMRE = 0; clrMRE = 0;
      setMWE = 0; clrMWE = 0;
      start = 'bx;

      case(state)

         RESET:
            PrepareFetch;

         FETCH1:
            nxt_state = FETCH2;

         FETCH2:
            nxt_state = FETCH3;

         FETCH3: begin
           
            clrMRE = 1; 
            PCwrt = 1;
            PCSrc = 2'b00; 
            IRwrt = 1;
            aluSelA = 0;
            aluSelB = 3'b001;
            aluOp = `ADD; // PC + 4
            
            nxt_state = DECODE; 
         end

         DECODE: begin
            Awrt = 1;
            Bwrt = 1;
            aluSelA = 0;
            aluSelB = 3'b011; 
            aluOp = `ADD; 

            case( IR[31:26] )
               6'b000_000:             // R-format
                  case( IR[5:3] )
                     3'b000: ; 
                     3'b001: nxt_state = EX_ALU_R; 
                     3'b010: nxt_state = EX_ALU_R;
                     3'b011: nxt_state = EX_ALU_R; //multu
                     3'b100: nxt_state = EX_ALU_R;
                     3'b101: nxt_state = EX_ALU_R;
                     3'b110: ;
                     3'b111: ;
                  endcase

               6'b001_000,           // addi
               6'b001_001,           // addiu
               6'b001_010,           // slti
               6'b001_011,           // sltiu
               6'b001_100,           // andi
               6'b001_101,           // ori
               6'b001_110,           // xori
               6'b001_111:           // lui
                  nxt_state = EX_ALU_I;

               6'b100_011:
                  nxt_state = EX_LW_1;

               6'b101_011:
                  nxt_state = EX_SW_1;

               6'b000_100,
               6'b000_101:
                  nxt_state = EX_BRA_1;
                  
               6'b000_010:
                  nxt_state = EX_J;
               
               6'b000_011:
                  nxt_state = EX_JAL;
                  
                  

               // rest of instructiones should be decoded here

            endcase
         end

         EX_ALU_R: begin
            // not bolde instructions:
            case( IR[5:3] )
            3'b100,
            3'b101: begin
               aluSelA = 1; // A is value of register rs in the instruction
               aluSelB = 3'b000;
               RegDst = 2'b11;
               MemtoReg = 3'b000;
               RFwrt = 1;
               case( IR[5:0] )
                     6'b100000: aluOp = `ADD; //add
                     6'b100001: aluOp = `ADD; //addu
                     6'b100010: aluOp = `SUB; //sub
                     6'b100011: aluOp = `SUB; //subu
                     6'b100100: aluOp = `AND; //and
                     6'b100101: aluOp = `OR ; //or
                     6'b100110: aluOp = `XOR; //xor
                     6'b100111: aluOp = `NOR; //nor
                     6'b101010: aluOp = `SLT; //slt
                     6'b101011: aluOp = `SLTU; //sltu
               endcase
               PrepareFetch;
            end
            endcase

            // BOlde instructions:
            case( IR[5:0] ) 
                     6'b001000: begin //jr

                        aluSelA = 1; 
                        aluSelB = 3'b000;
                        aluOp = `ADD;
                        PCSrc = 2'b00;
                        PCwrt = 1;
                        IorD = 0;

                        nxt_state = RESET;
                        
                     end

                     6'b001001: begin //jalr
                        // PC being written in register number 31 :
                        aluOp = `ADD;
                        aluSelA = 0; 
                        aluSelB = 3'b100; 
                        MemtoReg = 3'b000; 
                        RFwrt = 1;
                        RegDst = 2'b01;

                        nxt_state = EX_JALR;
                     end

                      6'b010000: begin //mfhi
                        RegDst = 2'b11;
                        MemtoReg = 3'b010;
                        RFwrt = 1;
                        PrepareFetch;
                     end

                      6'b010010: begin //mflo
                        RegDst = 2'b11;
                        MemtoReg = 3'b011;
                        RFwrt = 1;
                        PrepareFetch;
                     end

                      6'b011001: begin //multu
                        start = 1;
                        nxt_state = EX_MULT;
                     end 
            endcase
         end



         EX_ALU_I: begin
            aluSelA = 1;
            aluSelB = 3'b010;
            aluOp = `ADD;
            RegDst = 2'b00;
            MemtoReg = 2'b00;
            RFwrt = 1;
            case( IR[31:26])
               6'b001_000: begin
                aluOp = `ADD;             // addi
                SgnExt = 1;
               end 

               6'b001_001:begin  // addiu
                aluOp = `ADD;            
                SgnExt = 1;                 
               end
               6'b001_010:begin  // slti
                aluOp = `SLT;            
                SgnExt = 1; 
               end
               6'b001_011:begin  // sltiu
                aluOp = `SLTU;            
                SgnExt = 1;
               end
              
               6'b001_100: begin // andi 
                aluOp = `AND; 
                SgnExt = 0;
               end       

               6'b001_101: begin // ori 
                aluOp = `OR;   
                SgnExt = 0;
               end     

               6'b001_110: begin //xori
                aluOp = `XOR;   
                SgnExt = 0;
               end  

               6'b001_111: begin //lui
                RegDst = 2'b00;
                RFwrt = 1;
                MemtoReg = 2'b01;
               end
            endcase   
            PrepareFetch;  
         end

         EX_LW_1: begin
            aluSelA = 1;
            aluSelB = 3'b010;
            aluOp = `ADD;
            SgnExt = 1;
            IorD = 1;
            MARwrt = 1;
            setMRE = 1;
            nxt_state = EX_LW_2;
         end

         EX_LW_2: begin
            nxt_state = EX_LW_3;
         end

         EX_LW_3: begin
            nxt_state = EX_LW_4;
         end

         EX_LW_4: begin
            clrMRE = 1;
            MDRwrt = 1;
            nxt_state = EX_LW_5;
         end

         EX_LW_5: begin
            RegDst = 2'b00;
            MemtoReg = 3'b111;
            RFwrt = 1;
            PrepareFetch;
         end

         EX_SW_1: begin
            aluSelA = 1;
            aluSelB = 3'b010;
            IorD = 1;
            aluOp = `ADD;
            SgnExt = 1;
            MARwrt = 1;
            setMWE = 1;
            nxt_state = EX_SW_2;

         end

         EX_SW_2: begin
            clrMWE = 1;
            nxt_state = EX_SW_3;
         end

         EX_SW_3: begin
            PrepareFetch;
         end

         EX_BRA_1: begin
            aluSelA = 1;
            aluSelB = 3'b000;
            aluOp = `SUB;
            if (IR[31:26] == 6'b000_100) begin//BEQ
               if (aluZero) // line 8 branch
                  nxt_state = EX_BRA_2;
               else 
                  PrepareFetch;
            end
            else if (IR[31:26] == 6'b000_101)begin //BENQ
               if (~aluZero) 
                  nxt_state = EX_BRA_2;
               else
                  PrepareFetch;
            end 
                     
         end

         EX_BRA_2: begin
           aluSelA = 0; //A is PC
           aluSelB = 3'b011; // B is sgn extended 2 shifted
           IorD = 1;
           PCwrt = 1;
           PCSrc = 2'b00; // PC comes from ALU
           MARwrt = 1;
           setMRE = 1;
           aluOp = `ADD; 
           SgnExt = 1;
           nxt_state = FETCH1;
         end

         EX_J: begin
           //JUMPING
           PCwrt = 1;
           PCSrc = 2'b11;
         
           nxt_state = RESET;
         end

         EX_JAL: begin
           // PC being written in register number 31 :
           aluOp = `ADD;
           aluSelA = 0; //PC
           aluSelB = 3'b100; //0
           MemtoReg = 3'b000;
           RFwrt = 1;
           RegDst = 2'b01;
           nxt_state = EX_J;
         end

         EX_JALR: begin //29
            PCwrt = 1;
            PCSrc = 2'b01;
            
            nxt_state = RESET;
         end

         EX_MULT: begin //30
            if (!ready)
               nxt_state = EX_MULT;
            else begin
            HIwrt = 1;
            LOwrt = 1;
            PrepareFetch;
            end
         end
         

      

      endcase

   end

endmodule

//==============================================================================

module my_alu(
   input [2:0] Op,
   input [31:0] A,
   input [31:0] B,

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
         `AND : x =   A & B;
         `OR  : x =   A | B;
         `NOR : x = ~(A | B);
         `XOR : x =   A ^ B;
         default : x = 32'hxxxxxxxx;
      endcase

   assign #2 X = x;
   assign #2 Z = x == 32'h00000000;

endmodule

//==============================================================================

module reg_file(
   input clk,
   input write,
   input [4:0] WR,
   input [31:0] WD,
   input [4:0] RR1,
   input [4:0] RR2,
   output [31:0] RD1,
   output [31:0] RD2
);

   reg [31:0] rf_data [0:31];

   assign #2 RD1 = rf_data[ RR1 ];
   assign #2 RD2 = rf_data[ RR2 ];   

   always @( posedge clk ) begin
      if ( write )
         rf_data[ WR ] <= WD;

      rf_data[0] <= 32'h00000000;
   end

endmodule

//==============================================================================

module my_multiplier(
//---------------------Port directions and deceleration
   input clk,  
   input start,
   input [31:0] A, 
   input [31:0] B, 
   output reg [63:0] Product,
   output ready
    );



//----------------------------------------------------

//--------------------------------- register deceleration
reg [31:0]  Multiplicand ; // A
reg [31:0]  Multiplier; // ???? here is B
reg [5:0]  counter;
//-----------------------------------------------------

//----------------------------------- wire deceleration
wire product_write_enable;
wire [32:0] adder_output; // 8 bit + 1 bit more for cout.
wire [31:0] mux_output;
//-------------------------------------------------------

//------------------------------------ combinational logic
assign mux_output = product_write_enable ? Multiplicand:32'b0;
assign adder_output = mux_output + Product[63:32];

assign product_write_enable = Product [0];
assign ready = counter[5];
//-------------------------------------------------------

//------------------------------------- sequential Logic
always @ (posedge clk)

    if(start) begin
        counter <= 6'h0 ;
        Multiplier <= B; 
        Product <= {32'h00, B};
        Multiplicand <= A ;
    end

    else if(! ready) begin
        counter <= counter + 1;
        Product <= Product >> 1;
        //Multiplier <= Multiplier >> 1;
        //Multiplicand <= Multiplicand << 1;

      if(product_write_enable)
        Product [63:31] <= adder_output; // 9 bit of aproduct msb will be adder output
   end   

endmodule

