
`timescale 1ns/100ps
module control_unit(clock,reset,mw_opcode,mw_rs,mw_rt,mw_rd,xm_opcode,xm_funct,xm_stall,dx_opcode,dx_funct,fd_opcode,fd_funct,fwd_gpr_rd_data1,fwd_gpr_rd_data2,mw_RegWrite,mw_RegDest,w_MemtoReg,m_MemRead,m_MemWrite,xm_RegDest,dx_RegDest,dx_isJAL,dx_isSLL_SRL,x_ALU_Control,d_PCSrc1,d_PCSrc2,d_PCSrc3,d_isSLL_SRL);
input       clock;
input       reset;
input [5:0] mw_opcode;      //Instr[31:26]
input [4:0] mw_rs;
input [4:0] mw_rt;
input [4:0] mw_rd;
input [5:0] xm_opcode;
input [5:0] xm_funct;       //Defines operation when Instructyion in R type
input       xm_stall;
input [5:0] dx_opcode;
input [5:0] dx_funct;
input [5:0] fd_opcode;
input [5:0] fd_funct;
input [31:0] fwd_gpr_rd_data1;
input [31:0] fwd_gpr_rd_data2;
output reg mw_RegWrite;          // GPR wite Disabled(0)/Enabled(1)
output reg mw_RegDest;           // Write Destination register location
                            // RegDest = 1 -> R-type - Instr[15:11]
                            // RegDest = 0 -> I-type - Instr[21:16]
output w_MemtoReg;          // Steer ALU(0)/Load memory(1) output to GPR write port
output m_MemRead;           // Read from Data memory(LW/LB=1)
output m_MemWrite;          // Write to Data memory(SW/SB=1)
output reg xm_RegDest;
output reg dx_RegDest;
output reg dx_isJAL;
output reg dx_isSLL_SRL;         // Set when Instruction is SLL or SRL
output [2:0] x_ALU_Control; // Defines the ALU operation 
output d_PCSrc1;            // PCSrc1(1)-> Next PC is Jump address
                            // PCSrc1(0)-> Next PC based on PCSrc2
output d_PCSrc2;            // (=d_bcond)When PCSrc1=0-->
                            // if PCSrc2=1-> Next PC based on Immediate address
                            // if PCSrc2=0-> Next PC is PC+4
output d_PCSrc3;            // if PCSrc3=1-> Next PC is GPR[rs]
output d_isSLL_SRL;         // Read address 1 when either SLL or SRL is Inst[20:16](Rt) else Inst[25:21]:(rs) 

parameter TD = 1;

`define ADD  6'b100000
`define AND  6'b100100 
`define JR   6'b001000
`define NOP  6'b000000
`define OR   6'b100101 
`define SLL  6'b000000
`define SLT  6'b101010
`define SRL  6'b000010
`define SUB  6'b100010
`define XOR  6'b100110 

`define ADDI 6'b001000
`define ANDI 6'b001100
`define BEQ  6'b000100
`define BNE  6'b000101
`define LB   6'b100000
`define LW   6'b100011
`define SB   6'b101000
`define SW   6'b101011
`define SLTI 6'b001010 
`define ORI  6'b001101
`define XORI 6'b001110

`define J    6'b000010
`define JAL  6'b000011

reg [2:0] x_alu_ctrl;
wire d_bcond;             // 1 when opcode is BEQ and BNE and branch is taken
wire d_bcond_beq;
wire d_bcond_bne;
reg  xm_isJAL;
wire m_opcode_store;
wire m_opcode_load;
wire m_RegWrite;

assign m_opcode_store = (xm_opcode[5:3]==3'b101);
assign m_opcode_load  = (xm_opcode[5:3]==3'b100);

//-----Memory - Writeback-----

//assign w_MemtoReg  = (mw_opcode==`LW) || (mw_opcode==`LB); //1 - WB from Mem
//                                                           //0 - WB from ALU 
assign w_MemtoReg  = (mw_opcode[5:3]==3'b100);
  //GPR wite Disabled(0)/Enabled(1)
  //Check for STALL-When There is STALL , there is no write back

//************STALL condition added to w_RegWrite****************
//assign w_RegWrite  = ( (mw_opcode!=`SW)    &&  (mw_opcode!=`SB)  ) &&  
//                     ( (mw_opcode!=`BEQ)   &&  (mw_opcode!=`BNE) ) && 
//                     ( (mw_opcode!=`J)     &&  (mw_stall==1'b0)  ) && //--Check for STALL
//                     ( !((mw_opcode==6'd0) &&  (mw_funct==`JR))  );
assign m_RegWrite  = ( m_opcode_store==0 ) &&
                     ( (xm_opcode!=`BEQ)   &&  (xm_opcode!=`BNE) ) && 
                     ( (xm_opcode!=`J)     &&  (xm_stall==1'b0)  ) && //--Check for STALL
                     ( !((xm_opcode==6'd0) &&  (xm_funct==`JR))  );

//-----Execute - Memory-----
//assign m_MemRead   = ((xm_opcode==`LW)  || (xm_opcode==`LB)) && (xm_stall==1'b0);
assign m_MemRead   = (m_opcode_load==1'b1) && (xm_stall==1'b0);

//assign m_MemWrite  = ((xm_opcode==`SW)  || (xm_opcode==`SB)) && (xm_stall==1'b0);
assign m_MemWrite  = (m_opcode_store==1'b1) && (xm_stall==1'b0);


//-----Decode - Execute-----

assign x_ALU_Control = x_alu_ctrl;

//Possibility of removing one bit frommm alu_ctrl
always@(*) begin
  casex({dx_opcode,dx_funct})
    {6'd0 ,`ADD},{`ADDI,6'dx},
    {`LB  ,6'dx},{`LW  ,6'dx},
    {`SB  ,6'dx},{`SW  ,6'dx} : x_alu_ctrl = 3'd0;
    {6'd0 ,`AND},{`ANDI,6'dx} : x_alu_ctrl = 3'd1;
    {6'd0 ,`OR} ,{`ORI ,6'dx} : x_alu_ctrl = 3'd2;
    {6'd0 ,`SLL}              : x_alu_ctrl = 3'd3;
    {6'd0 ,`SLT},{`SLTI,6'dx} : x_alu_ctrl = 3'd4;
    {6'd0 ,`SRL}              : x_alu_ctrl = 3'd5;
    {6'd0 ,`SUB}              : x_alu_ctrl = 3'd6;
    {6'd0 ,`XOR},{`XORI,6'dx} : x_alu_ctrl = 3'd7;
//     {`BEQ ,6'dx}              : alu_ctrl = 4'd8;
//     {`BNE ,6'dx}              : alu_ctrl = 4'd9;
    default                   : x_alu_ctrl = 3'd0;
  endcase
end

//-----Fetch - Decode-----

// -------Checking for BRANCH in DECODE stage itself---------
// Current Implementation is wrong. Instead of register data
// register address is compared
assign d_bcond_beq = (fd_opcode==`BEQ) ? (fwd_gpr_rd_data1==fwd_gpr_rd_data2): 1'b0;
assign d_bcond_bne = (fd_opcode==`BNE) ? (fwd_gpr_rd_data1!=fwd_gpr_rd_data2): 1'b0;
assign d_bcond     = d_bcond_beq || d_bcond_bne;
//-----------------------------------------------------------

assign d_PCSrc1    = (fd_opcode==`J)   || (fd_opcode==`JAL);

// assign d_PCSrc2    = ((fd_opcode==`BEQ) || (fd_opcode==`BNE)) && (d_bcond==1'b1);
assign d_PCSrc2    = d_bcond; 

assign d_PCSrc3    = (fd_opcode==6'd0) && (fd_funct==`JR);

assign d_isSLL_SRL = (fd_opcode==6'b0) && ((fd_funct==`SLL)||(fd_funct==`SRL)); 

always@(posedge clock or posedge reset) begin
  if(reset==1'b1) begin
    dx_isSLL_SRL<= #TD 1'b0;
    dx_RegDest  <= #TD 1'b0;
    dx_isJAL    <= #TD 1'b0;
 
    xm_RegDest  <= #TD 1'b0;
    xm_isJAL    <= #TD 1'b0;
 
    mw_RegDest  <= #TD 1'b0;
    mw_RegWrite <= #TD 1'b0; 
//    mw_isJAL    <= #TD 1'b0;
  end 
  else begin
    dx_isSLL_SRL<= #TD d_isSLL_SRL; 
    dx_RegDest  <= #TD (fd_opcode==6'b0);  //1(R-type) - rd; 0(I-type) - rs
    dx_isJAL    <= #TD (fd_opcode==`JAL);  // On JAL, the writing of PC to GPR[31]
                                           // Added for Forwarding logic


    xm_RegDest  <= #TD dx_RegDest;         //1(R-type) - rd; 0(I-type) - rs
    xm_isJAL    <= #TD (dx_opcode==`JAL);  // On JAL, the writing of PC to GPR[31]
                                           // Added for Forwarding logic


    mw_RegDest  <= #TD xm_RegDest;         //1(R-type) - rd; 0(I-type) - rs
    mw_RegWrite <= #TD m_RegWrite; 
//    mw_isJAL    <= #TD xm_isJAL;           // On JAL, the writing of PC to GPR[31]
//                                           // Added for Forwarding logic
  end
end 

endmodule
