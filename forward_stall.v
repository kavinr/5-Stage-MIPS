
`timescale 1ns/100ps
module forward_stall(
    gpr_wr_addr, //Write back address from Write back state
    xm_gpr_wr_addr,
    mw_opcode,
    xm_opcode,
    xm_rt,
    dx_gpr_rd_addr1,
    dx_rt,
    dx_isSLL_SRL,
    dx_opcode,
    fd_opcode,
    fd_funct,
    fd_rs,
    fd_rt,
    gpr_rd_addr1,
    d_isSLL_SRL,
    d_fwd_rs,
    d_fwd_rt,
    d_stall,
    x_fwd_alu_src1, // 2 bit o/p
    x_fwd_alu_src2 // 2 bit o/p
    
);
input [4:0] gpr_wr_addr;
input [4:0] xm_gpr_wr_addr; //JAL not considered
input [5:0] mw_opcode;
input [5:0] xm_opcode;
input [4:0] xm_rt;
input [4:0] dx_gpr_rd_addr1;
input [4:0] dx_rt;
input       dx_isSLL_SRL;
input [5:0] dx_opcode;
input [5:0] fd_opcode;
input [5:0] fd_funct;
input [4:0] fd_rs;
input [4:0] fd_rt;
input [4:0] gpr_rd_addr1;
input       d_isSLL_SRL;

output reg [1:0] d_fwd_rs;
output reg [1:0] d_fwd_rt;
output           d_stall;
output reg [1:0] x_fwd_alu_src1;
output reg [1:0] x_fwd_alu_src2;

`define BEQ  6'b000100
`define BNE  6'b000101
`define JR   6'b001000
`define J    6'b000010
`define JAL  6'b000011

wire fd_opcode_R;            //R-type instruction
wire fd_Inst_JR;             //Instruction is JR
wire fd_opcode_J_JAL;        //Instructions is either J or JAL
wire fd_opcode_branch;       //Instuction is branch - BEQ or BNE
wire dx_opcode_R;             //Instruction in Execute stage is R-type
wire dx_opcode_load;         //Instruction in Execute stage is Load
wire xm_opcode_load;         //Instruction in Memory stage is Load
wire xm_opcode_load_store;   //Instruction in Memory stage is either Load or Store
wire mw_opcode_store;        //Instruction in Write back  stage is Store

assign fd_opcode_R          = (fd_opcode==6'd0);
assign fd_Inst_JR           = (fd_opcode_R==1'b1) && (fd_funct==`JR); 
assign fd_opcode_J_JAL      = (fd_opcode==`J) || (fd_opcode==`JAL);
assign fd_opcode_branch     = (fd_opcode==`BEQ) || (fd_opcode==`BNE);
assign dx_opcode_load       = (dx_opcode[5:3]==3'b100);
assign dx_opcode_R          = (dx_opcode==6'd0);
assign xm_opcode_load       = (xm_opcode[5:3]==3'b100);
assign xm_opcode_load_store = (xm_opcode[5]==1'b1);
assign mw_opcode_store      = (mw_opcode[5]==1'b1) && (mw_opcode[3]==1'b1);

// Forwarding logic for Top ALU
always@(*) begin
  x_fwd_alu_src1   = 2'b00;                    //Default value
  if(dx_gpr_rd_addr1 != 5'd0) begin            //ALU Source 1 register not equal to R0
    if( (xm_opcode_load_store == 1'b0) &&      //Current instruction in Memory stage is not Load or Store
        (xm_gpr_wr_addr   == dx_gpr_rd_addr1)   //Destination address at Memory stages matches with
      ) begin                                  //Source address for ALU top input     
      x_fwd_alu_src1 = 2'b10;
    end
    else if((mw_opcode_store ==1'b0) &&        //Current instruction in Write-back stage
                                               //is not Store. Load needs forwarding
            (gpr_wr_addr   == dx_gpr_rd_addr1) //Destination address at Write back stages matches with
           ) begin                             //Source address for ALU top input    
      x_fwd_alu_src1 = 2'b11;
    end
  end
end
   
// Forwarding logic for Bottom ALU
always@(*) begin
  x_fwd_alu_src2   = 2'b00;                        //Default value
  if((dx_opcode_R == 1'd1) && (dx_isSLL_SRL==1'b0) //R-type instruction but not SLL and SRL
     && (dx_rt != 5'd0)) begin                     //ALU Source 2 register not equal to R0
    if( (xm_opcode_load_store  == 1'b0) &&   //Current instruction in Memory stage is not Load or Store
        (xm_gpr_wr_addr == dx_rt)             //Destination address at Memory stages matches with
                                             //Source address for ALU bottom input
      ) begin                                
      x_fwd_alu_src2 = 2'b10;
    end
    else if((mw_opcode_store==1'b0) &&       //Current instruction in Write-back stage
                                             //is "not Store". Load needs forwarding
            (gpr_wr_addr   == dx_rt)         //Destination address at Write back stages matches with
           ) begin                           //Source address for ALU bottom input    
      x_fwd_alu_src2 = 2'b11;
    end
  end
end

//Forwarding For Branch(BEQ and BNE) and JR to DECODE stage(Source 1)

always@(*) begin
  d_fwd_rs = 2'b00;                       //Default value
//  if((fd_rs != 5'd0) && ((fd_opcode_branch==1'b1) || //rs not equal to R0 AND
//                         (fd_Inst_JR==1'b1))) begin  //(opcode=BEQ||BNE||JR)
  if(gpr_rd_addr1 != 5'd0) begin 
    if( (xm_opcode_load_store  == 1'b0) &&           //Current instruction in Memory stage is not Load or Store
        (xm_gpr_wr_addr == gpr_rd_addr1)          //Destination address at Memory stages matches with
      ) begin                             //rs in decode stage
      d_fwd_rs = 2'b10;
    end
    else if((mw_opcode_store==1'b0) &&    //Current instruction in Write-back stage
                                          //is not Store. Load needs forwarding
            (gpr_wr_addr == gpr_rd_addr1)        //Destination address at Write back stages matches with
           ) begin                        //rs in decode stage    
      d_fwd_rs = 2'b11;
    end
  end
end

//Forwarding For Branch(BEQ and BNE) to DECODE stage(Source 2)
always@(*) begin
  d_fwd_rt = 2'b00;                //Default value
//  if((fd_rt != 5'd0) && (fd_opcode_branch==1'b1)) begin //rs not equal to R0 AND (opcode=BEQ||BNE)

  if((((fd_opcode_R == 1'd1) && (d_isSLL_SRL==1'b0)) ||  //R-type instruction but not SLL and SRL
      (fd_opcode_branch==1'b1)) &&
      (fd_rt != 5'd0))begin                     //ALU Source 2 register not equal to R0
    if( (xm_opcode_load_store == 1'b0) && //Current instruction in Memory stage is not Load or Store
        (xm_gpr_wr_addr == fd_rt)          //Destination address at Memory stages matches with
                                          //rt in decode stage
      ) begin                             
      d_fwd_rt= 2'b10;
    end
    else if((mw_opcode_store==1'b0) &&    //Current instruction in Write-back stage
                                          //is "not Store". Load needs forwarding
            (gpr_wr_addr   == fd_rt)      //Destination address at Write back stages matches with
           ) begin                        //Source address for ALU top input    
      d_fwd_rt = 2'b11;
    end
  end
end

//Stall at Decode stage
assign d_stall = ( (dx_opcode_load==1'b1) && 
                   (
                    ( (fd_opcode_R==1'b1)&&( (dx_rt==gpr_rd_addr1)||((fd_Inst_JR==1'b0)&&(dx_rt==fd_rt)) ) ) ||
                    ( (fd_opcode_R==1'b0)&&(fd_opcode_J_JAL==1'b0)&&(dx_rt==fd_rs) )  ||
                    ( (fd_opcode_branch==1'b1)&&(dx_rt==fd_rt))
                   )
                 ) ||
                 ( (xm_opcode_load==1'b1) &&
                   ( 
                     (((fd_opcode_branch==1'b1) || (fd_Inst_JR==1'b1)) && (xm_rt==fd_rs)) ||
                     ((fd_opcode_branch==1'b1) && (xm_rt==fd_rt))
                   )
                 );
endmodule
