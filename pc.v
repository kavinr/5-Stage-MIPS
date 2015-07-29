
`timescale 1ns/100ps
module pc(clk, rst, fd_br_signext_sl2, fd_Inst_25_0, fwd_gpr_rd_data1, d_stall, jump, branch, jump_reg, pc,d_pc_plus_8);

input clk, rst;
input [31:0] fd_br_signext_sl2;
input [25:0] fd_Inst_25_0;
input [31:0] fwd_gpr_rd_data1;
input d_stall;
input jump;       //J or JAL
input branch;     //Branch - BEQ or BNE
input jump_reg;   //JR
output [31:0] pc;
output [31:0] d_pc_plus_8;

parameter TD = 1;

reg  [31:0] pc_val;
wire [31:0] br_loc, d_pc_plus_4,pc_plus_4;

assign pc_plus_4 = pc_val + 32'd4;                  // Next PC (PC+4) when the instruction  is not brach
assign d_pc_plus_4 = pc_val;                        // Since PC is used in the decode stage, 
                                                    // it is actually PC+4 for the instruction in the decode stage 
assign d_pc_plus_8 = pc_val + 32'd4;                // To calculate PC+8 to be written to GPR[31] 
                                                    // on JAL instruction
assign br_loc    = d_pc_plus_4 + fd_br_signext_sl2; // To calculate branch target


always @ (posedge clk or posedge rst)               //PCSrc Mux
begin
  if (rst==1'b1) begin
    pc_val <= #TD 31'd0;
  end
  else if(d_stall==1'b1) begin
   pc_val  <= #TD pc_val;
  end 
  else if (jump==1'b1) begin  //jump --> J and JAL
    pc_val <= #TD {d_pc_plus_4[31:28],fd_Inst_25_0,2'b00};
  end 
  else if (branch==1'b1) begin
    pc_val <= #TD br_loc;
  end
  else if(jump_reg==1'b1) begin
    pc_val <= #TD fwd_gpr_rd_data1;
  end 
  else begin
    pc_val <= #TD pc_plus_4;
  end
end


assign pc = pc_val;

endmodule
