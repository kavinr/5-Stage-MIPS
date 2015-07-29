
`timescale 1ns/100ps
module processor (clock, reset, PC, Inst, MemRead, MemWrite, Addr, Din, Dout);

input clock, reset;
input [31:0] Inst, Dout;
output MemRead, MemWrite;
output [31:0] PC, Addr, Din;

parameter TD = 1;
//Register Declarations

//Wire Declarations
wire [31:0] x_gpr_rd_data2_I;
wire [31:0] x_gpr_rd_data2_I_shamt;
wire [31:0] x_STORE_write_data;
wire [31:0] fwd_data_ALU_src1; 
wire [31:0] fwd_data_ALU_src2; 
reg [4:0] gpr_wr_addr;
wire [4:0] gpr_rd_addr1;
wire [4:0] gpr_rd_addr2;
wire [31:0] gpr_wr_data;
wire [31:0] gpr_rd_data1;
wire [31:0] gpr_rd_data2;
wire [31:0] fwd_gpr_rd_data1;
wire [31:0] fwd_gpr_rd_data2;
wire [31:0] fwd_data_DECODE_src1;
wire [31:0] fwd_data_DECODE_src2;
wire [31:0] mem_alu_data_out;

wire [31:0] pc;
wire [31:0] Inst_stall;   //On Stall, Inst_stall will be fd_Inst- to retain
                          //the value of fd_Inst on stall
wire [31:0] Inst_stall_b_j; //To fill branch delay slot with NOPs of J,JAL,BEQ,BNE,JR 
wire [15:0] fd_Inst_15_0;
wire [25:0] fd_Inst_25_0;
wire [4:0]  fd_rs;
wire [4:0]  fd_rt;
wire [1:0]  d_fwd_rs;
wire [1:0]  d_fwd_rt;
wire        d_stall;
wire [5:0]  fd_opcode;
wire [5:0]  fd_funct;
reg  [31:0] fd_Inst; 
//wire [31:0] fd_Inst_stall; 
wire [31:0] d_Inst_15_0_signext;
wire [31:0] d_br_signext_sl2;
wire d_PCSrc1; 
wire d_PCSrc2; 
wire d_PCSrc3;
wire d_isSLL_SRL;
wire [31:0] d_pc_plus_8;        //PC + 8 to be written to GPR[31] on JAL
reg [31:0] dx_pc_plus_8;
reg [31:0] dx_Inst;
reg [31:0] dx_Inst_15_0_signext;
reg [31:0] dx_gpr_rd_data2; 
reg [31:0] dx_shamt;
reg [4:0] dx_gpr_rd_addr1;
reg [4:0] dx_gpr_rd_addr2;
reg       dx_stall;
reg [31:0] dx_gpr_rd_data1;
wire [5:0] dx_opcode;
wire [5:0] dx_funct;
wire [4:0] dx_rs;
wire [4:0] dx_rt;
wire [4:0] dx_rd;
wire [31:0] ALU_datain1;
wire [31:0] ALU_datain2;
wire [1:0] x_fwd_alu_src1;
wire [1:0] x_fwd_alu_src2;
wire       dx_isSLL_SRL;
wire       dx_isJAL;
wire       dx_RegDest;
wire [2:0] x_ALU_Control;
wire [31:0] x_ALU_Result;
wire [4:0] x_gpr_wr_addr0;
wire [4:0] x_gpr_wr_addr;
reg  [4:0]  xm_gpr_wr_addr;
reg  [31:0] xm_ALU_Result;
reg  [31:0] xm_STORE_write_data;
reg         xm_stall;
reg [31:0] xm_Inst;
reg [31:0] xm_pc_plus_8;
wire [5:0] xm_opcode;
//reg [4:0]  xm_gpr_rd_addr1;
//reg [4:0]  xm_gpr_rd_addr2;
wire [4:0] xm_rt;
wire       m_MemRead;
wire       m_MemWrite;
wire       xm_RegDest;
reg [31:0] mw_Inst;
reg [31:0] mw_pc_plus_8;
reg [31:0] mw_Dout;
reg [31:0] mw_ALU_Result;
reg        mw_stall; 
wire [5:0] mw_opcode; 
wire [5:0] xm_funct;
wire [4:0] mw_rs;
wire [4:0] mw_rt;
wire [4:0] mw_rd;
wire       mw_RegWrite;
wire       mw_RegDest;
wire       w_MemtoReg;
wire       mw_is_JAL;
 
assign Inst_stall     = (d_stall==1'b1) ? fd_Inst : Inst; 

assign Inst_stall_b_j = ((d_PCSrc1==1'b1) || //J and JAL
                         (d_PCSrc2==1'b1) || //Branch - BEQ or BNE
                         (d_PCSrc3==1'b1)) ? 32'd0 : //Branch delay Slot - NOP
                                             Inst_stall;

//assign fd_Inst_stall = (d_stall==1'b1) ? 32'd0  : fd_Inst;//To pass NOP to subsequent stages

always@(posedge clock or posedge reset) begin
  if(reset==1'b1) begin
    //-----Fetch - Decode-----
     fd_Inst <= #TD 32'd0;
    
    //-----Decode - Execute-----
     dx_Inst              <= #TD 32'd0;
     dx_pc_plus_8         <= #TD 32'd0; //for JAL
     dx_Inst_15_0_signext <= #TD 32'd0;
     dx_gpr_rd_data2      <= #TD 32'd0; 
     dx_shamt             <= #TD 32'd0;
     dx_gpr_rd_data1      <= #TD 32'd0;
     dx_gpr_rd_addr1      <= #TD 5'd0; 
     dx_gpr_rd_addr2      <= #TD 5'd0;
     dx_stall             <= #TD 1'b0; 
    
    //-----Execute - Memory-----
     xm_Inst              <= #TD 32'd0;
     xm_pc_plus_8         <= #TD 32'd0; //for JAL
     xm_ALU_Result        <= #TD 32'd0; // To be used as address in the Memory stage
     xm_STORE_write_data  <= #TD 32'd0;
     xm_stall             <= #TD 1'b0;
     //xm_gpr_rd_addr1      <= #TD 5'd0; 
     //xm_gpr_rd_addr2      <= #TD 5'd0; 
    
    //-----Memory - Writeback-----
     mw_Inst              <= #TD 32'd0;
     mw_pc_plus_8         <= #TD 32'd0; //for JAL
     mw_Dout              <= #TD 32'd0;
     mw_ALU_Result        <= #TD 32'd0;
     mw_stall             <= #TD 1'b0;
      
   end
   else begin
     fd_Inst              <= #TD Inst_stall_b_j;
     dx_Inst              <= #TD fd_Inst; //Passes NOP on stall
     dx_pc_plus_8         <= #TD d_pc_plus_8;  
     dx_Inst_15_0_signext <= #TD d_Inst_15_0_signext;
     dx_gpr_rd_data2      <= #TD fwd_gpr_rd_data2;
     dx_shamt             <= #TD {27'd0,fd_Inst[10:6]};
     dx_gpr_rd_data1      <= #TD fwd_gpr_rd_data1;
     dx_gpr_rd_addr1      <= #TD gpr_rd_addr1; 
     dx_gpr_rd_addr2      <= #TD gpr_rd_addr2; //fd_rt
     dx_stall             <= #TD d_stall;
     xm_Inst              <= #TD dx_Inst;
     xm_pc_plus_8         <= #TD dx_pc_plus_8;
     xm_ALU_Result        <= #TD x_ALU_Result;
     xm_STORE_write_data  <= #TD dx_gpr_rd_data2;
     xm_stall             <= #TD dx_stall;
     mw_Inst              <= #TD xm_Inst;
     mw_pc_plus_8         <= #TD xm_pc_plus_8;
     mw_Dout              <= #TD Dout;
     mw_ALU_Result        <= #TD xm_ALU_Result;
     mw_stall             <= #TD xm_stall;
     //xm_gpr_rd_addr1      <= #TD dx_gpr_rd_addr1; 
     //xm_gpr_rd_addr2      <= #TD dx_gpr_rd_addr2;
   end
end

assign fd_Inst_15_0 = fd_Inst[15:0];
assign fd_Inst_25_0 = fd_Inst[25:0]; 
 
//Sign extension for Instruction[15:0] for
//*branch address calculation
//*Alu source
signext signext_u0(.ip(fd_Inst_15_0        ),
                   .op(d_Inst_15_0_signext)
                   );
//assign d_Inst_15_0_signext = {{16{fd_Inst_15_0[15]}},fd_Inst_15_0};

//Left shift for branch address calculation
lshift lshift_u0(.ip(d_Inst_15_0_signext),
                 .op(d_br_signext_sl2   )
                );

//Forwarding to Decode Stage for BEQ,BNE and JR
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_FWD_D1 (
                      .ip1(mem_alu_data_out), 
                      .ip0(xm_ALU_Result), 
                      .sel(d_fwd_rs[0]), 
                      .out(fwd_data_DECODE_src1)
                     );

mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_FWD_D2 (
                      .ip1(fwd_data_DECODE_src1), 
                      .ip0(gpr_rd_data1), 
                      .sel(d_fwd_rs[1]), 
                      .out(fwd_gpr_rd_data1)
                     );

mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_FWD_D3 (
                      .ip1(mem_alu_data_out), 
                      .ip0(xm_ALU_Result), 
                      .sel(d_fwd_rt[0]), 
                      .out(fwd_data_DECODE_src2)
                     );

mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_FWD_D4 (
                      .ip1(fwd_data_DECODE_src2), 
                      .ip0(gpr_rd_data2), 
                      .sel(d_fwd_rt[1]), 
                      .out(fwd_gpr_rd_data2)
                     );

//Program Counter Module with PCSrc Mux
pc pc_u0 (.clk               (clock           ),
          .rst               (reset           ), 
          .fd_br_signext_sl2 (d_br_signext_sl2), 
          .fd_Inst_25_0      (fd_Inst_25_0    ),
          .fwd_gpr_rd_data1  (fwd_gpr_rd_data1),
          .d_stall           (d_stall         ), 
          .jump              (d_PCSrc1        ), //J or JAL           
          .branch            (d_PCSrc2        ), //(=d_bcond)Branch- BEQ or BNE
          .jump_reg          (d_PCSrc3        ), //JR
          .pc                (pc              ),
          .d_pc_plus_8       (d_pc_plus_8     )
         );

// Control Unit
control_unit control_unit_u0(
                             .clock            (clock            ),
                             .reset            (reset            ),
                             .mw_opcode        (mw_opcode        ),
                             .mw_rs            (mw_rs            ),
                             .mw_rt            (mw_rt            ),
                             .mw_rd            (mw_rd            ),
                             .xm_opcode        (xm_opcode        ),
                             .xm_funct         (xm_funct         ),
                             .xm_stall         (xm_stall         ),
                             .dx_opcode        (dx_opcode        ),
                             .dx_funct         (dx_funct         ),
                             .fd_opcode        (fd_opcode        ),
                             .fd_funct         (fd_funct         ),
                             .fwd_gpr_rd_data1 (fwd_gpr_rd_data1 ),
                             .fwd_gpr_rd_data2 (fwd_gpr_rd_data2 ),
                             .mw_RegWrite      (mw_RegWrite      ),
                             .mw_RegDest       (mw_RegDest       ),
                             .w_MemtoReg       (w_MemtoReg       ),
                             .m_MemRead        (m_MemRead        ),
                             .m_MemWrite       (m_MemWrite       ),
                             .xm_RegDest       (xm_RegDest       ),
                             .dx_RegDest       (dx_RegDest       ),
                             .dx_isJAL         (dx_isJAL         ),
                             .dx_isSLL_SRL     (dx_isSLL_SRL     ),
                             .x_ALU_Control    (x_ALU_Control    ),
                             .d_PCSrc1         (d_PCSrc1         ), //J or JAL
                             .d_PCSrc2         (d_PCSrc2         ), //(=d_bcond)Branch- BEQ or BNE
                             .d_PCSrc3         (d_PCSrc3         ), //JR
                             .d_isSLL_SRL      (d_isSLL_SRL      )
                            );
assign mw_opcode = mw_Inst[31:26];
assign xm_funct  = xm_Inst[5:0];
assign mw_rs     = mw_Inst[25:21];
assign mw_rt     = mw_Inst[20:16];
assign mw_rd     = mw_Inst[15:11];
assign xm_opcode = xm_Inst[31:26];
assign xm_rt     = xm_Inst[20:16];
assign dx_opcode = dx_Inst[31:26];
assign dx_funct  = dx_Inst[5:0];
assign dx_rs     = dx_Inst[25:21];
assign dx_rt     = dx_Inst[20:16];
assign dx_rd     = dx_Inst[15:11];
assign fd_opcode = fd_Inst[31:26];
assign fd_funct  = fd_Inst[5:0];
assign fd_rs     = fd_Inst[25:21];
assign fd_rt     = fd_Inst[20:16];


                          
//Added for Forwarding block - Condition of JAL is not addded here.
//JAL check will be added in forwarding block
mux_2x1 #(.DATA_WIDTH(5)
         ) mux_2x1_u2(.ip1(dx_rd         ), //When Instruction is R-type
                      .ip0(dx_rt         ), //When Instruction is I-type 
		      .sel(dx_RegDest     ), //1-Inst[15:11];0-Inst[20:16]
                      .out(x_gpr_wr_addr0)
                     );

mux_2x1 #(.DATA_WIDTH(5)
         ) mux_2x1_u3(.ip1(5'd31       ), //When JAL write PC+8 to gpr[31] 
                      .ip0(x_gpr_wr_addr0), 
                      .sel(dx_isJAL     ),
                      .out(x_gpr_wr_addr )
                     );

//Multiplexer to select write back to GPR from ALU or MEM
//Another multiplexer selects this data and JAL address(PC+8)
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u4(.ip1(mw_Dout         ), // Dout -> read data from Data memory
                     .ip0(mw_ALU_Result   ),
                     .sel(w_MemtoReg      ),
                     .out(mem_alu_data_out)
                    );

mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u5(.ip1(mw_pc_plus_8    ),
                     .ip0(mem_alu_data_out),
                     .sel(mw_is_JAL       ),
                     .out(gpr_wr_data     )
                    );
//Read register 1 address for SLL and SRL alone is to be taken from
//Inst[20:16](Rt); others from Inst[25:21] (Rs)
mux_2x1 #(.DATA_WIDTH(5)
         )mux_2x1_u6(.ip1(fd_rt),
                     .ip0(fd_rs),
                     .sel(d_isSLL_SRL),
                     .out(gpr_rd_addr1)
                    );

assign gpr_rd_addr2 = fd_rt;

gpr gpr_u0(
           .clk     (clock       ),
           .RegWrite(mw_RegWrite ),
           .rd_addr1(gpr_rd_addr1),
           .rd_addr2(fd_rt       ),
           .wr_addr (gpr_wr_addr ),
           .wr_data (gpr_wr_data ),
           .rd_data1(gpr_rd_data1),
           .rd_data2(gpr_rd_data2)
          );

//ALU operand source Mux
//Need an additional mux to input SHAMT(shift amount) for SLL and SRL
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u7(
                     .ip1(dx_gpr_rd_data2     ), 
                     .ip0(dx_Inst_15_0_signext), 
                     .sel(dx_RegDest          ), //NM - Branch need not be considered
                     .out(x_gpr_rd_data2_I    )
                    );
//Mux used to take case of SLL and SRL instructions
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u8 (
                      .ip1(dx_shamt         ), 
                      .ip0(x_gpr_rd_data2_I ), 
                      .sel(dx_isSLL_SRL      ), 
                      .out(x_gpr_rd_data2_I_shamt )
                     );
//Mux to select FWD data for ALU SRC1
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u9 (
                      .ip1(mem_alu_data_out), 
                      .ip0(xm_ALU_Result), 
                      .sel(x_fwd_alu_src1[0]), 
                      .out(fwd_data_ALU_src1)
                     );
//Mux to select FWD data for ALU SRC2
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u10 (
                      .ip1(mem_alu_data_out), 
                      .ip0(xm_ALU_Result), 
                      .sel(x_fwd_alu_src2[0]), 
                      .out(fwd_data_ALU_src2)
                     );
//Mux to select  data for ALU SRC1
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u11 (
                      .ip1(fwd_data_ALU_src1), 
                      .ip0(dx_gpr_rd_data1), 
                      .sel(x_fwd_alu_src1[1]), 
                      .out(ALU_datain1)
                     );
//Mux to select  data for ALU SRC2
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u12 (
                      .ip1(fwd_data_ALU_src2), 
                      .ip0(x_gpr_rd_data2_I_shamt), 
                      .sel(x_fwd_alu_src2[1]), 
                      .out(ALU_datain2)
                     );
//Mux to select between GPR read data or Forward data for Store command
//Storing the content in GPR location specified by rt to Memory on Store
mux_2x1 #(.DATA_WIDTH(32)
         )mux_2x1_u13 (
                      .ip1(fwd_data_ALU_src2), 
                      .ip0(dx_gpr_rd_data2), 
                      .sel(x_fwd_alu_src2[1]), 
                      .out(x_STORE_write_data)
                     );
//ALU module
alu alu_u0(
	  .r1     (ALU_datain1),
	  .r2     (ALU_datain2),
	  .control(x_ALU_Control),
	  .result (x_ALU_Result )
         );

forward_stall forward_stall_u0(
                               .gpr_wr_addr(gpr_wr_addr), 
                               .xm_gpr_wr_addr(xm_gpr_wr_addr),
                               .mw_opcode(mw_opcode),
                               .xm_opcode(xm_opcode),
                               .xm_rt(xm_rt),
                               .dx_gpr_rd_addr1(dx_gpr_rd_addr1),
                               .dx_rt(dx_rt),
                               .dx_isSLL_SRL(dx_isSLL_SRL),
                               .dx_opcode(dx_opcode),
                               .fd_opcode(fd_opcode),
                               .fd_funct(fd_funct),
                               .fd_rs(fd_rs),
                               .fd_rt(fd_rt),
                               .gpr_rd_addr1(gpr_rd_addr1),
                               .d_isSLL_SRL(d_isSLL_SRL),
                               .d_fwd_rs(d_fwd_rs),
                               .d_fwd_rt(d_fwd_rt),
                               .d_stall(d_stall),
                               .x_fwd_alu_src1(x_fwd_alu_src1),
                               .x_fwd_alu_src2(x_fwd_alu_src2)
                              );
assign PC       = pc;
assign Din      = xm_STORE_write_data; 
assign Addr     = xm_ALU_Result;
assign MemWrite = m_MemWrite;
assign MemRead  = m_MemRead; 

always@(posedge clock or posedge reset) begin
  if(reset==1'b1) begin
    xm_gpr_wr_addr <= #TD 5'd0; 
    gpr_wr_addr <= #TD 5'd0; 
  end
  else begin
    xm_gpr_wr_addr <= #TD x_gpr_wr_addr;
    gpr_wr_addr    <= #TD xm_gpr_wr_addr;
  end
end

 
endmodule

