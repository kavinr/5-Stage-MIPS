
`timescale 1ns/100ps
module alu (
	input [31:0] r1,
	input [31:0] r2,
	input [2:0] control,
	output [31:0] result
);

integer in1, in2, out;

always @(*) 
	begin
		in1 = r1;
		in2 = r2;
		case(control)
			3'b000: out = in1 + in2; //ADD and ADDI also LB, LW, SB ,SW
			3'b001: out = in1 & in2; //AND and ANDI
			3'b010: out = in1 | in2; //OR and ORI
			3'b011: out = in1 << in2; //SLL, in1 shifted left by in2 amount
			3'b100: out = (in1 < in2) ? 1 : 0; //SLT and SLTI, set on less than, output is set to 1 if in1<in2 else 0
			3'b101: out = in1 >> in2; //SRL, in1 shifted right by in2 amount
			3'b110: out = in1 - in2; //SUB
			3'b111: out = in1 ^ in2; //XOR and XORI
// 			4'b1000: begin //BEQ, zero flag is 1 if branch is true i.e., if both inputs are equal
// 						if (in1 == in2) bcond_w = 1'b1;
// 						else bcond_w = 1'b0;
// 					  end
// 			4'b1001: begin //BNE, zero flag is 1 if branch is true i.e., if both inputs are not equal
// 						if (in1 != in2) bcond_w = 1'b1;
// 						else bcond_w = 1'b0;
// 					  end
// 			default: out = 0;
		endcase
	end
	assign result = out;
endmodule

