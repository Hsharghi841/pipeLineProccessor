// concatenating a, b
// {a, b}
// replicating a four times {4{a}}
// concat and replicate
// {{4{a}}, {3{b}}}

`define period 10
module test;
	reg clk, rst;
	cpu dut (clk, rst);
	integer i;
	always #(`period/2) clk = ~clk;
	initial begin
		clk = 1;
		rst = 1;
		#1;
		rst = 0;
		for (i = 0; i < 32; i = i + 1)
			$display ("r[%d]=%d", i, dut.rf.cells[i]);
		#`period;
		for (i = 0; i < 32; i = i + 1)
			$display ("r[%d]=%d", i, dut.rf.cells[i]);
		$finish;
	end
endmodule

// for executing r-type instructions only
`define instMemory 0
`define dataMemory 1

module cpu (input clk, rst);

	wire [31:0] pcOut, instruction, dmOut, aluOut, rfRd1, rfRd2;
	wire [31:0] immGenOut;
	wire [3:0] aluop;
	wire memToReg, regWrite, memRead, memWrite, jalr, jal, beq, zero, bltu;
	
	// assign regWrite = 1;
	// assign alusrc = 0;
	// assign memRead = 0;
	// assign memWrite = 0;
	// assign memToReg = 0	

	
	register PC (
		.clk(clk),
		.rst (rst),
		.in (jalr ? {aluOut & -2} : (jal | (beq & zero) | (bltu & aluOut[31]) ? (immGenOut << 1) + pcOut : pcOut + 4)),
		.out (pcOut)
	);

	memory #(.kind(`instMemory)) im (
		.clk (clk),
		.rst (rst),
		.memRead (1'b1),
		.memWrite (1'b0),
		.addressIn (pcOut),
		.dataIn(32'd0),
		.out (instruction)
	);

    wire [63:0] IFIDout;
    register #(.n(64)) IFIDreg (.clk(clk), .rst(rst), .in({pcOut, instruction}), .out(IFIDout));

	controlUnit cu (.inst(IFIDout[31:0]), .aluop(aluop), .alusrc(alusrc), .memRead(memRead), 
					.regWrite(regWrite), .memToReg(memToReg), .memWrite(memWrite), .jalr(jalr), .jal(jal), .beq(beq), .bltu(bltu));

	immGen ig (.inst(IFIDout[31:0]), .immGenOut(immGenOut));

	registerFile rf (
		.clk (clk),
		.rst (rst),
		.rs1(IFIDout [19:15]),
		.rs2(IFIDout [24:20]),
		.rd(MEMWBout[36:32]),
		.wrData (MEMWBout[36] ? MEMWBout[31:0] : (jal | jalr ? pcOut + 4 : MEMWBout[69:38])),
		.regWrite (MEMWBout[37]),
		.rdData1 (rfRd1),
		.rdData2 (rfRd2)
	);

    wire [116:0] IDEXout;
    register #(.n(117)) IDEXreg(.clk(clk), .rst(rst), .in({regWrite, aluop,alusrc,memRead,memWrite,memToReg,jal,jalr,beq,bltu, immGenOut, rfRd2, rfRd1,IFIDout [11:7]}), .out(IDEXout)); // سیم های جدید به سمت چپ اضافه شوند

	alu aluInstance (
		.op1(IDEXout[4:0]),
		.op2(alusrc ? IDEXout[99:68] : IDEXout[67:36]),
		.aluop(aluop),
		.result(aluOut),
		.zero(zero)
	);

	wire [49:0] EXMEMout;
    register #(.n(50)) EXMEMreg (.clk(clk), .rst(rst), .in({IFIDout[116], IFIDout[11:7], IDEXout[15:11], IDEXout[113:107], aluOut}), .out(EXMEMout));
	//                                                           49       48        44   43         39   38            32  31   0
	memory #(.kind(`dataMemory)) dm (
		.clk(clk),
		.rst(rst),
		.memRead(EXMEMout[38]),
		.memWrite(EXMEMout[37]),
		.addressIn(EXMEMout[31:0]),
		.dataIn(EXMEMout[43:39]),
		.out(dmOut)
	);

	wire [74:0] MEMWBout;
    register #(.n(75)) MEMWBreg (.clk(clk), .rst(rst), .in({EXMEMout[36:32], EXMEMout[31:0], EXMEMout[49:44], dmOut}), .out(MEMWBout));
//                                                          74           70   69         38   37          32  31   0

	
endmodule

module register #(parameter n) (input clk, rst, input [n - 1:0] in, output reg [n - 1:0] out);
	always @(posedge clk, posedge rst) begin
		if (rst)
			out <= 0;
		else
			out <= in;
	end
endmodule

module memory #(parameter kind) (input clk, rst, memRead, memWrite, input [31:0] addressIn, dataIn, output [31:0] out);
	reg [7:0] cells [1023:0];
	always @(posedge clk, posedge rst) begin
		if (rst) begin
			if (kind == `dataMemory)
				$readmemh("data.hex", cells);
			else
				$readmemh("instructions.hex", cells);
		end
		// little endian
		else if (memWrite) begin
			cells[(addressIn+0) % 1024] <= dataIn   [7:0];
			cells[(addressIn+1) % 1024] <= dataIn [15:8];
			cells[(addressIn+2) % 1024] <= dataIn [23:16];
			cells[(addressIn+3) % 1024] <= dataIn [31:24];
		end
		// big endian
		// else if (memWrite) begin
		// 	cells[addressIn] <= dataIn   [31:24];
		// 	cells[addressIn+1] <= dataIn [23:16];
		// 	cells[addressIn+2] <= dataIn [15:8];
		// 	cells[addressIn+3] <= dataIn [7:0];
		// end
	end
	// little endian
	assign out [7:0]   = memRead ? cells[(addressIn+0) % 1024] : 8'd0;
	assign out [15:8]  = memRead ? cells[(addressIn+1) % 1024] : 8'd0;
	assign out [23:16] = memRead ? cells[(addressIn+2) % 1024]  : 8'd0;
	assign out [31:24] = memRead ? cells[(addressIn+3) % 1024]   : 8'd0;
	// big endian
	// assign out [31:24] = memRead ? cells[addressIn]   : 8'd0;
	// assign out [23:16] = memRead ? cells[addressIn+1] : 8'd0;
	// assign out [15:8] = memRead ? cells[addressIn+2]  : 8'd0;
	// assign out [7:0] = memRead ? cells[addressIn+3]   : 8'd0;
endmodule

module registerFile (input clk, rst, regWrite, input [4:0] rs1, rs2, rd, input [31:0] wrData, output [31:0] rdData1, rdData2);
	reg [31:0] cells [31:0];
	always @(posedge clk, posedge rst) begin
		if (rst)
			$readmemh("reg.hex", cells);
		else if (regWrite)
			cells [rd] <= wrData;
		cells[0] = 0;
	end
	assign rdData1 = cells[rs1];
	assign rdData2 = cells[rs2];
endmodule

`define ADD 0
`define SUB 1
`define AND 2
`define XOR 3

module alu (input [31:0] op1, op2, input [3:0] aluop, output zero, output reg [31:0] result);
	
	always @(*) begin
		case (aluop)
			`ADD: result = op1 + op2;
			`SUB: result = op1 - op2;
			`AND: result = op1 & op2;
			`XOR: result = op1 ^ op2;
		endcase
	end
	
	assign zero = (result == 0 ? 1'b1 : 0);

endmodule

module immGen(input [31:0] inst, output reg [31:0] immGenOut);
	wire [6:0] opcode = inst [6:0];
	always @(*) begin
		case (opcode)
			7'b0010011, 7'b0000011: begin
				immGenOut = {{20{inst[31]}}, {inst[31:20]}};
			end
			7'b1100111: begin
				immGenOut = {{20{inst[31]}}, inst[31:20]};
			end	
			7'b0100011: begin
				immGenOut = {{20{inst[31]}}, inst[31:25], inst[11:7]};
			end	
			7'b1101111: begin
				immGenOut = {{12{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21]};
			end	
			7'b1100011:begin
				immGenOut = {{20{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8]};
			end	
		endcase
	end
endmodule

module controlUnit (input [31:0] inst, output reg alusrc, memRead, memToReg, regWrite, memWrite, jalr, jal, beq, bltu, [3:0] aluop);
	wire [6:0] opcode = inst [6:0];
	always @(*) begin
		memRead = 0;
		alusrc = 0;
		memToReg = 0;
		regWrite = 1;
		memWrite = 0;
		jalr = 0;
		jal = 0;
		beq = 0;
		bltu = 0;
		case (opcode)
			7'b0110011: begin
				case(inst[31:25])
					7'b0000000: aluop = `ADD;
				 	7'b0100000: aluop = `SUB;
				endcase
			end
			7'b0010011: begin // addi
				alusrc = 1;
				aluop = `ADD;
			end
			7'b0000011: begin // lw
				memRead = 1;
				alusrc = 1;
				memToReg = 1;
				aluop = `ADD;
			end
			7'b0100011: begin // sw
				alusrc = 1;
				regWrite = 0;
				memWrite = 1;
				aluop = `ADD;
			end
			7'b1100111: begin // jalr
				alusrc = 1;
				jalr = 1;
				aluop = `ADD;
			end
			7'b1101111: begin // jal
				jal = 1;
			end
			7'b1100011: begin // beq, bltu
				aluop = `SUB; 
				case (inst[14:12])
					3'b000: beq = 1;
					3'b110: bltu = 1;
				endcase
			end
		endcase
	end
endmodule

