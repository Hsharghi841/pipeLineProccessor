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
//		$finish;
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

	wire [74:0] MEMWBout;
	
	// assign regWrite = 1;
	// assign alusrc = 0;
	// assign memRead = 0;
	// assign memWrite = 0;
	// assign memToReg = 0	

	
	register #(.n(32)) PC (
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
		.wrData (MEMWBout[73] ? MEMWBout[31:0] : (MEMWBout[72] | MEMWBout[71] ? pcOut + 4 : MEMWBout[68:37])),
		.regWrite (MEMWBout[74]),
		.rdData1 (rfRd1),
		.rdData2 (rfRd2)
	);

    wire [123:0] IDEXout;
    register #(.n(124)) IDEXreg(.clk(clk), .rst(rst), .in({IFIDout[24:20],IFIDout[19:15],regWrite, aluop,alusrc,memRead,memWrite,memToReg,jal,jalr,beq,bltu, immGenOut, rfRd2, rfRd1,IFIDout [11:7]}), .out(IDEXout)); // سیم های جدید به سمت چپ اضافه شوند
//                                                         123                           113   112 109  108    107      106      105    104 103 102  101  100    69  68 37   36 5   4          0
	wire [76:0] EXMEMout;
    // Instantiate dataHazard module
    wire [1:0] forwardA, forwardB;
    dataHazard dh (
        .rs1(IDEXout[118:114]),
        .rs2(IDEXout[123:119]),
        .ex_rd(EXMEMout[75:71]),
        .mem_rd(MEMWBout[36:32]),
        .wb_rd(MEMWBout[36:32]),
        .ex_regWrite(EXMEMout[76]),
        .mem_regWrite(MEMWBout[74]),
        .wb_regWrite(MEMWBout[74]),
        .forwardA(forwardA),
        .forwardB(forwardB)
    );

    // Update ALU inputs based on forwarding
    wire [31:0] aluIn1, aluIn2;
    assign aluIn1 = (forwardA == 2'b10) ? EXMEMout[31:0] :
                    (forwardA == 2'b01) ? MEMWBout[68:37] :
                    (forwardA == 2'b11) ? MEMWBout[31:0] : IDEXout[36:5];

    assign aluIn2 = (forwardB == 2'b10) ? EXMEMout[31:0] :
                    (forwardB == 2'b01) ? MEMWBout[31:0] :
                    (forwardB == 2'b11) ? MEMWBout[31:0] : IDEXout[108] ? IDEXout[100:69] : IDEXout[68:37];


    alu aluInstance (
		.op1(aluIn1),
		.op2(aluIn2),
		.aluop(IDEXout[112:109]),
		.result(aluOut),
		.zero(zero)
	);

	
    register #(.n(77)) EXMEMreg (.clk(clk), .rst(rst), .in({IDEXout[113], IDEXout[4:0], IDEXout[68:37], IDEXout[107:101], aluOut}), .out(EXMEMout));
	//                                                 76       75        71   70        39    38         32     31   0
	memory #(.kind(`dataMemory)) dm (
		.clk(clk),
		.rst(rst),
		.memRead(EXMEMout[38]),
		.memWrite(EXMEMout[37]),
		.addressIn(EXMEMout[31:0]),
		.dataIn(EXMEMout[70:39]),
		.out(dmOut)
	);

    register #(.n(75)) MEMWBreg (.clk(clk), .rst(rst), .in({EXMEMout[76], EXMEMout[36:32], EXMEMout[31:0], EXMEMout[75:71], dmOut}), .out(MEMWBout));
//                                                               74       73           69   68         37   36          32  31   0

	
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



module dataHazard (
    input [4:0] rs1, rs2, ex_rd, mem_rd, wb_rd,
    input ex_regWrite, mem_regWrite, wb_regWrite,
    output reg [1:0] forwardA, forwardB
);
    always @(*) begin
        // Default forwarding signals
        forwardA = 2'b00;
        forwardB = 2'b00;

        // EXE to EXE forwarding
        if (ex_regWrite && (ex_rd != 0) && (ex_rd == rs1)) forwardA = 2'b10;
        if (ex_regWrite && (ex_rd != 0) && (ex_rd == rs2)) forwardB = 2'b10;

        // MEM to EXE forwarding
        if (mem_regWrite && (mem_rd != 0) && (mem_rd == rs1) && !(ex_regWrite && (ex_rd == rs1))) forwardA = 2'b01;
        if (mem_regWrite && (mem_rd != 0) && (mem_rd == rs2) && !(ex_regWrite && (ex_rd == rs2))) forwardB = 2'b01;

        // WB to EXE forwarding
        if (wb_regWrite && (wb_rd != 0) && (wb_rd == rs1) && !(ex_regWrite && (ex_rd == rs1)) && !(mem_regWrite && (mem_rd == rs1))) forwardA = 2'b11;
        if (wb_regWrite && (wb_rd != 0) && (wb_rd == rs2) && !(ex_regWrite && (ex_rd == rs2)) && !(mem_regWrite && (mem_rd == rs2))) forwardB = 2'b11;
    end
endmodule

