`timescale 1ns/1ps
module ALU_design(
	  input [31:0] A,B,  		// ALU 32-bit Inputs                 
	  input [4:0] ALU_CONTROL,	// ALU Control Signal
	  input clk,					// clock signal
	  output reg [31:0] Y, 		// ALU 32-bit Output
	  output reg Z, V, N, C		// Zero, Overflow, Negative, and Carry Flags
	);
	 
	wire [31:0] resAlu;
	wire [31:0] resBool;
	wire [31:0] resComp;
	wire [31:0] resShift;
	wire Zf,Vf,Nf,Cf;
	
	ARITHMETIC_UNIT arith(
	.clk(clk),
	.A(A),
	.B(B),
	.ALU_CONTROL(ALU_CONTROL),
	.Z(Zf),
	.V(Vf),
	.N(Nf),
	.C(Cf),
	.Y(resAlu)
	);
	
	BOOLEAN_UNIT boolian(
	.A(A),
	.B(B),
	.ALU_CONTROL(ALU_CONTROL),
	.Y(resBool)
	);
	
	COMPARISON_UNIT comp(
	.A(A),
	.B(B),
	.ALU_CONTROL(ALU_CONTROL),
	.Y(resComp)
	);
	
	SHIFTER_UNIT shift(
	.A(A),
	.B(B[4:0]),
	.ALU_CONTROL(ALU_CONTROL),
	.Y(resShift)
	);
	
	
	always @(posedge clk) begin
		case(ALU_CONTROL[4:3])
		2'b00: begin
			Y = resAlu; 
			Z = Zf; 
			V = Vf; 
			N = Nf; 
			C = Cf; 
		end
		2'b01:
			Y = resBool;
		2'b10:
			Y = resComp;
		2'b11: 
			Y = resShift;
		endcase
	end
	
endmodule

module ADD_SUB_MODULE(
	input [31:0] A,B,
	input [4:0] ALU_CONTROL,
	
	output reg Z, V, N, C,	//flags
	output reg [31:0] Y
	);
	
	reg AVF, E, As, Bs; //sign ffs
	reg [30:0]T;
	
 always @(*) begin
	As = A[31];
	Bs = B[31]; 
	if (ALU_CONTROL[1:0] == 2'b00) begin //ADD
		if (As ^ Bs == 0) begin //As = Bs
			{E,Y[30:0]} = A[30:0] + B[30:0];
			AVF = E;
			Y[31] = As;
			if (AVF == 1) begin
				V = 1; // yes overflow
				C = 1; // E =C
			end
			else begin
				V = 0; // no overflow
				C = 0; //E =C
			end
			if (Y[30:0] == 31'd0)
				Z =1; // result zero
			else 
				Z =0; //result not zero
			if (As == 1)
				N = 1; //result negative
			else
				N = 0; //result positive
		end
		else if (As ^ Bs == 1) begin //As != Bs
			T[30:0] = ~B[30:0];
			{E,Y[30:0]} = A[30:0] + T[30:0] + 31'd1;
			AVF = 0;
			V = 0; //reset overflow flag
			if (E == 1) begin //A >= B
				C = 1; //update carry
				if (Y[30:0] == 31'd0) begin
					Y[31] = 0; //adjust sign bit of the result
					Z = 1; // zero
					N = 0; // not negative
				end
				else begin
					if (As == 1) begin //since A is Augend 
						Z = 0; // non zero
						N = 1; // negative
						Y[31]=1;
					end
					else begin
						Z= 0;  //non zero
						N = 0; //positive
						Y[31]=0;
					end
			   end
					
			end
			
			else if (E == 0) begin // A < B
				C = 0; // carry uppdate
				Y[30:0] = ~(Y[30:0]) + 1; //adjust  magnitude
				Y[31] = ~As;  //adjust sign bit of the result
				if (Y[31] == 1) begin 
						Z = 0; // non zero
						N = 1; // negative
				end
				else begin
						Z = 0;  //non zero
						N = 0; //positive
				end
		
			end
			
		end
		
	end
	
	else if (ALU_CONTROL[1:0] == 2'b01) begin //SUB
		if (As ^ Bs == 1) begin //As != Bs
			{E,Y[30:0]} = A[30:0] + B[30:0];
			AVF = E;
			if (AVF == 1) begin
				V = 1; // yes overflow
				C = 1; //E=C
			end
			else begin
				V = 0; // no overflow 
				C = 0; // E=C
			end
			if (Y[30:0] == 0)
				Z =1; // result zero
			else 
				Z =0; //result not zero
			if (As == 1)
				N = 1; //result negative
			else
				N = 0; //result positive
		end
		else if (As ^ Bs == 0) begin //As = Bs
			T[30:0] = ~B[30:0];
			{E,Y[30:0]} = A[30:0] + T[30:0] + 31'd1;
			AVF = 0;
			V = 0; //reset overflow flag
			if (E == 1) begin //A >= B
				C = 1; //carry update
				if (Y[30:0] == 31'd0) begin
					Y[31] = 0; //adjust sign bit of the result
					Z = 1; // zero
					N = 0; // not negative
				end
				else begin
					if (As == 1) begin //since A is Minuend 
						Z = 0; // non zero
						N = 1; // negative
						Y[31]=1;
					end
					else begin
						Z= 0;  //non zero
						N = 0; //positive
						Y[31]=0;
					end
				end
					
			end
			
			else if (E == 0) begin // A < B
				C = 0; //carry update
				Y[30:0] = ~(Y[30:0]) + 31'd1; //adjust  magnitude
				Y[31] = ~As;  //adjust sign bit of the result
				if (Y[31] == 1) begin 
						Z = 0; // non zero
						N = 1; // negative
				end
				else begin
						Z=  0;  //non zero
						N = 0; //positive
				end
		
			end
			
		end
			
	end
 end
endmodule

module MUL_MODULE(
	input [15:0] A,Bf, //A is multiplicand, Bf is multiplier
	output reg Z, V, N, C,	//flags
	output reg [31:0] Y //result
);
	reg E, As, Bs, Qs; //ffs
	reg [15:0] Q, B; //Q refers to A in the lecture notes
	integer SC; //counter

 always @(*) begin
	//values
	B[15:0] = Bf[15:0]; //copy Bf to B register
	As = A[15]; //update As
	Qs = A[15] ^ B[15]; //adjust Qs, will be the sign
	Q[14:0] = 15'd0; // load with 0 except sign bit
	Q[15] = Qs; //update sign bit
	E=0; //initially E=0
   SC = 15; //counter starts with 15
	
	while (SC>0) begin //loop (algorithm applied)
		if(B[0] == 1)
			{E,Q[14:0]} = Q[14:0]+A[14:0]; //add
	
		{E,Q[14:0],B[14:0]} = {E,Q[14:0],B[14:0]} >> 1; //shr
		SC = SC - 1; //decrease counter
	end //end loop
 
	Y[29:0] = {Q[14:0], B[14:0]}; //load the magnitude
	Y[30]=0;  //load 30th bit of the result with 0 (which is actuall dont care)
	Y[31]=Qs; //load the sign bit
 
	V=0; //NO OVERFLOW OCCURS for multiplication
	
	if (E == 1) begin
		C = 1; // carry update
	end
	else begin
		C = 0; //carry update
	end
	
	if (Qs == 1)
		N = 1; //negative
	else
		N = 0;
		
	if (Y[29:0] == 0) 
		Z = 1; //zero
	else
		Z = 0; //non-zero
 end
endmodule


module DIV_MODULE(
	input [31:0] AQ, //dividend 32bit with sign
	input [15:0] B, //divisor 16bit with sign
	output reg Z, V, N, C,	//flags
	output reg [31:0] Y
);
	integer SC;
	reg E, As, Bs, Qs, DVF;
	reg [29:0]AQ_TEMP;
	reg [14:0]T; //temp reg will be used
	
	always @(*) begin
	//values
	As = AQ[31]; //Dividend sign
	Bs = B[15]; //divisior sign
	Qs = As ^ Bs; //final sign bit
	AQ_TEMP[29:0] = {AQ[30:16],AQ[14:0]}; //AQ register in algorithm	
	E= 0; //initially zero
	SC = 15; //initialize counter
	T[14:0] = ~(B[14:0]); //I implement these seperately bec of some errors
	{E,AQ_TEMP[29:15]} = AQ_TEMP[29:15]+ T[14:0] + 15'd1;
	
	if (E == 1) begin //A[30:16] >= B
		{E,AQ_TEMP[29:15]} = AQ_TEMP[29:15]+B[14:0];
		DVF = 1;
		V = 1; //overflow update
		C = 1;
	end
	
	else begin
		{E,AQ_TEMP[29:15]} = AQ_TEMP[29:15]+B[14:0];
		DVF = 0;
		V = 0; //overflow update
		
		while (SC>0) begin
			{E,AQ_TEMP[29:0]} = {E,AQ_TEMP[29:0]} << 1; //shL
			
			if (E == 0) begin
				T[14:0] = ~(B[14:0]); 
				{E,AQ_TEMP[29:15]} = AQ_TEMP[29:15]+ T[14:0] + 15'd1;
				
				if (E == 1) begin 
					AQ_TEMP[0] = 1;
					SC = SC -1;
				end
				else begin
					{E,AQ_TEMP[29:15]} = AQ_TEMP[29:15]+B[14:0];
					SC = SC -1;
				end
			end
			
			else begin
				T[14:0] = ~(B[14:0]); 
				{E,AQ_TEMP[29:15]} = AQ_TEMP[29:15]+ T[14:0] + 15'd1;
				AQ_TEMP[0] = 1;
				SC = SC -1;
			end
		end //while end
		
		Y[31:0]={Qs,1'b0,AQ_TEMP[29:0]}; //result is 32bit as below
		//Y =[sign, 1bit 0, 15 bit remainder, 15 bit quotient];
		
		V = 0; //no overflow
		C = 0; //carry update	
		if (Qs == 1)
			N = 1; //negative
		else
			N = 0;
		if (AQ_TEMP[14:0] == 0) 
			Z = 1; //zero
		 else
			Z = 0; //non-zero
	end
	
	end //always
	
endmodule


module ARITHMETIC_UNIT( 
	input clk,	// clock signal
	//other parameters 
	input [31:0] A,B, 
	input [4:0] ALU_CONTROL,
	output reg Z, V, N, C,	//flags
	output reg [31:0] Y //output
	);	
	wire [31:0] resAddSub;
	wire [31:0] resDiv;
	wire [31:0] resMul;
	wire Zf0,Vf0,Nf0,Cf0,Zf1,Vf1,Nf1,Cf1,Zf2,Vf2,Nf2,Cf2;
	
	ADD_SUB_MODULE add_sub(
	.A(A),
	.B(B),
	.ALU_CONTROL(ALU_CONTROL),
	.Z(Zf0),
	.V(Vf0),
	.N(Nf0),
	.C(Cf0),
	.Y(resAddSub)
	);
	
	DIV_MODULE div(
	.AQ(A),
	.B(B[15:0]),
	.Z(Zf1),
	.V(Vf1),
	.N(Nf1),
	.C(Cf1),
	.Y(resDiv)
	);
	
	MUL_MODULE mul(
	.A(A[15:0]),
	.Bf(B[15:0]),
	.Z(Zf2),
	.V(Vf2),
	.N(Nf2),
	.C(Cf2),
	.Y(resMul)
	);
	
	always @(posedge clk)
	begin
		case(ALU_CONTROL[1:0])
		2'b00: begin//signed addition
			Y = resAddSub;
			Z = Zf0;
			V = Vf0;
			N = Nf0;
			C = Cf0;
		end
		2'b01: begin //signed subtraction
			Y = resAddSub;
			Z = Zf0;
			V = Vf0;
			N = Nf0;
			C = Cf0;
		end
		2'b10: begin //signed multiplication
			Y = resMul;
			Z = Zf2;
			V = Vf2;
			N = Nf2;
			C = Cf2;
		end
		2'b11: begin //signed division
			Y = resDiv;
			Z = Zf1;
			V = Vf1;
			N = Nf1;
			C = Cf1;
		end
		endcase
	end
endmodule


module BOOLEAN_UNIT( //parameters 
	input [31:0] A,B,
	input [4:0] ALU_CONTROL,
	output reg [31:0] Y
	);	
	
	always @(*)
	begin
		case(ALU_CONTROL)
		5'b01000: //bit-wise and
			Y = A & B;
		5'b01001: //bit-wise or
			Y = A | B;
		5'b01010: //bit-wise xor
			Y = A ^ B;
		5'b01011: //bit-wise nor
			Y = ~(A | B);
		5'b01100: //bit-wise nand
			Y = ~(A & B);
		5'b01101: //bit-wise xnor
			Y = A ~^ B;
		endcase
	end
	
endmodule


module COMPARISON_UNIT( //write parameters 
	input [31:0] A,B,
	input [4:0] ALU_CONTROL,
	//input N,V,Z,C, -> I run this module without using flags.
	output reg [31:0] Y
	);	
	always @(*)
	begin
		case(ALU_CONTROL[1:0])
		2'b00: begin //EQ
			if ( (A[31:0] == B[31:0]) || (A[30:0] == B[30:0] && A[30:0] == 31'd0) ) 
				Y = 32'd1;
		end
		2'b01: begin //LT
			if (A[31] > B[31])
				Y = 32'd2;
			else if (A[31] == 1) begin
				if (A[30:0] > B[30:0])
					Y = 32'd2;
			end
			else begin
				if (A[30:0] < B[30:0])
					Y = 32'd2;
			end
		end
		2'b10: begin //GT
			if (A[31] < B[31])
				Y = 32'd4;
			else if (A[31] == 1) begin
				if (A[30:0] < B[30:0])
					Y = 32'd4;
			end
			else begin
				if (A[30:0] > B[30:0])
					Y = 32'd4;
			end
		end
		endcase
	end
	
endmodule


module SHIFTER_UNIT( //parameters 
	input signed [31:0] A,
	input [4:0] B,
	input [4:0] ALU_CONTROL,
	output reg signed [31:0] Y //in order to handle >>>
	);
	integer n=0;
	always @(*)
	begin
		case(ALU_CONTROL[1:0])
		2'b00: // LSL
			Y = A << B[4:0];
		2'b01: // LSR
			Y = A >> B[4:0];
		2'b10: // ASR
			Y = A >>> B[4:0];
		2'b11: begin// REV
			for (n=0; n<32; n=n+1) begin //loop to reverse
				Y[n] = A[31-n];
			end
		end
		endcase
	end
	
endmodule
