`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/11/2023 03:37:40 AM
// Design Name: 
// Module Name: part4
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/19/2023 12:25:11 PM
// Design Name: 
// Module Name: Memory
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Memory(
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1
    input wire cs, //Chip is enable when cs = 0
    input wire clock,
    output reg[7:0] o // Output
);
    //Declaration oýf the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o = ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] <= data; 
        end
    end
endmodule

module dec_16bit_or(
    input wire [15:0] A,
    output wire B);
    wire [15:0] int;
    assign int[0] = A[0];
    generate
        genvar x;
        for (x = 1; x < 16; x = x + 1)
        begin: OR
           assign int[x] = int[x - 1] | A[x];
        end
    endgenerate
    assign B = int[15];
endmodule



module dec_4to16(
    input wire [3:0] I,
    output reg [15:0] O);
    always @(*)
            case(I)
            4'h0: begin O = 16'h0001; end
            4'h1: begin O = 16'h0002; end
            4'h2: begin O = 16'h0004; end
            4'h3: begin O = 16'h0008; end
            4'h4: begin O = 16'h0010; end
            4'h5: begin O = 16'h0020; end
            4'h6: begin O = 16'h0040; end
            4'h7: begin O = 16'h0080; end   
            4'h8: begin O = 16'h0100; end
            4'h9: begin O = 16'h0200; end
            4'ha: begin O = 16'h0400; end
            4'hb: begin O = 16'h0800; end
            4'hc: begin O = 16'h1000; end
            4'hd: begin O = 16'h2000; end
            4'he: begin O = 16'h4000; end
            4'hf: begin O = 16'h8000; end                                  
            default: begin O = 16'h0000; end            
        endcase      
endmodule







module mux_2to1_8bit (
  input [7:0] i0,
  input [7:0] i1,
  input s,
  output reg [7:0]o
);

always @(*) begin
  case (s)
    1'b0: o = i0;
    1'b1: o = i1;
  endcase
end

endmodule

 module mux8to1 (
  input [7:0] data_0,
  input [7:0] data_1,
  input [7:0] data_2,
  input [7:0] data_3,
  input [7:0] data_4,
  input [7:0] data_5,
  input [7:0] data_6,
  input [7:0] data_7,
  input [2:0] sel,
  output reg [7:0] out
);

always @(*) begin
  case (sel)
    3'b000: out = data_0;
    3'b001: out = data_1;
    3'b010: out = data_2;
    3'b011: out = data_3;
    3'b100: out = data_4;
    3'b101: out = data_5;
    3'b110: out = data_6;
    3'b111: out = data_7;
    default: out = 8'b0;
  endcase
end

endmodule

module mux_16to1_1bit (
    input [15:0] data_in,
    input [3:0] sel,
    output reg data_out
);

always @* begin
    case (sel)
        4'b0000: data_out = data_in[0];
        4'b0001: data_out = data_in[1];
        4'b0010: data_out = data_in[2];
        4'b0011: data_out = data_in[3];
        4'b0100: data_out = data_in[4];
        4'b0101: data_out = data_in[5];
        4'b0110: data_out = data_in[6];
        4'b0111: data_out = data_in[7];
        4'b1000: data_out = data_in[8];
        4'b1001: data_out = data_in[9];
        4'b1010: data_out = data_in[10];
        4'b1011: data_out = data_in[11];
        4'b1100: data_out = data_in[12];
        4'b1101: data_out = data_in[13];
        4'b1110: data_out = data_in[14];
        4'b1111: data_out = data_in[15];
        default: data_out = 1'b0;
    endcase
end

endmodule
module mux_16to1(
    input wire [7:0] i0,
    input wire [7:0] i1,
    input wire [7:0] i2,
    input wire [7:0] i3,
    input wire [7:0] i4,
    input wire [7:0] i5,
    input wire [7:0] i6,
    input wire [7:0] i7,
    input wire [7:0] i8,
    input wire [7:0] i9,
    input wire [7:0] i10,
    input wire [7:0] i11,
    input wire [7:0] i12,
    input wire [7:0] i13,
    input wire [7:0] i14,
    input wire [7:0] i15,
    input wire [3:0] s,
    output reg [7:0] o);
    
    
always @* begin
    case (s)
        4'b0000: o = i0;
        4'b0001: o = i1;
        4'b0010: o = i2;
        4'b0011: o = i3;
        4'b0100: o = i4;
        4'b0101: o = i5;
        4'b0110: o = i6;
        4'b0111: o = i7;
        4'b1000: o = i8;
        4'b1001: o = i9;
        4'b1010: o = i10;
        4'b1011: o = i11;
        4'b1100: o = i12;
        4'b1101: o = i13;
        4'b1110: o = i14;
        4'b1111: o = i15;
        default: o = 8'b0;
    endcase
end

endmodule


module mux_4to1_8bit (
    input [7:0] i0,
    input [7:0] i1,
    input [7:0] i2,
    input [7:0] i3,
    input [1:0] s,
    output reg o
);

always @* begin
    case (s)
        2'b00: o = i0;
        2'b01: o = i1;
        2'b10: o = i2;
        2'b11: o = i3;
    endcase
end

endmodule
module reg_8bit (D, clk, enable, Q);
    input wire [7:0] D; // Data input
    input wire clk, enable;
    output reg [7:0] Q; // output Q
    always @(posedge clk)
    begin
        if (enable)
            Q <= D;
    end
endmodule

module reg_1bit(D, clk, enable, Q);
    input wire D, clk, enable;
    output reg Q;
    always @(posedge clk)
    begin
        if (enable)
            Q <= D;
    end            
endmodule

module comparator(
    input signed [7:0] a,
    input signed [7:0] b,
    output reg [7:0] out
);

always @* begin
    if (a > b) begin
        out = a;
    end else if (a < b) begin
        out = 8'b0000_0000;
    end
end

endmodule




module zero_check(
    input wire [7:0] A,
    output wire B);
    wire [7:0] int;
    assign int[0] = A[0];
    generate
        genvar x;
        for (x = 1; x < 8; x = x + 1)
        begin: OR
            assign int[x] = int[x - 1] | A[x];        
        end
    endgenerate
    assign B = !int[7];
endmodule
module full_adder(
    input wire A, 
    input wire B,
    input wire Cin,
    output wire Cout,
    output wire Out);
    // Out
    wire AxorB;
    assign AxorB = A ^ B;
    assign Out = AxorB ^ Cin;
    // Cout
    wire AxorB_and_Cin;
    wire AandB;
    assign AandB = A & B;
    assign AxorB_and_Cin = AxorB & Cin;
    assign Cout = AandB | AxorB_and_Cin;
endmodule

module RPA_8bit(
    input wire [7:0] A,
    input wire [7:0] B,
    input wire Cin,
    output wire Cout,
    output wire [7:0] Out);
    wire [8:0]int_carry;
    assign int_carry[0] = Cin;
    generate
        genvar x;
        for (x = 0; x < 8; x = x +1)
        begin: RPA
            full_adder RPA(.A(A[x]), .B(B[x]), .Out(Out[x]),
            .Cin(int_carry[x]), .Cout(int_carry[x+1])); 
        end       
    endgenerate
    assign Cout = int_carry[8];
endmodule


module not_8bit(
    input wire [7:0] A,
    output wire [7:0] B);
    generate
        genvar x;
        for (x = 0; x < 8; x = x + 1)
        begin: NOT
            assign B[x] = !A[x];
        end    
    endgenerate
endmodule

module and_8bit(
    input wire [7:0] A,
    input wire [7:0] B,
    output wire [7:0] C);
    generate
        genvar x;
        for (x = 0; x < 8; x = x + 1)
        begin: AND
            assign C[x] = A[x] & B[x];
        end    
    endgenerate
endmodule

module or_8bit(
    input wire [7:0] A,
    input wire [7:0] B,
    output wire [7:0] C);
    generate
        genvar x;
        for (x = 0; x < 8; x = x + 1)
        begin: AND
            assign C[x] = A[x] | B[x];
        end    
    endgenerate
endmodule

module xor_8bit(
    input wire [7:0] A,
    input wire [7:0] B,
    output wire [7:0] C);
    generate
        genvar x;
        for (x = 0; x < 8; x = x + 1)
        begin: AND
            assign C[x] = A[x] ^ B[x];
        end    
    endgenerate
endmodule

module nand_8bit(
    input wire [7:0] A,
    input wire [7:0] B,
    output wire [7:0] C);
    generate
        genvar x;
        for (x = 0; x < 8; x = x + 1)
        begin: AND
            assign C[x] = ~(A[x] & B[x]);
        end    
    endgenerate
endmodule






module part1_8bit(I, Clk, E, FunSel, Q);
    input wire [7:0] I;
    input wire Clk;
    input wire E;
    input wire [1:0] FunSel;
    output wire [7:0] Q;
    // Register
    wire [7:0] mux_to_reg;
    reg_8bit register(.D(mux_to_reg), .clk(Clk), .enable(E), .Q(Q));
    // Multiplexer
    wire [7:0] mux_i0;
    assign mux_i0[7:0] = Q[7:0] - 8'd1; // Decrement
    wire [7:0] mux_i1;
    assign mux_i1[7:0] = Q[7:0] + 8'd1; // Increment
    wire [7:0] mux_i3;
    assign mux_i3 = 8'd0; // Load 0
    mux_4to1_8bit mux(.i0(mux_i3), .i1(I), .i2(mux_i0), 
                      .i3(mux_i1), .s(FunSel) ,.o(mux_to_reg));    
endmodule
module part1_16bit(I, Clk, E, FunSel, Q);
    input wire [15:0] I;
    input wire Clk;
    input wire E;
    input wire [1:0] FunSel;
    output wire [15:0] Q;
    // Register
    wire [15:0] mux_to_reg;
    reg_8bit register(.D(mux_to_reg), .clk(Clk), .enable(E), .Q(Q));
    // Multiplexer
    wire [15:0] mux_i0;
    assign mux_i0[15:0] = Q[15:0] - 16'd1; // Decrement
    wire [15:0] mux_i1;
    assign mux_i1[15:0] = Q[15:0] + 16'd1; // Increment
    wire [15:0] mux_i3;
    assign mux_i3 = 16'd0; // Load 0
    mux_4to1_8bit mux(.i0(mux_i3), .i1(I), .i2(mux_i0), 
                      .i3(mux_i1), .s(FunSel) ,.o(mux_to_reg));    
endmodule
module part_2a(
    I,
    LH,
    Enable,
    Clk,
    FunSel,
    IRout);
    // Data input
    input wire [7:0] I;
    // Low High
    input wire LH;
    wire [15:0] ir_data;
    mux_2to1_8bit low(.i0(I), .i1(IRout[7:0]), .o(ir_data[7:0]), .s(LH));
    mux_2to1_8bit high(.i0(IRout[15:8]), .i1(I), .o(ir_data[15:8]), .s(LH));
    // Enable
    input wire Enable;
    // FunSel
    input wire [1:0] FunSel;
    // Output
    output wire [15:0] IRout;
    // Register
    input wire Clk;
    part1_16bit IR(.Clk(Clk), .E(Enable), .Q(IRout),
                    .FunSel(FunSel), .I(ir_data));    
endmodule
module part_2b(
    input [3:0] regsel, // Register select input for R1, R2, R3, R4
    input [3:0] tsel,   // Register select input for T1, T2, T3, T4
    input [1:0] funsel, // Function select input
    input [7:0] data_in, // Data input
    input [2:0] o1sel, // Output 1 select input
    input [2:0] o2sel, // Output 2 select input
    output wire [7:0] o1, // Output 1
    output wire [7:0] o2,  // Output 2
    input wire clk
);
    wire [7:0] R1;
    wire [7:0] R2;
    wire [7:0] R3;
    wire [7:0] R4;
    wire [7:0] T1;
    wire [7:0] T2;
    wire [7:0] T3;
    wire [7:0] T4;
    mux8to1 out1(.data_0(T1), .data_1(T2), .data_2(T3), .data_3(T4), .data_4(R1), .data_5(R2), .data_6(R3), .data_7(R4), .sel(o1sel), .out(o1));
    mux8to1 out2(.data_0(T1), .data_1(T2), .data_2(T3), .data_3(T4), .data_4(R1), .data_5(R2), .data_6(R3), .data_7(R4), .sel(o2sel), .out(o2));
    // Enable signals for registers based on regsel and tsel inputs
    reg R1_en, R2_en, R3_en, R4_en;
    reg T1_en, T2_en, T3_en, T4_en;
    wire select_R1;
    assign select_R1 = regsel[0];
    wire select_R2;
    assign select_R2 = regsel[1];
    wire select_R3;
    assign select_R3 = regsel[2];
    wire select_R4;
    assign select_R4 = regsel[3];
    wire select_T1;
    assign select_T1 = tsel[0];
    wire select_T2;
    assign select_T2 = tsel[1];
    wire select_T3;
    assign select_T3 = tsel[2];
    wire select_T4;
    assign select_T4 = tsel[3];
    
    part1_8bit reg1(.E(select_R1), .Q(R1), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit reg2(.E(select_R2), .Q(R2), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit reg3(.E(select_R3), .Q(R3), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit reg4(.E(select_R4), .Q(R4), .Clk(clk), .I(data_in), .FunSel(funsel));
   
    part1_8bit treg1(.E(select_T1), .Q(T1), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit treg2(.E(select_T2), .Q(T2), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit treg3(.E(select_T3), .Q(T3), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit treg4(.E(select_T4), .Q(T4), .Clk(clk), .I(data_in), .FunSel(funsel));   
   
   
   
endmodule

module part_2c(
    input [3:0] regsel, // Register select input for R1, R2, R3, R4
    input [1:0] funsel, // Function select input
    input [7:0] data_in, // Data input
    input [1:0] o1sel, // Output 1 select input
    input [1:0] o2sel, // Output 2 select input
    output wire [7:0] o1, // Output 1
    output wire [7:0] o2,  // Output 2
    input wire clk
    
);

    wire [7:0] PC;
    wire [7:0] AR;
    wire [7:0] SP;
    wire [7:0] PCPast;
    mux_4to1_8bit out1(.i0(AR), .i1(SP), .i2(PCPast), .i3(PC), .s(o1sel), .o(o1));
    mux_4to1_8bit out2(.i0(AR), .i1(SP), .i2(PCPast), .i3(PC), .s(o2sel), .o(o2));
    // Enable signals for registers based on regsel and tsel inputs
    wire select_AR;
    assign select_AR = regsel[0];
    wire select_SP;
    assign select_SP = regsel[1];
    wire select_PCPast;
    assign select_PCPast = regsel[2];
    wire select_PC;
    assign select_PC = regsel[3];
    
    part1_8bit reg1(.E(select_AR), .Q(AR), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit reg2(.E(select_SP), .Q(SP), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit reg3(.E(select_PCPast), .Q(PCPast), .Clk(clk), .I(data_in), .FunSel(funsel));
    part1_8bit reg4(.E(select_PC), .Q(PC), .Clk(clk), .I(data_in), .FunSel(funsel));
   
   
endmodule




module ALU(input wire [7:0] A,
    input wire [7:0] B,
    input wire [3:0] FunSel,
    output wire [7:0] OutALU,
    output wire [3:0] zcno,
    input wire Clk);
    // Decode FunSel
    wire [15:0] inst;
    dec_4to16 instruction_decoder(.I(FunSel), .O(inst));
    // A B complement
    wire [7:0] A_neg;
    wire [7:0] B_neg;
    not_8bit a_neg(.A(A), .B(A_neg));
    not_8bit b_neg(.A(B), .B(B_neg));
    // Adder
    wire [7:0] RPAout, RPA_A, RPA_B;
    mux_4to1_8bit rpa_a(.o(RPA_A), .s(FunSel[1:0]), 
    .i0(A), .i1(A), .i2(A));
    mux_4to1_8bit rpa_b(.o(RPA_B), .s(FunSel[1:0]), 
    .i0(B), .i1(B_neg), .i2(B_neg));
    wire cin;
    wire cout;
    assign cin = (inst[5]) ;
    RPA_8bit adder(.Out(RPAout), .A(RPA_A), .B(RPA_B),
    .Cin(cin), .Cout(cout));
    comparator comp(
    .a(A),.b(B),.out(Comp)
);
    // Logic Operations
    wire [7:0] AandB, AorB, AxorB; 
    and_8bit a_and_b(.A(A), .B(B), .C(AandB));
    or_8bit a_or_b(.A(A), .B(B), .C(AorB));
    xor_8bit a_xor_b(.A(A), .B(B), .C(AxorB));
    nand_8bit a_nand_b(.A(A), .B(B), .C(AnandB));
    // Zero Flag
    wire zc;
    zero_check zero_flag(.A(OutALU), .B(zc));
    reg_1bit Z(.clk(Clk), .Q(zcno[3]),
    .enable(1), .D(zc));
    // Carry Flag
    wire carry_flag_enable;
    wire [15:0] cfo;
    assign cfo[4] = inst[4], cfo[5] = inst[5], cfo[6] = inst[6],
    cfo[11] = inst[11], cfo[12] = inst[12], cfo[15] = inst[15];
    dec_16bit_or carry_en(.A(cfo), .B(carry_flag_enable));
    wire carry_flag_input;
    wire [15:0] carry_flag_input_mux;
    assign carry_flag_input_mux[4] = cout, 
    carry_flag_input_mux[5] = cout, carry_flag_input_mux[6] = cout,
    carry_flag_input_mux[10] = A[7], carry_flag_input_mux[11] = A[0], 
    carry_flag_input_mux[15] = A[0];
    mux_16to1_1bit carry_flag_input_MUX(.data_in(carry_flag_input_mux), 
    .data_out(carry_flag_input), .sel(FunSel));
    reg_1bit C(.clk(Clk), .Q(zcno[2]), .enable(carry_flag_enable), .D(carry_flag_input));
    // Negative Flag
    wire neg_flag_enable = !inst[14];    
    reg_1bit N(.clk(Clk), .Q(zcno[1]),
    .enable(neg_flag_enable), .D(OutALU[7]));  
    // Overflow Flag
    wire ls_overflow = (A[7] ^ A[6]) & (inst[13]);
    wire add_overflow = ((A[7] & B[7] & !RPAout[7]) | (!A[7] & !B[7] & RPAout[7]))
    & (inst[4]);
    wire sub_overflow = ((!A[7] & B[7] & RPAout[7]) | (A[7] & !B[7] & !RPAout[7]))
    & inst[5]; 
    wire comp_overflow = ((!A[7] & B[7] & RPAout[7]) | (A[7] & !B[7] & !RPAout[7]))
    & inst[6];   
    wire overflow_flag = ls_overflow | add_overflow | sub_overflow | comp_overflow;        
    wire ovf_enable;
    wire [15:0] ofe;
    assign ofe[4] = inst[4], ofe[5] = inst[5], ofe[6] = inst[6],
    ofe[13] = inst[13];
    dec_16bit_or ovf_en(.A(ofe), .B(ovf_enable));
    reg_1bit O(.D(overflow_flag), .enable(ovf_enable), .clk(Clk), .Q(zcno[0]));        
        // Logic and Circular Shifts
    wire [7:0] lsl;
    assign lsl[0] = 0, lsl[1] = A[0], lsl[2] = A[1], lsl[3] = A[2],
    lsl[4] = A[3], lsl[5] = A[4], lsl[6] = A[5], lsl[7] = A[6];
    wire [7:0] lsr;
    assign lsr[0] = A[1], lsr[1] = A[2], lsr[2] = A[3], lsr[3] = A[4],
    lsr[4] = A[5], lsr[5] = A[6], lsr[6] = A[7], lsr[7] = 0;
    wire [7:0] csr;
    assign csr[0] = A[1], csr[1] = A[2], csr[2] = A[3], csr[3] = A[4],
    csr[4] = A[5], csr[5] = A[6], csr[6] = A[7], csr[7] = zcno[2];
    // Arithmetic Shift
    wire [7:0] asl;
    assign asl[7] = A[6], asl[6] = A[5], asl[5] = A[4], asl[4] = A[3],
    asl[3] = A[2], asl[2] = A[1], asl[1] = A[0], asl[0] = 0;
    wire [7:0] asr;
    assign asr[7]= A[7], asr[6]= A[7], asr[5]= A[6], asr[4]= A[5],
    asr[3]= A[4], asr[2]= A[3], asr[1]= A[2], asr[0]= A[1];
      // ALU MUX
    mux_16to1 ALU(.s(FunSel), .o(OutALU),
    .i0(A), .i1(B), .i2(A_neg), .i3(B_neg), .i4(RPAout),
    .i5(RPAout), .i6(Comp), .i7(AandB), .i8(AorB),
    .i9(AnandB), .i10(AxorB), .i11(lsl), .i12(lsr),
    .i13(asl), .i14(asr), .i15(csr));    
    
    
endmodule


module ALU_System(
    input wire [1:0] RF_OutASel,
    input wire [1:0] RF_OutBSel,
    input wire [1:0] RF_FunSel,
    input wire [3:0] RF_RSel,
    input wire [3:0] RF_TSel,
    input wire [3:0] ALU_FunSel,
    input wire [1:0] ARF_OutASel,
    input wire [1:0] ARF_OutBSel,
    input wire [1:0] ARF_FunSel,
    input wire [2:0] ARF_RegSel,
    input wire IR_LH,
    input wire IR_Enable,
    input wire [1:0] IR_Funsel,
    input wire Mem_WR,
    input wire Mem_CS,
    input wire [1:0] MuxASel,
    input wire [1:0] MuxBSel,
    input wire MuxCSel,
    input wire Clock,
    output wire [7:0] ALUOut,
    output wire [3:0] ALUOutFlag,
    output wire [15:0] IROut // Wire width is changed to 16 bits.
   
    );  
    // Mux Outputs
    wire [7:0] MuxAOut;
    wire [7:0] MuxBOut;
    wire [7:0] MuxCOut;    
    // IR
    wire [7:0] MemoryOut; 
    part_2a IR(.I(MemoryOut), .IRout(IROut), .Clk(Clock),
    .Enable(IR_Enable), .LH(IR_LH), .FunSel(IR_Funsel));
    // MUXB
    wire [7:0] AOut;
    mux_4to1_8bit MUXB(.i0(ALUOut),.i1(MemoryOut), .i2(IROut[7:0]), // Since IR wire width is changed, added the adequate width.
    .i3(AOut), .o(MuxBOut), .s(MuxBSel));
    // ARF
    wire [7:0] BOut;
    wire [7:0] Address;
    part_2c ARF(.clk(Clock), .o2(Address), .o1(AOut), .data_in(MuxBOut), .o1sel(ARF_OutASel),
    .o2sel(ARF_OutBSel), .regsel(ARF_RegSel), .funsel(ARF_FunSel));
    // Memory address is same with ARF_BOut.
    Memory MEM(.address(Address), .clock(Clock), .o(MemoryOut),
    .data(ALUOut), .wr(Mem_WR), .cs(Mem_CS));
    // MUXA
    mux_4to1_8bit MUXA(.i0(ALUOut), .i1(MemoryOut), .i2(IROut[7:0]), // Since IR wire width is changed, added the adequate width.
    .i3(AOut), .s(MuxASel), .o(MuxAOut));
    // RF
    wire [7:0] O1;
    wire [7:0] O2;
    part_2b RF(.o1(O1), .o2(O2), .data_in(MuxAOut), .clk(Clock),
    .funsel(RF_FunSel), .o1sel(RF_OutASel), .o2sel(RF_OutBSel), .regsel(RF_RSel), .tsel(RF_TSel)
    );
    // MUXC
    mux_2to1_8bit MUXC(.i0(O1), .i1(AOut), .s(MuxCSel), .o(MuxCOut));
    //ALU
    ALU ALU(.A(MuxCOut), .B(O2), .FunSel(ALU_FunSel), .Clk(Clock),
    .OutALU(ALUOut), .zcno(ALUOutFlag));    
endmodule
