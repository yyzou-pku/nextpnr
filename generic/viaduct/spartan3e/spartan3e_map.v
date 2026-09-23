// Yosys 内部 cell -> Spartan-3E 原语
module \$lut (A, Y);
	parameter WIDTH = 0;
	parameter LUT = 0;
	input [WIDTH-1:0] A;
	output Y;
	// WIDTH>5 需 F6/F7/F8（跳 slice），暂未实现；显式失败而不悄悄截断。
	wire _TECHMAP_FAIL_ = WIDTH > 8;
	// T7：WIDTH<=4 → 单个 4-LUT；WIDTH=5 → 宽 LUT（F/G 两个 4-LUT + F5 mux）。
	// （abc -lut 5 时才会出现 WIDTH=5。）
	generate
	if (WIDTH <= 4) begin
		localparam rep = 1<<(4-WIDTH);
		LUT4 #(.INIT({rep{LUT}})) _TECHMAP_REPLACE_ (.I(A[3:0]), .F(Y));
	end else if (WIDTH == 5) begin
		LUT5 #(.INIT(LUT[31:0])) _TECHMAP_REPLACE_ (.I(A[3:0]), .J(A[3:0]), .S(A[4]), .O(Y));
	end else if (WIDTH == 6) begin
		LUT6 #(.INIT(LUT[63:0])) _TECHMAP_REPLACE_ (.FI(A[3:0]), .GI(A[3:0]), .FJ(A[3:0]),
			.GJ(A[3:0]), .BX0(A[4]), .BX2(A[4]), .BY0(A[5]), .O(Y));
	end else if (WIDTH == 7) begin
		LUT7 #(.INIT(LUT[127:0])) _TECHMAP_REPLACE_ (
			.FI(A[3:0]), .GI(A[3:0]), .FJ(A[3:0]), .GJ(A[3:0]),
			.FK(A[3:0]), .GK(A[3:0]), .FL(A[3:0]), .GL(A[3:0]),
			.BXI(A[4]), .BXJ(A[4]), .BXK(A[4]), .BXL(A[4]),
			.BYI(A[5]), .BYJ(A[5]), .BYK(A[6]), .O(Y));
	end else begin
		LUT8 #(.INIT(LUT[255:0])) _TECHMAP_REPLACE_ (
			.FI(A[3:0]), .GI(A[3:0]), .FJ(A[3:0]), .GJ(A[3:0]),
			.FK(A[3:0]), .GK(A[3:0]), .FL(A[3:0]), .GL(A[3:0]),
			.BXI(A[4]), .BXJ(A[4]), .BXK(A[4]), .BXL(A[4]),
			.BYI(A[5]), .BYJ(A[5]), .BYK(A[6]),
			.MI(A[3:0]), .NI(A[3:0]), .MJ(A[3:0]), .NJ(A[3:0]),
			.MK(A[3:0]), .NK(A[3:0]), .ML(A[3:0]), .NL(A[3:0]),
			.BXM(A[4]), .BXN(A[4]), .BXO(A[4]), .BXP(A[4]),
			.BYM(A[5]), .BYN(A[5]), .BYO(A[6]), .BYL(A[7]), .O(Y));
	end
	endgenerate
endmodule

module  \$_DFF_P_    (input D, C, output Q); DFF   _TECHMAP_REPLACE_ (.D(D), .CLK(C), .Q(Q)); endmodule
module  \$_DFFE_PP_  (input D, C, E, output Q); DFFE  _TECHMAP_REPLACE_ (.D(D), .CLK(C), .CE(E), .Q(Q)); endmodule
module  \$_SDFF_PP0_ (input D, C, R, output Q); DFFR  _TECHMAP_REPLACE_ (.D(D), .CLK(C), .SR(R), .Q(Q)); endmodule
module  \$_SDFF_PP1_ (input D, C, R, output Q); DFFS  _TECHMAP_REPLACE_ (.D(D), .CLK(C), .SR(R), .Q(Q)); endmodule
module  \$_SDFFE_PP0P_ (input D, C, R, E, output Q); DFFRE _TECHMAP_REPLACE_ (.D(D), .CLK(C), .SR(R), .CE(E), .Q(Q)); endmodule
module  \$_SDFFE_PP1P_ (input D, C, R, E, output Q); DFFSE _TECHMAP_REPLACE_ (.D(D), .CLK(C), .SR(R), .CE(E), .Q(Q)); endmodule
module  \$_DFF_PP0_  (input D, C, R, output Q); DFFC  _TECHMAP_REPLACE_ (.D(D), .CLK(C), .SR(R), .Q(Q)); endmodule
module  \$_DFF_PP1_  (input D, C, R, output Q); DFFP  _TECHMAP_REPLACE_ (.D(D), .CLK(C), .SR(R), .Q(Q)); endmodule
module  \$_DFFE_PP0P_ (input D, C, R, E, output Q); DFFCE _TECHMAP_REPLACE_ (.D(D), .CLK(C), .SR(R), .CE(E), .Q(Q)); endmodule
module  \$_DFFE_PP1P_ (input D, C, R, E, output Q); DFFPE _TECHMAP_REPLACE_ (.D(D), .CLK(C), .SR(R), .CE(E), .Q(Q)); endmodule

// 三态：**故意不映射 `$_TBUF_`**。
// 它应由 `iopadmap -toutpad OBUFT ~T:I:PAD` / `-tinoutpad IOBUF ~T:O:I:PAD` 合并到焊盘
// 单元里（`~T` = 负极性使能，因为 bel 的 `T` 是高有效三态）。
// 若在这里映射，`~E` 会在 `abc` 之后生成、再也变不成 LUT，而且会与 iopadmap 的取反叠加。
// 万一有 `$_TBUF_` 漏到 nextpnr，会明确报「没有 bel 能实现该 cell」，而不是悄悄极性出错。

// IBUFG → INBUF：uarch 只认 INBUF（IOB bel 的 `PAD`/`O`）。
// 这样 IBUFG 可放在任意焊盘，时钟经 fabric 到 BUFG（不做专用时钟路径）。
module IBUFG (output O, input I);
    INBUF _TECHMAP_REPLACE_ (.PAD(I), .O(O));
endmodule
