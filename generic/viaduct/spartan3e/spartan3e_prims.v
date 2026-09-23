// Spartan-3E 原语（供 Yosys 的 techmap / 黑盒声明使用）
// LUT 输出端口名沿用 bel 的引脚名 `F`（对应 chipdb 的 LUT4 bel）。

module LUT4 #(parameter [15:0] INIT = 0) (input [3:0] I, output F);
    wire [7:0] s3 = I[3] ? INIT[15:8] : INIT[7:0];
    wire [3:0] s2 = I[2] ? s3[7:4]   : s3[3:0];
    wire [1:0] s1 = I[1] ? s2[3:2]   : s2[1:0];
    assign F = I[0] ? s1[1] : s1[0];
endmodule

// T7：5 输入宽 LUT（用 slice 内的 F/G 两个 4-LUT + F5 宽 mux）。
//   F5 = S ? F : G；S = BX。
//   F 用 `I[3:0]`、G 用 `J[3:0]`（物理上两者必须接同一组网络，见 spartan3e_map.v），
//   INIT[31:16] 是 S=1 那一半（F），INIT[15:0] 是 S=0 那一半（G）。
module LUT5 #(parameter [31:0] INIT = 0)
    (input [3:0] I, input [3:0] J, input S, output O);
    wire [4:0] fi = {1'b1, I};
    wire [4:0] gi = {1'b0, J};
    assign O = S ? INIT[fi] : INIT[gi];
endmodule

// ---- FF：CE / 复位（SR）的组合 ----
// 同步复位类（FF_SR_SYNC = 1）：DFFR / DFFS / DFFRE / DFFSE
// 异步类（FF_SR_SYNC = 0，保持器件默认）：DFFC / DFFP
module DFF   (input CLK, D, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK) Q <= D;
endmodule
module DFFE  (input CLK, D, CE, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK) if (CE) Q <= D;
endmodule
module DFFR  (input CLK, D, SR, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK) if (SR) Q <= 1'b0; else Q <= D;
endmodule
module DFFS  (input CLK, D, SR, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK) if (SR) Q <= 1'b1; else Q <= D;
endmodule
module DFFRE (input CLK, D, SR, CE, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK) if (SR) Q <= 1'b0; else if (CE) Q <= D;
endmodule
module DFFSE (input CLK, D, SR, CE, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK) if (SR) Q <= 1'b1; else if (CE) Q <= D;
endmodule
module DFFC  (input CLK, D, SR, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK or posedge SR) if (SR) Q <= 1'b0; else Q <= D;
endmodule
module DFFP  (input CLK, D, SR, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK or posedge SR) if (SR) Q <= 1'b1; else Q <= D;
endmodule
// T8：异步复位/置位 + 时钟使能（对应 Yosys `$_DFFE_PP0P_`/`$_DFFE_PP1P_`）
module DFFCE (input CLK, D, CE, SR, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK or posedge SR) if (SR) Q <= 1'b0; else if (CE) Q <= D;
endmodule
module DFFPE (input CLK, D, CE, SR, output reg Q);
    initial Q = 1'b0;
    always @(posedge CLK or posedge SR) if (SR) Q <= 1'b1; else if (CE) Q <= D;
endmodule

module INBUF (input PAD, output O);
    assign O = PAD;
endmodule
module OUTBUF (output PAD, input I);
    assign PAD = I;
endmodule

// 三态。端口名与 bel 引脚一致：`I`=数据(IOI.O1)，`T`=三态控制(IOI.T1，高=高阻)，
// `O`=输入数据(IOI.I)，`PAD`=双向焊盘。
// 注意：三态的**配置位与普通 OBUF 完全相同**（MUX_O=O1 / MUX_T=T1，实测 ISE 位流），
// 区别只在于 T1 是否被布线驱动；不驱动即恒导通。
module OBUFT (output PAD, input I, input T);
    assign PAD = T ? 1'bz : I;
endmodule

module IOBUF (inout PAD, input I, input T, output O);
    assign PAD = T ? 1'bz : I;
    assign O = PAD;
endmodule

// 全局时钟缓冲。物理上是一个 BUFGMUX（I0/I1/S），`BUFG` 只用 I0。
// bel 的 `I` 绑到 `IMUX_BUFG_CLK[i]`、`O` 绑到 `GCLK_N/S[i]`；
// 之后由 chipdb 里已有的时钟树 pip（GCLK_N/S → GCLK_QUAD → GCLK → IMUX_CLK）到达 FF。
module BUFG (input I, output O);
    assign O = I;
endmodule

// 专用时钟输入缓冲。功能等同 INBUF；**不要求**焊盘是专用时钟脚
// （信号可经 fabric 进 BUFG）。端口名与 Xilinx 一致（`I`=焊盘, `O`）。
module IBUFG (output O, (* iopad_external_pin *) input I);
    assign O = I;
endmodule

// T7：6 输入宽 LUT = 两个 slice 的 F5 + F6（跨 slice 的 FX 宽 mux）。
//   FI/GI = 下面那个 slice 的 F/G 输入，FJ/GJ = 上面那个 slice（+2）的 F/G 输入；
//   BX0/BX2 = 两者的 BX（= I4），BY0 = 下面那个 slice 的 BY（= I5）。
//   INIT[63:48]=F0, [47:32]=G0, [31:16]=F2, [15:0]=G2。
module LUT6 #(parameter [63:0] INIT = 0)
    (input [3:0] FI, input [3:0] GI, input [3:0] FJ, input [3:0] GJ,
     input BX0, BX2, BY0, output O);
    wire f0 = INIT[{2'b11, FI}];
    wire g0 = INIT[{2'b10, GI}];
    wire f2 = INIT[{2'b01, FJ}];
    wire g2 = INIT[{2'b00, GJ}];
    wire f5a = BX0 ? f0 : g0;
    wire f5b = BX2 ? f2 : g2;
    assign O = BY0 ? f5a : f5b;
endmodule

// T7：7 输入宽 LUT = 4 个 slice 的 F5（= 4 个 F6 的输入）+ F7。
//   FI/GI=slice0, FJ/GJ=slice1, FK/GK=slice2, FL/GL=slice3（4 个低输入共用）；
//   BXI..BXL = 各自 BX（= I4）；BYI/BYJ = slice0/1 的 BY（= I5）；BYK = slice2 的 BY（= I6）。
//   输出到 slice2 的 Y（GYMUX=FX）。
module LUT7 #(parameter [127:0] INIT = 0)
    (input [3:0] FI, input [3:0] GI, input [3:0] FJ, input [3:0] GJ,
     input [3:0] FK, input [3:0] GK, input [3:0] FL, input [3:0] GL,
     input BXI, BXJ, BXK, BXL, BYI, BYJ, BYK, output O);
    wire f5i = BXI ? INIT[{3'b111, FI}] : INIT[{3'b110, GI}];
    wire f5k = BXK ? INIT[{3'b101, FK}] : INIT[{3'b100, GK}];
    wire f5j = BXJ ? INIT[{3'b011, FJ}] : INIT[{3'b010, GJ}];
    wire f5l = BXL ? INIT[{3'b001, FL}] : INIT[{3'b000, GL}];
    wire fx0 = BYI ? f5i : f5k;
    wire fx1 = BYJ ? f5j : f5l;
    assign O = BYK ? fx0 : fx1;
endmodule

// T7：8 输入宽 LUT = 两个 F7（本 CLB 与**上方** CLB 的 SLICE2.FX）+ F8。
//   FI..GL/BY* = 本 CLB 的 4 个 slice；MI..NL/BYM.. = 上方 CLB 的 4 个 slice；
//   BYL = 本 CLB slice3 的 BY（= I7）。输出到本 CLB slice3 的 Y（GYMUX=FX）。
//   INIT[255:128] = 本 CLB 的 F7（I7=1），INIT[127:0] = 上方 CLB 的 F7（I7=0）。
module LUT8 #(parameter [255:0] INIT = 0)
    (input [3:0] FI, input [3:0] GI, input [3:0] FJ, input [3:0] GJ,
     input [3:0] FK, input [3:0] GK, input [3:0] FL, input [3:0] GL,
     input BXI, BXJ, BXK, BXL, BYI, BYJ, BYK,
     input [3:0] MI, input [3:0] NI, input [3:0] MJ, input [3:0] NJ,
     input [3:0] MK, input [3:0] NK, input [3:0] ML, input [3:0] NL,
     input BXM, BXN, BXO, BXP, BYM, BYN, BYO, BYL, output O);
    wire af5i = BXI ? INIT[{1'b1, 3'b111, FI}] : INIT[{1'b1, 3'b110, GI}];
    wire af5k = BXK ? INIT[{1'b1, 3'b101, FK}] : INIT[{1'b1, 3'b100, GK}];
    wire af5j = BXJ ? INIT[{1'b1, 3'b011, FJ}] : INIT[{1'b1, 3'b010, GJ}];
    wire af5l = BXL ? INIT[{1'b1, 3'b001, FL}] : INIT[{1'b1, 3'b000, GL}];
    wire afx0 = BYI ? af5i : af5k;
    wire afx1 = BYJ ? af5j : af5l;
    wire afx2 = BYK ? afx0 : afx1;
    wire bf5i = BXM ? INIT[{1'b0, 3'b111, MI}] : INIT[{1'b0, 3'b110, NI}];
    wire bf5k = BXO ? INIT[{1'b0, 3'b101, MK}] : INIT[{1'b0, 3'b100, NK}];
    wire bf5j = BXN ? INIT[{1'b0, 3'b011, MJ}] : INIT[{1'b0, 3'b010, NJ}];
    wire bf5l = BXP ? INIT[{1'b0, 3'b001, ML}] : INIT[{1'b0, 3'b000, NL}];
    wire bfx0 = BYM ? bf5i : bf5k;
    wire bfx1 = BYN ? bf5j : bf5l;
    wire bfx2 = BYO ? bfx0 : bfx1;
    assign O = BYL ? afx2 : bfx2;
endmodule
