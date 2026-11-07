// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
//============================================================================
//
//  Toobin' (Atari Games, 1988) MiSTer FPGA core — top-level glue (emu).
//
//  Wires toobin_core (68010 + JSA sound + SDRAM datapath + ce_pix video) to the
//  MiSTer framework: PLL clocks, hps_io (ROM download + controls), arcade_video
//  + screen_rotate (ROT270 vertical monitor), audio, and the SDRAM chip pins.
//  See rtl/toobin_core.sv.
//
//  Single clk_sys domain + clock enables (the accurate model — the real main
//  board derives 68010/8 MHz and pixel/16 MHz from one master by division).
//
//  The PLL outputs clk_sys = 64 MHz (SDRAM headroom: the tile prefetch does ~192
//  SDRAM reads/line; 32 MHz is bandwidth-marginal) and clk_sdram = 64 MHz phase-
//  shifted for the SDRAM_CLK pin.  The core dividers below (PIX_DIV=4 -> 16 MHz
//  pixel, CPU_DIV=8 -> 8 MHz CPU) and SDRAM_CLK_MHZ=64 assume that; regenerate
//  pll.qip if you change them.
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
//  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
//
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_OSD + per-pin push-pull mask, USER_IO widened to 8 bits
	output        USER_OSD,
	output  [7:0] USER_PP,
	input   [7:0] USER_IN,
	output  [7:0] USER_OUT,
	// [MiSTer-DB9 END]

	input         OSD_STATUS
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_PP driven by wrapper; USER_OUT driven by joydb (USER_OUT_DRIVE) below
assign USER_PP = USER_PP_DRIVE;
// [MiSTer-DB9 END]
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1        = 0;
assign VGA_SCALER    = 0;
assign VGA_DISABLE   = 0;
assign HDMI_FREEZE   = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT= 0;

assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

//////////////////////////////////////////////////////////////////

// Toobin' runs on a VERTICAL monitor (ROT270).  screen_rotate handles the display
// rotation via the DDR framebuffer.  The advertised "Original" aspect has to follow
// the Orientation option: rotated presents a 3:4 portrait frame, but the passthrough
// states present the native 4:3 landscape raster, and advertising 3:4 for those would
// squash it.  See the orientation block further down for the state encoding.
// VIDEO_ARX/ARY are driven by video_freak (down in the video section) so the
// Scale option can force integer scaling; these are the requested ratios it
// starts from.
wire [1:0] ar = status[122:121];
wire       ar_rotated = (status[5:4] == 2'd0) & ~direct_video;
wire [11:0] arx = (!ar) ? (ar_rotated ? 12'd3 : 12'd4) : 12'(ar - 1'd1);
wire [11:0] ary = (!ar) ? (ar_rotated ? 12'd4 : 12'd3) : 12'd0;

`include "build_id.v"
localparam CONF_STR = {
	"Toobin;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[5:4],Orientation,Vert,No Rotate,No Rotate 180;",
	"O[22:20],Scale,Normal,V-Integer,HV-Integer,Narrower HV-Integer;",
	"-;",
	// CRT Adjust: rmonic79's MiSTer-CRT-Adjust engine (rtl/video/crt_adjust.sv)
	// behind the glue in rtl/video/toobin_analog_adjust.sv.  Three amounts plus
	// an On/Off.  H-Position is built as a content shift; the reasoning for this
	// raster, and its +32 right-hand limit, is in toobin_analog_adjust.sv.
	//
	// Every amount is plain two's complement, so the list is 0,+1..+max then
	// -min..-1 and the decode is a $signed().  None of these touch the core's
	// timing; the stage sits between toobin_core and arcade_video.  Off is a
	// bit-exact bypass, so HDMI is untouched unless the user turns it On.
	//
	// H1 hides the three amounts while the stage is inactive, which includes the
	// case where the framework has forced the scandoubler and the stage gates
	// itself off (see `crt_enable`).
	//
	// Bits 47..65 are fresh on purpose: the previous analog stage used 23..46
	// (H-Size, H-Position, VGA H-Shift, VGA V-Shift), and reusing them would
	// read an old saved H-Size as an On/Off.  23..46 are left unused.
	"P1,CRT Adjust;",
	"P1-;",
	"P1O[47],CRT Adjust,Off,On;",
	"H1P1O[52:48],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[59:53],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,+49,+50,+51,+52,+53,+54,+55,+56,+57,+58,+59,+60,+61,+62,+63,-64,-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-53,-52,-51,-50,-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[65:60],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	// No DIP page: Toobin' has no dip switches.  MAME's INPUT_PORTS_START(toobin) has only
	// the FF8800 controls and the FF9000 service/status word; every operator setting lives
	// in the X2804 EEPROM and is changed from the game's own self-test.  A "DIP;" line here
	// would ask MiSTer Main to splice in a menu the MRA can never populate.
	"O[6],Service,Off,On;",
	"-;",
	// [MiSTer-DB9-Pro BEGIN] - Saturn-first joy_type (canonical bit notation)
	"O[127:126],UserIO Joystick,Off,Saturn,DB9MD,DB15;",
	"O[125],UserIO Players, 1 Player,2 Players;",
	// [MiSTer-DB9-Pro END]
	"T[0],Reset;",
	"J1,Left Back,Right Back,Left Forward,Right Forward,Throw Can/Start,Coin;",
	"jn,A,B,X,Y,L,R;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
// CRT Adjust master enable.  Declared here because status_menumask below reads
// it.  The read rate assumes the native 15 kHz pixel ratio, so the stage turns
// itself off whenever the framework forces the scandoubler (upstream's rule),
// and the H1 menu entries hide while it is off.
wire crt_enable = status[47] & ~forced_scandoubler;

wire [10:0] ps2_key;
wire        direct_video;
wire [21:0] gamma_bus;

wire        ioctl_download;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] ioctl_index;
wire        ioctl_wait;
wire        ioctl_upload, ioctl_upload_req;
wire  [7:0] ioctl_upload_index, ioctl_din;

wire [31:0] joystick_0, joystick_1;

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper
wire         CLK_JOY = CLK_50M;                 // Assign clock between 40-50Mhz
wire   [1:0] joy_type_raw    = status[127:126]; // 0=Off, 1=Saturn, 2=DB9MD, 3=DB15
wire         joy_2p          = status[125];
// SNAC cores: replace 1'b0 with the core's SNAC enable expression so SNAC
// preempts the joydb wrapper on shared USER_IO pins. Default 1'b0 is no-op.
wire         snac_active     = 1'b0;
// MT32-pi cores on primary USER_IO: replace 1'b0 with the core's MT32-active
// expression (e.g. `mt32_use` under `ifndef SECOND_MT32`, `~mt32_disable` for
// TRS-80's inverted polarity). Suppresses the OSD-open autodetect probe so it
// doesn't read the RPi's I2C master traffic as a ghost Saturn signature.
wire         mt32_primary_active = 1'b0;
wire   [1:0] joy_type        = snac_active ? 2'd0 : joy_type_raw;
wire         joy_db9md_en    = (joy_type == 2'd2);
wire         joy_db15_en     = (joy_type == 2'd3);
wire         joy_any_en      = |joy_type;
// [MiSTer-DB9 END]

// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
wire         saturn_unlocked;                   // driven by hps_io UIO_DB9_KEY (0xFE)
// [MiSTer-DB9-Pro END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper wires + instance
wire   [7:0] USER_OUT_DRIVE;
wire   [7:0] USER_PP_DRIVE;
wire  [15:0] joydb_1, joydb_2;
wire         joydb_1ena, joydb_2ena;
wire  [15:0] joy_raw_payload;
// Programmable-remap matrix (joydb_remap inside joydb): clk_sys carries the
// 0xFD selector load (HPS-bus domain). joydb_*_mapped are the MiSTer-standard
// joystick words the core consumes at its joy0_USB merge point. db9_remap_*
// are driven by the hps_io instance (see the .db9_remap_* bindings there).
// Until a core consumes joydb_*_mapped (Layer B), the matrix stays at identity
// and synthesis prunes it -- binding these is dormant + fleet-safe.
wire  [15:0] joydb_1_mapped, joydb_2_mapped;
wire         db9_remap_cmd;
wire   [5:0] db9_remap_byte_cnt;
wire  [15:0] db9_remap_din;

// [MiSTer-DB9 BEGIN] - DB9 remap factory default (used until Main_MiSTer streams UIO 0xFD)
// Derived from CONF_STR J1, same rule as db9_map.cpp; lets the core work on a stock MiSTer binary.
// DB15:  Left Back=A, Right Back=B, Left Forward=C, Right Forward=D, Throw Can/Start=E, Coin=SELECT
// DB9MD: Left Back=A, Right Back=B, Left Forward=C, Right Forward=X, Throw Can/Start=Y, Coin=MODE
wire  [35:0] db9_remap_default_db15  = 36'hFFFB87654;
wire  [35:0] db9_remap_default_db9md = 36'hFFFB87654;
// [MiSTer-DB9 END]
joydb joydb (
  .clk             ( CLK_JOY         ),
  .clk_sys         ( clk_sys            ),
  .USER_IN         ( USER_IN         ),
  .OSD_STATUS          ( OSD_STATUS          ),
  .snac_active         ( snac_active         ),
  .mt32_primary_active ( mt32_primary_active ),
  .joy_type        ( joy_type        ),
  .joy_2p          ( joy_2p          ),
  .saturn_unlocked ( saturn_unlocked ),
  .USER_OUT_DRIVE  ( USER_OUT_DRIVE  ),
  .USER_PP_DRIVE   ( USER_PP_DRIVE   ),
  .USER_OSD        ( USER_OSD        ),
  .joydb_1         ( joydb_1         ),
  .joydb_2         ( joydb_2         ),
  .joydb_1ena      ( joydb_1ena      ),
  .joydb_2ena      ( joydb_2ena      ),
  .remap_cmd       ( db9_remap_cmd      ),
  .remap_byte_cnt  ( db9_remap_byte_cnt ),
  .remap_din       ( db9_remap_din      ),
  .remap_default_db15  ( db9_remap_default_db15  ),
  .remap_default_db9md ( db9_remap_default_db9md ),
  .joydb_1_mapped  ( joydb_1_mapped     ),
  .joydb_2_mapped  ( joydb_2_mapped     ),
  .joy_raw         ( joy_raw_payload )
);

assign USER_OUT = USER_OUT_DRIVE;
// [MiSTer-DB9 END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: rename USB joystick wires
wire [31:0] joystick_0_USB, joystick_1_USB;
// [MiSTer-DB9 END]

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),
	.direct_video(direct_video),

	.buttons(buttons),
	.status(status),
	.status_menumask({14'd0, ~crt_enable, 1'b0}),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index),
	.ioctl_din(ioctl_din),

	.joystick_0(joystick_0_USB),
	.joystick_1(joystick_1_USB),

	.ps2_key(ps2_key),
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joy_raw
	.joy_raw(joy_raw_payload),
	// programmable remap matrix selector load (UIO_DB9_MAP 0xFD)
	.db9_remap_cmd(db9_remap_cmd),
	.db9_remap_byte_cnt(db9_remap_byte_cnt),
	.db9_remap_din(db9_remap_din),
	// [MiSTer-DB9 END]
	// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
	.saturn_unlocked(saturn_unlocked)
	// [MiSTer-DB9-Pro END]
);

// [MiSTer-DB9-Pro BEGIN] - DB controllers muted while OSD is open
// joydb_*_mapped is already in MiSTer-standard J1 order, identical to the USB
// joystick word, so it replaces the USB input as-is.
assign joystick_0 = joydb_1ena ? (OSD_STATUS ? 16'b0 : joydb_1_mapped) : joystick_0_USB;
assign joystick_1 = joydb_2ena ? (OSD_STATUS ? 16'b0 : joydb_2_mapped) : joydb_1ena ? joystick_0_USB : joystick_1_USB;
// [MiSTer-DB9-Pro END]

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;               // 64 MHz game clock (see PLL note above)
wire clk_sdram;             // 64 MHz phase-shifted for the SDRAM_CLK pin
wire clk_20_unused;
wire pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.rst(1'b0),
	.outclk_0(clk_sys),
	.outclk_1(clk_20_unused),
	.outclk_2(clk_sdram),
	.locked(pll_locked)
);

wire reset = RESET | status[0] | buttons[1] | ~pll_locked;

///////////////////////   CONTROLS   /////////////////////////////

// Five direct cabinet buttons per player. Throw Can is also the game's Start:
// [4]=Left Back [5]=Right Back [6]=Left Forward [7]=Right Forward
// [8]=Throw Can/Start [9]=Coin.  D-pad bits [3:0] are intentionally unused.
wire [15:0] sw_ff8800;
toobin_inputs u_inputs (
	.p1_l_back(joystick_0[4]), .p1_r_back(joystick_0[5]),
	.p1_l_fwd(joystick_0[6]),  .p1_r_fwd(joystick_0[7]),
	.p1_throw_start(joystick_0[8]),
	.p2_l_back(joystick_1[4]), .p2_r_back(joystick_1[5]),
	.p2_l_fwd(joystick_1[6]),  .p2_r_fwd(joystick_1[7]),
	.p2_throw_start(joystick_1[8]),
	.switches(sw_ff8800) );

// Separate coin chutes: the cabinet has two, entering on the JSA's
// /RDIO D1:D0 -- player 1's Coin button = chute 1, player 2's = chute 2.
wire coin1     = joystick_0[9];
wire coin2     = joystick_1[9];
wire self_test = status[6];

///////////////////////   CORE   /////////////////////////////////

wire        ce_pix;
wire  [7:0] core_r, core_g, core_b;
wire        core_hs, core_vs, core_hb, core_vb;
wire signed [15:0] aud_l, aud_r;

toobin_core #(.SDRAM_CLK_MHZ(64), .PIX_DIV(4), .CPU_DIV(8)) u_core
(
	.clk_sys(clk_sys), .reset(reset), .init_reset(~pll_locked),

	.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout), .ioctl_index(ioctl_index), .ioctl_wait(ioctl_wait),
	.ioctl_upload(ioctl_upload), .ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index), .ioctl_din(ioctl_din),

	.sw_ff8800(sw_ff8800), .self_test(self_test), .coin1(coin1), .coin2(coin2),

	.ce_pix(ce_pix), .vga_r(core_r), .vga_g(core_g), .vga_b(core_b),
	.hsync(core_hs), .vsync(core_vs), .hblank(core_hb), .vblank(core_vb),

	.aud_l(aud_l), .aud_r(aud_r),

	.SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH), .SDRAM_CLK(), .SDRAM_CKE(SDRAM_CKE), .SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE)
);

// SDRAM clock pin from the phase-shifted PLL output (not the controller's ~clk)
assign SDRAM_CLK = clk_sdram;

///////////////////////   AUDIO   ////////////////////////////////

assign AUDIO_S = 1'b1;              // signed samples
assign AUDIO_L = aud_l;
assign AUDIO_R = aud_r;

///////////////////////   VIDEO   ////////////////////////////////

wire [7:0] R, G, B;
wire       HSync, VSync, HBlank, VBlank;
assign R = core_r;
assign G = core_g;
assign B = core_b;
assign HSync  = core_hs;
assign VSync  = core_vs;
assign HBlank = core_hb;
assign VBlank = core_vb;

// The cabinet raster is always ROT270 and the release core presents its native
// pixels without an OSD-selectable scaler effect. MiSTer's forced scandoubler
// signal remains framework-controlled for displays that require it.
wire [2:0] fx = 3'b000;

// Orientation (status[5:4]).  screen_rotate exposes exactly THREE usable states, not
// four: it derives `do_flip = no_rotate && flip` and `fb_en = ~no_rotate | flip`
// (sys/arcade_video.v:233,288), so rotating and flipping are mutually exclusive --
// `flip` means "180 degrees INSTEAD of rotating" and is dead while the core rotates.
// (An earlier "Vert/Flip" option drove `flip` with `no_rotate` tied to 0, so it was
// ANDed with a constant zero and could never do anything.)
//   0 Vert         : rotate CCW into the DDR framebuffer -- a normal horizontal monitor
//   1 No Rotate    : straight passthrough  -- a physically rotated (TATE) monitor
//   2 No Rotate 180: passthrough flipped 180 -- monitor rotated the other way
// direct_video bypasses the scaler/framebuffer entirely, so it forces the passthrough
// state; this is the standard MiSTer arcade idiom (`no_rotate = <opt> | direct_video`).
wire [1:0] orient     = status[5:4];
wire       no_rotate  = (orient != 2'd0) | direct_video;
wire       rotate_ccw = 1'b1;                       // ROT270
wire       flip       = (orient == 2'd2) & ~direct_video;
wire       video_rotated;

assign CLK_VIDEO = clk_sys;

// CRT Adjust (H-Size / H-Position / V-Shift, see toobin_analog_adjust.sv).
// Placed ahead of arcade_video so the mixer, scandoubler and video_freak all
// see the adjusted raster; the core's own timing is untouched.  While On the
// pixel rate is the stage's read rate, not ce_pix, which is why arcade_video is
// clocked from adj_ce rather than ce_pix.
wire [7:0] adj_r, adj_g, adj_b;
wire       adj_hs, adj_vs, adj_hb, adj_vb, adj_ce;

toobin_analog_adjust #(.H_TOTAL(640), .V_TOTAL(416), .CLK_PER_PIX(4), .HPOS_MODE(1), .AW(11)) u_analog_adjust
(
	.clk        (clk_sys),
	.ce_pix     (ce_pix),
	.osd_on     (crt_enable),
	.osd_hsize  (status[52:48]),
	.osd_hpos   (status[59:53]),
	.osd_vshift (status[65:60]),
	.r_in (R), .g_in (G), .b_in (B),
	.hs_in(HSync), .vs_in(VSync), .hb_in(HBlank), .vb_in(VBlank),
	.r_out(adj_r), .g_out(adj_g), .b_out(adj_b),
	.hs_out(adj_hs), .vs_out(adj_vs), .hb_out(adj_hb), .vb_out(adj_vb),
	.ce_out(adj_ce)
);

wire vga_de_raw;

arcade_video #(.WIDTH(512), .DW(24)) arcade_video
(
	.*,
	.clk_video(clk_sys),
	.ce_pix(adj_ce),
	.RGB_in({adj_r, adj_g, adj_b}),
	.HBlank(adj_hb),
	.VBlank(adj_vb),
	.HSync(adj_hs),
	.VSync(adj_vs),
	.VGA_DE(vga_de_raw),
	.fx(fx)
);

// Scale (status[22:20]) maps onto video_freak's SCALE encoding, which is
// documented at sys/video_freak.sv:32 as
//   0 normal, 1 V-integer, 2 HV-Integer-, 3 HV-Integer+, 4 HV-Integer.
// "Narrower HV-Integer" is therefore 2, and plain "HV-Integer" is 4 -- they are
// not adjacent, so the menu order and the encoding deliberately differ.
wire [2:0] scale_sel = (status[22:20] == 3'd0) ? 3'd0 :   // Normal
                       (status[22:20] == 3'd1) ? 3'd1 :   // V-Integer
                       (status[22:20] == 3'd2) ? 3'd4 :   // HV-Integer
                                                 3'd2;    // Narrower HV-Integer

video_freak video_freak
(
	.CLK_VIDEO  (CLK_VIDEO),
	.CE_PIXEL   (CE_PIXEL),
	.VGA_VS     (VGA_VS),
	.HDMI_WIDTH (HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE     (VGA_DE),
	.VIDEO_ARX  (VIDEO_ARX),
	.VIDEO_ARY  (VIDEO_ARY),
	.VGA_DE_IN  (vga_de_raw),
	.ARX        (arx),
	.ARY        (ary),
	.CROP_SIZE  (12'd0),
	.CROP_OFF   (5'd0),
	.SCALE      (scale_sel)
);

screen_rotate screen_rotate (.*);

///////////////////////   STATUS LED   ///////////////////////////

assign LED_USER = ioctl_download;

endmodule
