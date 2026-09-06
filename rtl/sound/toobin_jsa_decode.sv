// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2026 RetroShrimp. GPL v2 or later.
//
// Toobin' JSA-I sound-CPU (6502) address decoder.
//
// Faithful to MAME atarijsa.cpp atarijsa1_map (JSA_MASTER_CLOCK 3.579545 MHz;
// 6502 @ /2, YM2151 @ x1, POKEY @ /2), checked against the real decoder PAL:
// Atari 136056-2101, the "2L-PAL" (PAL16L8A) on SP-320 sheet 20, named as such in
// the Toobin' audio-PCB parts list (decode the JEDEC with `jedutil -view ...
// gal16v8`).  Toobin is JSA-I = YM2151 + POKEY, with the
// TMS5220 depopulated, so /VOICE (0x2A00) is a no-op here.
//
// The PAL agrees with MAME's map everywhere except one mirror: its /YAM output is
//   /YAM = A13 & /A11 & /A12 & /A14 & /A15
// i.e. the YM2151 answers across 0x2000-0x27FF (mirror 0x07FE), not just
// 0x2000-0x2001.  (The PAL also makes /IRQACK read-only via the LS138 enable; that
// lives in toobin_jsa_bus.sv where R/W is known.)
//
//   0x0000-0x1FFF  RAM (8 KB)
//   0x2000-0x27FF  YM2151 (reg select = A0)  (mirror 0x07FE, per the PAL)
//   0x2800         N/C                        (mirror 0x01F9)
//   0x2802         /RDP  sound_command_r      (mirror 0x01F9)
//   0x2804         /RDIO read inputs          (mirror 0x01F9)
//   0x2806         /IRQACK r/w                (mirror 0x01F9)
//   0x2A00         /VOICE (TMS5220 — N/C)     (mirror 0x01F9)
//   0x2A02         /WRP  sound_response_w     (mirror 0x01F9)
//   0x2A04         /WRIO output latch         (mirror 0x01F9)
//   0x2A06         /MIX  mixing levels        (mirror 0x01F9)
//   0x2C00-0x2C0F  POKEY (reg = A3..A0)       (mirror 0x03F0)
//   0x3000-0x3FFF  banked ROM ("cpubank")
//   0x4000-0xFFFF  fixed ROM
//
// MAME mirror semantics: a register at base B with mirror M matches when
// (addr & ~M) == B, so the "care" mask is ~M.

module toobin_jsa_decode
(
	input  logic [15:0] addr,
	output logic        sel_ram,
	output logic        sel_ym,
	output logic        sel_rdp,
	output logic        sel_rdio,
	output logic        sel_irqack,
	output logic        sel_voice,   // TMS5220 slot — unused on Toobin
	output logic        sel_wrp,
	output logic        sel_wrio,
	output logic        sel_mix,
	output logic        sel_pokey,
	output logic        sel_bank,
	output logic        sel_rom,
	output logic        ym_reg,      // YM2151 register select (A0)
	output logic  [3:0] pokey_reg    // POKEY register (A3..A0)
);

	localparam logic [15:0] M28 = 16'hFE06;   // ~0x01F9 : 0x28xx/0x2Axx block care mask
	localparam logic [15:0] MPK = 16'hFC00;   // ~(0x03F0 mirror | 0x000F reg) : POKEY care mask

	assign sel_ram    = (addr[15:13] == 3'b000);              // 0x0000-0x1FFF
	assign sel_ym     = (addr[15:11] == 5'b00100);            // 0x2000-0x27FF (PAL /YAM)
	assign sel_rdp    = (addr & M28) == 16'h2802;
	assign sel_rdio   = (addr & M28) == 16'h2804;
	assign sel_irqack = (addr & M28) == 16'h2806;
	assign sel_voice  = (addr & M28) == 16'h2A00;
	assign sel_wrp    = (addr & M28) == 16'h2A02;
	assign sel_wrio   = (addr & M28) == 16'h2A04;
	assign sel_mix    = (addr & M28) == 16'h2A06;
	assign sel_pokey  = (addr & MPK) == 16'h2C00;            // 0x2C00-0x2C0F (+mirror)
	assign sel_bank   = (addr[15:12] == 4'h3);               // 0x3000-0x3FFF
	assign sel_rom    = (addr[15:14] != 2'b00);              // 0x4000-0xFFFF

	assign ym_reg    = addr[0];
	assign pokey_reg = addr[3:0];

endmodule
