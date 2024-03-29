//! Module for disassembling opcodes into human readable representations

use crate::mmu::Memory;

fn invalid_inst(op: u8) -> String {
    format!("0x{:2X}", op)
}

pub fn disasm(mut addr: u16, mem: &Memory) -> String {
    macro_rules! get_n { () => ({ addr += 1; mem.lb(addr - 1) }) }
    macro_rules! get_ni { () => (get_n!() as i8) }
    macro_rules! get_nn { () => (get_n!() as u16 + ((get_n!() as u16) << 8)) }

    let op = mem.lb(addr);
    addr += 1;

    match op {
        0x00 => format!("nop"),
        0x01 => format!("ld\t\tbc, {}", get_nn!()),
        0x02 => format!("ld\t\t(bc), a"),
        0x03 => format!("inc\t\tbc"),
        0x04 => format!("inc\t\tb"),
        0x05 => format!("dec\t\tb"),
        0x06 => format!("ld\t\tb, {}", get_n!()),
        0x07 => format!("rlca"),
        0x08 => format!("ld\t\t({:X}h), sp", get_nn!()),
        0x09 => format!("add\t\thl, bc"),
        0x0A => format!("ld\t\ta, (bc)"),
        0x0B => format!("dec\t\tbc"),
        0x0C => format!("inc\t\tc"),
        0x0D => format!("dec\t\tc"),
        0x0E => format!("ld\t\tc, {}", get_n!()),
        0x0F => format!("rrca"),

        0x10 => format!("stop"),
        0x11 => format!("ld\t\tde, {}", get_nn!()),
        0x12 => format!("ld\t\t(de), a"),
        0x13 => format!("inc\t\tde"),
        0x14 => format!("inc\t\td"),
        0x15 => format!("dec\t\td"),
        0x16 => format!("ld\t\td, {}", get_n!()),
        0x17 => format!("rla"),
        0x18 => format!("jr\t\t{}", get_ni!()),
        0x19 => format!("add\t\thl, de"),
        0x1A => format!("ld\t\ta, (de)"),
        0x1B => format!("dec\t\tde"),
        0x1C => format!("inc\t\te"),
        0x1D => format!("dec\t\te"),
        0x1E => format!("ld\t\te, {}", get_n!()),
        0x1F => format!("rra"),

        0x20 => format!("jr\t\tnz, {}", get_ni!()),
        0x21 => format!("ld\t\thl, {}", get_nn!()),
        0x22 => format!("ldi\t\t(hl), a"),
        0x23 => format!("inc\t\thl"),
        0x24 => format!("inc\t\th"),
        0x25 => format!("dec\t\th"),
        0x26 => format!("ldi\t\th, {}", get_n!()),
        0x27 => format!("daa"),
        0x28 => format!("jr\t\tz, {}", get_ni!()),
        0x29 => format!("add\t\thl, hl"),
        0x2A => format!("ldi\t\ta, (hl)"),
        0x2B => format!("dec\t\thl"),
        0x2C => format!("inc\t\tl"),
        0x2D => format!("dec\t\tl"),
        0x2E => format!("ld\t\tl, {}", get_n!()),
        0x2F => format!("cpl"),

        0x30 => format!("jr\t\tnc, {}", get_ni!()),
        0x31 => format!("ld\t\tsp, ({:X})", get_nn!()),
        0x32 => format!("ldd\t\t(hl), a"),
        0x33 => format!("inc\t\tsp"),
        0x34 => format!("inc\t\t(hl)"),
        0x35 => format!("dec\t\t(hl)"),
        0x36 => format!("ld\t\t(hl), {}", get_n!()),
        0x37 => format!("scf"),
        0x38 => format!("jr\t\tc, {}", get_ni!()),
        0x39 => format!("add\t\thl, sp"),
        0x3A => format!("ldd\t\ta, (hl)"),
        0x3B => format!("dec\t\tsp"),
        0x3C => format!("inc\t\ta"),
        0x3D => format!("dec\t\ta"),
        0x3E => format!("ld\t\ta, {}", get_n!()),
        0x3F => format!("ccf"),

        0x40 => format!("ld\t\tb, b"),
		0x41 => format!("ld\t\tb, c"),
        0x42 => format!("ld\t\tb, d"),
        0x43 => format!("ld\t\tb, e"),
        0x44 => format!("ld\t\tb, h"),
        0x45 => format!("ld\t\tb, l"),
        0x46 => format!("ld\t\tb, (hl)"),
        0x47 => format!("ld\t\tb, a"),
        0x48 => format!("ld\t\tc, b"),
        0x49 => format!("ld\t\tc, c"),
        0x4A => format!("ld\t\tc, d"),
        0x4B => format!("ld\t\tc, e"),
        0x4C => format!("ld\t\tc, h"),
        0x4D => format!("ld\t\tc, hl"),
        0x4E => format!("ld\t\tc, (hl)"),
        0x4F => format!("ld\t\tc, a"),

        0x50 => format!("ld\t\td, b"),
        0x51 => format!("ld\t\td, c"),
        0x52 => format!("ld\t\td, d"),
        0x53 => format!("ld\t\td, e"),
        0x54 => format!("ld\t\td, h"),
        0x55 => format!("ld\t\td, l"),
        0x56 => format!("ld\t\td, (hl)"),
        0x57 => format!("ld\t\td, a"),
        0x58 => format!("ld\t\te, b"),
        0x59 => format!("ld\t\te, c"),
        0x5A => format!("ld\t\te, d"),
        0x5B => format!("ld\t\te, e"),
        0x5C => format!("ld\t\te, h"),
        0x5D => format!("ld\t\te, l"),
        0x5E => format!("ld\t\te, (hl)"),
        0x5F => format!("ld\t\te, a"),

        0x60 => format!("ld\t\th, b"),
        0x61 => format!("ld\t\th, c"),
        0x62 => format!("ld\t\th, d"),
        0x63 => format!("ld\t\th, e"),
        0x64 => format!("ld\t\th, h"),
        0x65 => format!("ld\t\th, l"),
        0x66 => format!("ld\t\th, (hl)"),
        0x67 => format!("ld\t\th, a"),
        0x68 => format!("ld\t\tl, b"),
        0x69 => format!("ld\t\tl, c"),
        0x6A => format!("ld\t\tl, d"),
        0x6B => format!("ld\t\tl, e"),
        0x6C => format!("ld\t\tl, h"),
        0x6D => format!("ld\t\tl, l"),
        0x6E => format!("ld\t\tl, (hl)"),
        0x6F => format!("ld\t\tl, a"),

        0x70 => format!("ld\t\t(hl), b"),
        0x71 => format!("ld\t\t(hl), c"),
        0x72 => format!("ld\t\t(hl), d"),
        0x73 => format!("ld\t\t(hl), e"),
        0x74 => format!("ld\t\t(hl), h"),
        0x75 => format!("ld\t\t(hl), l"),
        0x76 => format!("halt"),
        0x77 => format!("ld\t\t(hl), a"),
        0x78 => format!("ld\t\ta, b"),
        0x79 => format!("ld\t\ta, c"),
        0x7A => format!("ld\t\ta, d"),
        0x7B => format!("ld\t\ta, e"),
        0x7C => format!("ld\t\ta, h"),
        0x7D => format!("ld\t\ta, l"),
        0x7E => format!("ld\t\ta, (hl)"),
        0x7F => format!("ld\t\ta, a"),

        0x80 => format!("add\t\ta, b"),
        0x81 => format!("add\t\ta, c"),
        0x82 => format!("add\t\ta, d"),
        0x83 => format!("add\t\ta, e"),
        0x84 => format!("add\t\ta, h"),
        0x85 => format!("add\t\ta, l"),
        0x86 => format!("add\t\ta, (hl)"),
        0x87 => format!("add\t\ta, a"),
        0x88 => format!("adc\t\ta, b"),
        0x89 => format!("adc\t\ta, c"),
        0x8A => format!("adc\t\ta, d"),
        0x8B => format!("adc\t\ta, e"),
        0x8C => format!("adc\t\ta, h"),
        0x8D => format!("adc\t\ta, l"),
        0x8E => format!("adc\t\ta, (hl)"),
        0x8F => format!("adc\t\ta"),

        0x90 => format!("sub\t\ta, b"),
        0x91 => format!("sub\t\ta, c"),
        0x92 => format!("sub\t\ta, d"),
        0x93 => format!("sub\t\ta, e"),
        0x94 => format!("sub\t\ta, h"),
        0x95 => format!("sub\t\ta, l"),
        0x96 => format!("sub\t\ta, (hl)"),
        0x97 => format!("sub\t\ta, a"),
        0x98 => format!("suc\t\ta, b"),
        0x99 => format!("suc\t\ta, c"),
        0x9A => format!("suc\t\ta, d"),
        0x9B => format!("suc\t\ta, e"),
        0x9C => format!("suc\t\ta, h"),
        0x9D => format!("suc\t\ta, l"),
        0x9E => format!("sbc\t\ta, (HL)"),
        0x9F => format!("suc\t\ta, a"),

        0xA0 => format!("and\t\ta, b"),
        0xA1 => format!("and\t\ta, c"),
        0xA2 => format!("and\t\ta, d"),
        0xA3 => format!("and\t\ta, e"),
        0xA4 => format!("and\t\ta, h"),
        0xA5 => format!("and\t\ta, l"),
        0xA6 => format!("and\t\ta, (hl)"),
        0xA7 => format!("and\t\ta, a"),
        0xA8 => format!("xor\t\ta, b"),
        0xA9 => format!("xor\t\ta, c"),
        0xAA => format!("xor\t\ta, d"),
        0xAB => format!("xor\t\ta, e"),
        0xAC => format!("xor\t\ta, h"),
        0xAD => format!("xor\t\ta, l"),
        0xAE => format!("xor\t\ta, (hl)"),
        0xAF => format!("xor\t\ta, a"),

        0xB1 => format!("or\t\ta, c"),
        0xB2 => format!("or\t\ta, d"),
        0xB0 => format!("or\t\ta, b"),
        0xB3 => format!("or\t\ta, e"),
        0xB4 => format!("or\t\ta, h"),
        0xB5 => format!("or\t\ta, l"),
        0xB6 => format!("or\t\ta, (hl)"),
        0xB7 => format!("or\t\ta, a"),
        0xB8 => format!("cp\t\ta, b"),
        0xB9 => format!("cp\t\ta, c"),
        0xBA => format!("cp\t\ta, d"),
        0xBB => format!("cp\t\ta, e"),
        0xBC => format!("cp\t\ta, h"),
        0xBD => format!("cp\t\ta, l"),
        0xBE => format!("cp\t\ta, (hl)"),
        0xBF => format!("cp\t\ta, a"),

        0xC0 => format!("ret\t\tnz"),
        0xC1 => format!("pop\t\tbc"),
        0xC2 => format!("jp\t\tnz, {:X}h", get_nn!()),
        0xC3 => format!("jp\t\t{:X}h", get_nn!()),
        0xC4 => format!("call\tnz, {:X}h", get_nn!()),
        0xC5 => format!("push\tbc"),
        0xC6 => format!("add\t\ta, {}", get_n!()),
        0xC7 => format!("rst\t\t00h"),
        0xC8 => format!("ret\t\tz"),
        0xC9 => format!("ret"),
        0xCA => format!("jp\t\tz, {:X}h", get_nn!()),
        0xCB => disasm_long(addr, mem),
        0xCC => format!("call\tz, {:X}h", get_nn!()),
        0xCD => format!("call\t{:X}h", get_nn!()),
        0xCE => format!("adc\t\ta, {:X}h", get_n!()),
        0xCF => format!("rst\t\t08h"),

        0xD0 => format!("ret\t\tnc"),
        0xD1 => format!("pop\t\tde"),
        0xD2 => format!("jp\t\tnc, {:X}h", get_nn!()),
        0xD3 => invalid_inst(op),
        0xD4 => format!("call\tnc, {:X}h", get_nn!()),
        0xD5 => format!("push\tde"),
        0xD6 => format!("add\t\ta, {}", get_n!()),
        0xD7 => format!("rst\t\t10h"),
        0xD8 => format!("ret\t\tc"),
        0xD9 => format!("reti"),
        0xDA => format!("jp\t\tc, {:X}h", get_nn!()),
        0xDB => invalid_inst(op),
        0xDC => format!("call\tc, {:X}h", get_nn!()),
        0xDD => invalid_inst(op),
        0xDE => format!("sbc\t\ta, {:X}h", get_n!()),
        0xDF => format!("rst\t\t18h"),

        0xE0 => format!("ld\t\t(FF00+{:X}h), a", get_n!()),
        0xE1 => format!("pop\t\thl"),
        0xE2 => format!("ld\t\t(FF00+C), a"),
        0xE3 => invalid_inst(op),
        0xE4 => invalid_inst(op),
        0xE5 => format!("push\thl"),
        0xE6 => format!("and\t\ta, {}", get_n!()),
        0xE7 => format!("rst\t\t20h"),
        0xE8 => format!("add\t\tsp, {}", get_n!()),
        0xE9 => format!("jp\t\thl"),
        0xEA => format!("ld\t\t({:X}h), a", get_nn!()),
        0xEB => invalid_inst(op),
        0xEC => invalid_inst(op),
        0xED => invalid_inst(op),
        0xEE => format!("xor\t\ta, {}",get_n!()),
        0xEF => format!("rst\t\t28h"),

        0xF0 => format!("ld\t\ta, (FF00+{:X}h)", get_n!()),
        0xF1 => format!("pop\t\taf"),
        0xF2 => format!("ld\t\ta, (FF00+C)"),
        0xF3 => format!("di"),
        0xF4 => invalid_inst(op),
        0xF5 => format!("push\taf"),
        0xF6 => format!("or\t\ta, {}", get_n!()),
        0xF7 => format!("rst\t\t30h"),
        0xF8 => format!("ld\t\thl, sp+{}", get_ni!()),
        0xF9 => format!("ld\t\tsp, hl"),
        0xFA => format!("ld\t\ta, ({:X}h)", get_nn!()),
        0xFB => format!("ei"),
        0xFC => invalid_inst(op),
        0xFD => invalid_inst(op),
        0xFE => format!("cp\t\ta, {}", get_n!()),
        0xFF => format!("rst\t\t38h"),
    }
}

fn disasm_long(addr: u16, mem: &Memory) -> String{
    let op = mem.lb(addr);

    match op {
        0x00 => format!("rlc\t\tb"),
        0x01 => format!("rlc\t\tc"),
        0x02 => format!("rlc\t\td"),
        0x03 => format!("rlc\t\te"),
        0x04 => format!("rlc\t\th"),
        0x05 => format!("rlc\t\tl"),
        0x06 => format!("rlc\t\t(HL)"),
        0x07 => format!("rlc\t\ta"),
        0x08 => format!("rrc\t\tb"),
        0x09 => format!("rrc\t\tc"),
        0x0A => format!("rrc\t\td"),
        0x0B => format!("rrc\t\te"),
        0x0C => format!("rrc\t\th"),
        0x0D => format!("rrc\t\tl"),
        0x0E => format!("rrc\t\t(HL)"),
        0x0F => format!("rrc\t\ta"),

        0x10 => format!("rl\t\tb"),
        0x11 => format!("rl\t\tc"),
        0x12 => format!("rl\t\td"),
        0x13 => format!("rl\t\te"),
        0x14 => format!("rl\t\th"),
        0x15 => format!("rl\t\tl"),
        0x16 => format!("rl\t\t(HL)"),
        0x17 => format!("rl\t\ta"),
        0x18 => format!("rr\t\tb"),
        0x19 => format!("rr\t\tc"),
        0x1A => format!("rr\t\td"),
        0x1B => format!("rr\t\te"),
        0x1C => format!("rr\t\th"),
        0x1D => format!("rr\t\tl"),
        0x1E => format!("rr\t\t(HL)"),
        0x1F => format!("rr\t\ta"),

        0x20 => format!("sla\t\tb"),
        0x21 => format!("sla\t\tc"),
        0x22 => format!("sla\t\td"),
        0x23 => format!("sla\t\te"),
        0x24 => format!("sla\t\th"),
        0x25 => format!("sla\t\tl"),
        0x26 => format!("sla\t\t(HL)"),
        0x27 => format!("sla\t\ta"),
        0x28 => format!("sra\t\tb"),
        0x29 => format!("sra\t\tc"),
        0x2A => format!("sra\t\td"),
        0x2B => format!("sra\t\te"),
        0x2C => format!("sra\t\th"),
        0x2D => format!("sra\t\tl"),
        0x2E => format!("sra\t\t(HL)"),
        0x2F => format!("sra\t\ta"),

        0x30 => format!("swap\t\tb"),
        0x31 => format!("swap\t\tc"),
        0x32 => format!("swap\t\td"),
        0x33 => format!("swap\t\te"),
        0x34 => format!("swap\t\th"),
        0x35 => format!("swap\t\tl"),
        0x36 => format!("swap\t\t(HL)"),
        0x37 => format!("swap\t\ta"),
        0x38 => format!("srl\t\tb"),
        0x39 => format!("srl\t\tc"),
        0x3A => format!("srl\t\td"),
        0x3B => format!("srl\t\te"),
        0x3C => format!("srl\t\th"),
        0x3D => format!("srl\t\tl"),
        0x3E => format!("srl\t\t(HL)"),
        0x3F => format!("srl\t\ta"),

        0x40 => format!("bit\t\t0, b"),
        0x41 => format!("bit\t\t0, c"),
        0x42 => format!("bit\t\t0, d"),
        0x43 => format!("bit\t\t0, e"),
        0x44 => format!("bit\t\t0, h"),
        0x45 => format!("bit\t\t0, l"),
        0x46 => format!("bit\t\t0, (hl)"),
        0x47 => format!("bit\t\t0, a"),
        0x48 => format!("bit\t\t1, b"),
        0x49 => format!("bit\t\t1, c"),
        0x4A => format!("bit\t\t1, d"),
        0x4B => format!("bit\t\t1, e"),
        0x4C => format!("bit\t\t1, h"),
        0x4D => format!("bit\t\t1, l"),
        0x4E => format!("bit\t\t1, (hl)"),
        0x4F => format!("bit\t\t1, a"),

        0x50 => format!("bit\t\t2, b"),
        0x51 => format!("bit\t\t2, c"),
        0x52 => format!("bit\t\t2, d"),
        0x53 => format!("bit\t\t2, e"),
        0x54 => format!("bit\t\t2, h"),
        0x55 => format!("bit\t\t2, l"),
        0x56 => format!("bit\t\t2, (hl)"),
        0x57 => format!("bit\t\t2, a"),
        0x58 => format!("bit\t\t3, b"),
        0x59 => format!("bit\t\t3, c"),
        0x5A => format!("bit\t\t3, d"),
        0x5B => format!("bit\t\t3, e"),
        0x5C => format!("bit\t\t3, h"),
        0x5D => format!("bit\t\t3, l"),
        0x5E => format!("bit\t\t3, (hl)"),
        0x5F => format!("bit\t\t3, a"),

        0x60 => format!("bit\t\t4, b"),
        0x61 => format!("bit\t\t4, c"),
        0x62 => format!("bit\t\t4, d"),
        0x63 => format!("bit\t\t4, e"),
        0x64 => format!("bit\t\t4, h"),
        0x65 => format!("bit\t\t4, l"),
        0x66 => format!("bit\t\t4, (hl)"),
        0x67 => format!("bit\t\t4, a"),
        0x68 => format!("bit\t\t5, b"),
        0x69 => format!("bit\t\t5, c"),
        0x6A => format!("bit\t\t5, d"),
        0x6B => format!("bit\t\t5, e"),
        0x6C => format!("bit\t\t5, h"),
        0x6D => format!("bit\t\t5, l"),
        0x6E => format!("bit\t\t5, (hl)"),
        0x6F => format!("bit\t\t5, a"),

        0x70 => format!("bit\t\t6, b"),
        0x71 => format!("bit\t\t6, b"),
        0x72 => format!("bit\t\t6, b"),
        0x73 => format!("bit\t\t6, b"),
        0x74 => format!("bit\t\t6, b"),
        0x75 => format!("bit\t\t6, b"),
        0x76 => format!("bit\t\t6, (hl)"),
        0x77 => format!("bit\t\t6, a"),
        0x78 => format!("bit\t\t7, b"),
        0x79 => format!("bit\t\t7, c"),
        0x7A => format!("bit\t\t7, d"),
        0x7B => format!("bit\t\t7, e"),
        0x7C => format!("bit\t\t7, j"),
        0x7D => format!("bit\t\t7, l"),
        0x7E => format!("bit\t\t7, (hl)"),
        0x7F => format!("bit\t\t7, a"),

        0x80 => format!("res\t\t0, b"),
        0x81 => format!("res\t\t0, c"),
        0x82 => format!("res\t\t0, d"),
        0x83 => format!("res\t\t0, e"),
        0x84 => format!("res\t\t0, h"),
        0x85 => format!("res\t\t0, l"),
        0x86 => format!("res\t\t0, (hl)"),
        0x87 => format!("res\t\t0, a"),
        0x88 => format!("res\t\t1, b"),
        0x89 => format!("res\t\t1, c"),
        0x8A => format!("res\t\t1, d"),
        0x8B => format!("res\t\t1, e"),
        0x8C => format!("res\t\t1, h"),
        0x8D => format!("res\t\t1, l"),
        0x8E => format!("res\t\t1, (hl)"),
        0x8F => format!("res\t\t1, a"),

        0x90 => format!("res\t\t2, b"),
        0x91 => format!("res\t\t2, c"),
        0x92 => format!("res\t\t2, d"),
        0x93 => format!("res\t\t2, e"),
        0x94 => format!("res\t\t2, h"),
        0x95 => format!("res\t\t2, l"),
        0x96 => format!("res\t\t2, (hl)"),
        0x97 => format!("res\t\t2, a"),
        0x98 => format!("res\t\t3, b"),
        0x99 => format!("res\t\t3, c"),
        0x9A => format!("res\t\t3, d"),
        0x9B => format!("res\t\t3, e"),
        0x9C => format!("res\t\t3, h"),
        0x9D => format!("res\t\t3, l"),
        0x9E => format!("res\t\t3, (hl)"),
        0x9F => format!("res\t\t3, a"),

        0xA0 => format!("res\t\t4, b"),
        0xA1 => format!("res\t\t4, c"),
        0xA2 => format!("res\t\t4, d"),
        0xA3 => format!("res\t\t4, e"),
        0xA4 => format!("res\t\t4, h"),
        0xA5 => format!("res\t\t4, l"),
        0xA6 => format!("res\t\t4, (hl)"),
        0xA7 => format!("res\t\t4, a"),
        0xA8 => format!("res\t\t5, b"),
        0xA9 => format!("res\t\t5, c"),
        0xAA => format!("res\t\t5, d"),
        0xAB => format!("res\t\t5, e"),
        0xAC => format!("res\t\t5, h"),
        0xAD => format!("res\t\t5, l"),
        0xAE => format!("res\t\t5, (hl)"),
        0xAF => format!("res\t\t5, a"),

        0xB0 => format!("res\t\t6, b"),
        0xB1 => format!("res\t\t6, c"),
        0xB2 => format!("res\t\t6, d"),
        0xB3 => format!("res\t\t6, e"),
        0xB4 => format!("res\t\t6, h"),
        0xB5 => format!("res\t\t6, l"),
        0xB6 => format!("res\t\t6, (hl)"),
        0xB7 => format!("res\t\t6, a"),
        0xB8 => format!("res\t\t7, b"),
        0xB9 => format!("res\t\t7, c"),
        0xBA => format!("res\t\t7, d"),
        0xBB => format!("res\t\t7, e"),
        0xBC => format!("res\t\t7, h"),
        0xBD => format!("res\t\t7, l"),
        0xBE => format!("res\t\t7, (hl)"),
        0xBF => format!("res\t\t7, a"),

        0xC0 => format!("set\t\t0, b"),
        0xC1 => format!("set\t\t0, c"),
        0xC2 => format!("set\t\t0, d"),
        0xC3 => format!("set\t\t0, e"),
        0xC4 => format!("set\t\t0, h"),
        0xC5 => format!("set\t\t0, l"),
        0xC6 => format!("set\t\t0, (hl)"),
        0xC7 => format!("set\t\t0, a"),
        0xC8 => format!("set\t\t1, b"),
        0xC9 => format!("set\t\t1, c"),
        0xCA => format!("set\t\t1, d"),
        0xCB => format!("set\t\t1, e"),
        0xCC => format!("set\t\t1, h"),
        0xCD => format!("set\t\t1, l"),
        0xCE => format!("set\t\t1, (hl)"),
        0xCF => format!("set\t\t1, a"),

        0xD0 => format!("set\t\t2, b"),
        0xD1 => format!("set\t\t2, c"),
        0xD2 => format!("set\t\t2, d"),
        0xD3 => format!("set\t\t2, e"),
        0xD4 => format!("set\t\t2, h"),
        0xD5 => format!("set\t\t2, l"),
        0xD6 => format!("set\t\t2, (hl)"),
        0xD7 => format!("set\t\t2, a"),
        0xD8 => format!("set\t\t3, b"),
        0xD9 => format!("set\t\t3, c"),
        0xDA => format!("set\t\t3, d"),
        0xDB => format!("set\t\t3, e"),
        0xDC => format!("set\t\t3, h"),
        0xDD => format!("set\t\t3, l"),
        0xDE => format!("set\t\t3, (hl)"),
        0xDF => format!("set\t\t3, a"),

        0xE0 => format!("set\t\t4, b"),
        0xE1 => format!("set\t\t4, c"),
        0xE2 => format!("set\t\t4, d"),
        0xE3 => format!("set\t\t4, e"),
        0xE4 => format!("set\t\t4, h"),
        0xE5 => format!("set\t\t4, l"),
        0xE6 => format!("set\t\t4, (hl)"),
        0xE7 => format!("set\t\t4, a"),
        0xE8 => format!("set\t\t5, b"),
        0xE9 => format!("set\t\t5, c"),
        0xEA => format!("set\t\t5, d"),
        0xEB => format!("set\t\t5, e"),
        0xEC => format!("set\t\t5, h"),
        0xED => format!("set\t\t5, l"),
        0xEE => format!("set\t\t5, (hl)"),
        0xEF => format!("set\t\t5, a"),

        0xF0 => format!("set\t\t6, b"),
        0xF1 => format!("set\t\t6, c"),
        0xF2 => format!("set\t\t6, d"),
        0xF3 => format!("set\t\t6, e"),
        0xF4 => format!("set\t\t6, h"),
        0xF5 => format!("set\t\t6, l"),
        0xF6 => format!("set\t\t6, (hl)"),
        0xF7 => format!("set\t\t6, a"),
        0xF8 => format!("set\t\t7, b"),
        0xF9 => format!("set\t\t7, c"),
        0xFA => format!("set\t\t7, d"),
        0xFB => format!("set\t\t7, e"),
        0xFC => format!("set\t\t7, h"),
        0xFD => format!("set\t\t7, l"),
        0xFE => format!("set\t\t7, (hl)"),
        0xFF => format!("set\t\t7, a"),
    }
}
