//! Here we define functions to emulate the GB instructions. Because many of the instructions only
//! differ by the registers they are acting on macros are used to reduce code duplication.
//! (Inspired by alexchricton's GB emulator)

use cpu::Cpu;
use mmu::Memory;

// Flags for the GB processor:
pub const ZERO_FLAG: u8 = 0b1000_0000;
pub const ADD_SUB_FLAG: u8 = 0b0100_0000;
pub const HALF_CARRY_FLAG: u8 = 0b0010_0000;
pub const CARRY_FLAG: u8 = 0b0001_0000;
// Note: the lower 4 bits are unused.

/// Fetch and execute the next instruction, returning the number of cycles used to complete the
/// operation.
pub fn fetch_exec(cpu: &mut Cpu, mem: &mut Memory) -> u8 {
    // Read the next byte after pc, useful for instructions requiring a `n` parameter
    macro_rules! get_n { () => (mem.lb(cpu.bump())) }
    // Read the next two bytes after pc, useful for instructions requiring a `nn` parameter
    macro_rules! get_nn { () => (
        get_n!() as u16 + (get_n!() as u16 << 8)
    ) }
    // Check if the zero flag is set
    macro_rules! zflag { () => ( cpu.f & ZERO_FLAG != 0 ) }
    // Check if the carry flag is set
    macro_rules! cflag { () => ( cpu.f & CARRY_FLAG != 0 ) }

    //
    // Instructions inherited from the Z80 processor
    //

    // LD r1, r2
    macro_rules! ld { ($r1: ident, $r2: ident) => ({
        cpu.$r1 = cpu.$r2;
    }) }
    // LD r, n
    macro_rules! ld_rn { ($r: ident) => ({
        cpu.$r = get_n!();
    }) }
    // LD r, (DD)
    macro_rules! ld_rDD { ($r: ident, $DD: ident) => ({
        cpu.$r = mem.lb(cpu.$DD().get());
    }) }
    // LD (DD), r
    macro_rules! ld_DDr { ($DD: ident, $r: ident) => ({
        mem.sb(cpu.$DD().get(), cpu.$r);
    }) }
    // LD (DD), n
    macro_rules! ld_DDn { ($DD: ident) => ({
        let val =  mem.lb(cpu.bump());
        mem.sb(cpu.$DD().get(), val);
    }) }
    // LD r, (NN)
    macro_rules! ld_rNN { ($r: ident) => ({
        let addr = get_nn!();
        cpu.$r = mem.lb(addr);
    }) }
    // LD (NN), r
    macro_rules! ld_NNr { ($r: ident) => ({
        let addr = get_nn!();
        mem.sb(addr, cpu.$r);
    }) }
    // LD dd, nn
    macro_rules! ld_ddnn { ($dd: ident) => ({
        let val = get_nn!();
        cpu.$dd().set(val);
    }) }
    // LD dd, (nn)
    macro_rules! ld_ddNN { ($dd: ident) => ({
        let addr = get_nn!();
        cpu.$dd().high = mem.lb(addr + 1);
        cpu.$dd().low = mem.lb(addr);
    }) }
    // LD (nn), dd
    macro_rules! ld_NNdd { ($dd: ident) => ({
        let addr = get_nn!();
        mem.sb(addr + 1, cpu.$dd().high);
        mem.sb(addr, cpu.$dd().low);
    }) }
    // LD SP, HL
    macro_rules! ld_sphl { () => (
        cpu.sp = cpu.hl().get();
    ) }
    // LD (nn), SP
    macro_rules! ld_NNsp { () => ({
        let addr = get_nn!();
        mem.sw(addr, cpu.sp);
    }) }
    // LD A,(FF00+n)
    macro_rules! ld_aIOn { () => ({
        let addr = 0xFF00 | get_n!() as u16;
        cpu.a = mem.lb(addr);
    }) }
    // LD A,(FF00+C)
    macro_rules! ld_aIOc { () => ({
        let addr = 0xFF00 | cpu.c as u16;
        cpu.a = mem.lb(addr);
    }) }
    // LD (FF00+n),A
    macro_rules! ld_IOan { () => ({
        let addr = 0xFF00 | get_n!() as u16;
        mem.sb(addr, cpu.a);
    }) }
    // LD (FF00+C),A
    macro_rules! ld_IOca { () => ({
        let addr = 0xFF00 | cpu.c as u16;
        mem.sb(addr, cpu.a);
    }) }
    // PUSH qq
    macro_rules! push_qq { ($qq: ident) => ({
        let val = cpu.$qq().get();
        cpu.push16(mem, val);
    }) }
    // POP qq
    macro_rules! pop_qq { ($qq: ident) => ({
        let val = cpu.pop16(mem);
        cpu.$qq().set(val);
    }) }
    // POP af
    macro_rules! pop_af { () => ({
        pop_qq!(af);
        cpu.f &= 0xF0;
    }) }

    // ADD A, s
    macro_rules! add_a { ($s: expr) => ({
        let val = $s;
        cpu.f = 0;
        if (cpu.a & 0xf) + (val & 0xf) > 0xf { cpu.f |= HALF_CARRY_FLAG; }
        if cpu.a as u16 + val as u16 > 0xff { cpu.f |= CARRY_FLAG; }
        cpu.a += val;
        if cpu.a == 0 { cpu.f |= ZERO_FLAG; }
    }) }
    macro_rules! add_an { () => (add_a!(get_n!())) }
    macro_rules! add_ar { ($r: ident) => (add_a!(cpu.$r)) }
    macro_rules! add_aHL { () => (add_a!(mem.lb(cpu.hl().get()))) }

    // ADD HL, ..
    macro_rules! add_hl { ($s: expr) => ({
        let a = cpu.hl().get() as u32;
        let b = $s as u32;

        cpu.f &= ZERO_FLAG;
        if (a & 0xfff) + (b & 0xfff) > 0xfff { cpu.f |= HALF_CARRY_FLAG; }
        if a + b > 0xffff { cpu.f |= CARRY_FLAG; }

        cpu.hl().set((a + b) as u16);
    }) }
    macro_rules! add_hlss { ($ss: ident) => (add_hl!(cpu.$ss().get())) }
    macro_rules! add_hlsp { () => (add_hl!(cpu.sp)) }

    // ADD SP, n
    macro_rules! add_spn { () => ({
        // Get offset as a 2s complement integer
        let offset = (get_n!() as i8) as i16;
        // Not sure if conditions bits should be set here
        cpu.sp = (cpu.sp as i16 + offset) as u16;
    }) }

    // ADC A, s
    macro_rules! adc_a { ($s: expr) => (
        add_a!($s + ((cpu.f >> 4) & 0x1))
    ) }
    macro_rules! adc_an { () => (adc_a!(get_n!())) }
    macro_rules! adc_ar { ($r: ident) => (adc_a!(cpu.$r)) }
    macro_rules! adc_aHL { () => (adc_a!(mem.lb(cpu.hl().get()))) }

    // ADC HL, ss
    macro_rules! adc_hlss { ($ss: ident) => (
        add_hl!(cpu.$ss.get() + ((cpu.f >> 4) & 0x1))
    ) }

    // CP A, s
    macro_rules! cp_a { ($s: expr) => ({
        let val = $s;
        cpu.f = ADD_SUB_FLAG;
        if (cpu.a & 0xf) < (val & 0xf) { cpu.f |= HALF_CARRY_FLAG; }
        if cpu.a < val { cpu.f |= CARRY_FLAG; }
        if cpu.a == val { cpu.f |= ZERO_FLAG; }
    }) }
    macro_rules! cp_an { () => (cp_a!(get_n!())) }
    macro_rules! cp_ar { ($r: ident) => (cp_a!(cpu.$r)) }
    macro_rules! cp_aHL { () => (cp_a!(mem.lb(cpu.hl().get()))) }

    // SUB A, s
    macro_rules! sub_a { ($s: expr) => ({
        let val = $s;
        cp_a!(val);
        cpu.a -= val;
    }) }
    macro_rules! sub_an { () => (sub_a!(get_n!())) }
    macro_rules! sub_ar { ($r: ident) => (sub_a!(cpu.$r)) }
    macro_rules! sub_aHL { () => (sub_a!(mem.lb(cpu.hl().get()))) }

    // SBC A, s
    macro_rules! sbc_a { ($s: expr) => (
        sub_a!($s + ((cpu.f >> 4) & 0x1))
    ) }
    macro_rules! sbc_an { () => (sbc_a!(get_n!())) }
    macro_rules! sbc_ar { ($r: ident) => (sbc_a!(cpu.$r)) }
    macro_rules! sbc_aHL { () => (sbc_a!(mem.lb(cpu.hl().get()))) }

    // AND A, s
    macro_rules! and_a { ($s: expr) => ({
        cpu.a &= $s;
        cpu.f = HALF_CARRY_FLAG;
        if cpu.a == 0 { cpu.f |= ZERO_FLAG; }
    }) }
    macro_rules! and_an { () => (and_a!(get_n!())) }
    macro_rules! and_ar { ($r: ident) => (and_a!(cpu.$r)) }
    macro_rules! and_aHL { () => (and_a!(mem.lb(cpu.hl().get()))) }

    // OR A, s
    macro_rules! or_a { ($s: expr) => ({
        cpu.a |= $s;
        cpu.f = 0;
        if cpu.a == 0 { cpu.f |= ZERO_FLAG; }
    }) }
    macro_rules! or_an { () => (or_a!(get_n!())) }
    macro_rules! or_ar { ($r: ident) => (or_a!(cpu.$r)) }
    macro_rules! or_aHL { () => (or_a!(mem.lb(cpu.hl().get()))) }

    // XOR A, s
    macro_rules! xor_a { ($s: expr) => ({
        cpu.a ^= $s;
        cpu.f = 0;
        if cpu.a == 0 { cpu.f |= ZERO_FLAG; }
    }) }
    macro_rules! xor_an { () => (xor_a!(get_n!())) }
    macro_rules! xor_ar { ($r: ident) => (xor_a!(cpu.$r)) }
    macro_rules! xor_aHL { () => (xor_a!(mem.lb(cpu.hl().get()))) }

    // INC r
    macro_rules! inc_r { ($r: ident) => ({
        cpu.$r += 1;
        cpu.f &= CARRY_FLAG;
        if (cpu.$r & 0xf) == 0 { cpu.f |= HALF_CARRY_FLAG; }
        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }
    // INC (HL)
    macro_rules! inc_HL { () => ({
        let val = mem.lb(cpu.hl().get()) + 1;
        mem.sb(cpu.hl().get(), val);

        cpu.f &= CARRY_FLAG;
        if (val & 0xf) == 0 { cpu.f |= HALF_CARRY_FLAG; }
        if val == 0 { cpu.f |= ZERO_FLAG; }
    }) }
    // INC ss
    macro_rules! inc_ss { ($ss: ident) => ({
        let val = cpu.$ss().get() + 1;
        cpu.$ss().set(val);
    }) }
    // INC sp
    macro_rules! inc_sp { () => ({
        cpu.sp += 1;
    }) }
    // DEC r
    macro_rules! dec_r { ($r: ident) => ({
        cpu.$r -= 1;
        cpu.f &= CARRY_FLAG;
        cpu.f |= ADD_SUB_FLAG;
        if (cpu.$r & 0xf) == 0xf { cpu.f |= HALF_CARRY_FLAG; }
        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }
    // DEC (HL)
    macro_rules! dec_HL { () => ({
        let val = mem.lb(cpu.hl().get()) - 1;
        mem.sb(cpu.hl().get(), val);

        cpu.f &= CARRY_FLAG;
        cpu.f |= ADD_SUB_FLAG;
        if (val & 0xf) == 0xf { cpu.f |= HALF_CARRY_FLAG; }
        if val == 0 { cpu.f |= ZERO_FLAG; }
    }) }
    // DEC ss
    macro_rules! dec_ss { ($ss: ident) => ({
        let val = cpu.$ss().get() - 1;
        cpu.$ss().set(val);
    }) }
    // DEC sp
    macro_rules! dec_sp { () => ({
        cpu.sp -= 1;
    }) }

    // DAA
    macro_rules! daa { () => ({
        daa(cpu);
    }) }

    // CPL
    macro_rules! cpl { () => ({
        cpu.a ^= 0xFF;
        cpu.f |= HALF_CARRY_FLAG | ADD_SUB_FLAG;
    }) }
    // CCF
    macro_rules! ccf { () => ({
        cpu.f ^= CARRY_FLAG;
        cpu.f &= ZERO_FLAG | CARRY_FLAG;
        // Should the old carry flag be copied to HALF_CARRY_FLAG?
    }) }
    // SCF
    macro_rules! scf { () => ({
        cpu.f &= ZERO_FLAG;
        cpu.f |= CARRY_FLAG;
    }) }
    // NOP
    macro_rules! nop { () => ( {} ) }
    // HALT
    macro_rules! halt { () => ( cpu.halt(); ) }
    // DI
    macro_rules! di { () => ( cpu.disable_interrupts(mem); ) }
    // EI
    macro_rules! ei { () => ( cpu.enable_interrupts(mem); ) }

    // RLCA
    macro_rules! rlca { () => ({
        let copy_bit = cpu.a >> 7;
        cpu.a = (cpu.a << 1) + copy_bit;
        if copy_bit == 0 { cpu.f &= !CARRY_FLAG } else { cpu.f |= CARRY_FLAG };
    }) }
    // RLA
    macro_rules! rla { () => ({
        let copy_bit = if cpu.f & CARRY_FLAG == 0 { 0 } else { 1 };
        if cpu.a >> 7 == 0 { cpu.f &= !CARRY_FLAG } else { cpu.f |= CARRY_FLAG };
        cpu.a = (cpu.a << 1) + copy_bit;
    }) }
    // RRCA
    macro_rules! rrca { () => ({
        let copy_bit = cpu.a & 0x1;
        if copy_bit == 0 { cpu.f &= !CARRY_FLAG } else { cpu.f |= CARRY_FLAG };
        cpu.a = (cpu.a >> 1) + (copy_bit << 7);
    }) }
    // RRA
    macro_rules! rra { () => ({
        let copy_bit = if cpu.f & CARRY_FLAG == 0 { 0 } else { 1 };
        if cpu.a & 0x1 == 0 { cpu.f &= !CARRY_FLAG } else { cpu.f |= CARRY_FLAG };
        cpu.a = (cpu.a >> 1) + (copy_bit << 7);
    }) }

    // JP nn
    macro_rules! jp_nn { () => ({
        let addr = get_nn!();
        cpu.jump(addr);
    }) }
    // JP c, nn
    macro_rules! jp_cnn { ($c: expr) => (
        if $c { jp_nn!(); 4 } else { cpu.pc += 2; 3 }
    ) }
    // JP hl
    macro_rules! jp_hl { () => ({
        let addr = cpu.hl().get();
        cpu.jump(addr);
    }) }
    // JR e
    macro_rules! jr_e { () => ({
        // Get offset as a 2s-complement integer
        let offset = (mem.lb(cpu.bump()) as i8) as i16;
        cpu.pc = (cpu.pc as i16 + offset) as u16;
    }) }
    // JR c, e
    macro_rules! jr_ce { ($c: expr) => (
        if $c { jr_e!(); 3 } else { cpu.pc += 1; 2 }
    ) }
    // JP (HL)
    macro_rules! jp_HL { () => ({
        let addr = mem.lw(cpu.hl().get());
        cpu.jump(addr);
    }) }

    // CALL nn
    macro_rules! call_nn { () => ({
        let addr = get_nn!();
        cpu.call(mem, addr);
    }) }
    // CALL c, nn
    macro_rules! call_cnn { ($cc: expr) => (
        if $cc { call_nn!(); 6 } else { cpu.pc += 2; 3 }
    ) }
    // RET
    macro_rules! ret { () => ({
        cpu.ret(mem);
    }) }
    // RET c
    macro_rules! ret_c { ($c: expr) => (
        if $c { ret!(); 3 } else { 1 }
    ) }
    // RETI
    macro_rules! reti { () => ({
        cpu.enable_interrupts(mem);
        ret!();
    }) }
    // RST p
    macro_rules! rst_p { ($p: expr) => ({
        let addr = $p;
        cpu.call(mem, addr);
    }) }

    //
    // Added instructions for GB
    //

    macro_rules! inc_hl { () => ({
        let v = cpu.hl().get();
        cpu.hl().set(v + 1);
    }) }

    macro_rules! dec_hl { () => ({
        let v = cpu.hl().get();
        cpu.hl().set(v - 1);
    }) }

    // LD sp, nn
    macro_rules! ld_spnn { () => ({
        let val = get_nn!();
        cpu.sp = val;
    }) }
    // LDI A, (HL)
    macro_rules! ldi_aHL { () => ({
        cpu.a = mem.lb(cpu.hl().get());
        inc_hl!();
    }) }
    // LDI (HL), A
    macro_rules! ldi_HLa { () => ({
        let a = cpu.a;
        mem.sb(cpu.hl().get(), a);
        inc_hl!();
    }) }
    // LDD A, (HL)
    macro_rules! ldd_aHL { () => ({
        cpu.a = mem.lb(cpu.hl().get());
        dec_hl!();
    }) }
    // LDD (HL), A
    macro_rules! ldd_HLa { () => ({
        let a = cpu.a;
        mem.sb(cpu.hl().get(), a);
        dec_hl!();
    }) }
    // LD HL,SP+n
    macro_rules! ld_hlspn { () => ({
        let val = ((get_n!() as i8) as i16 + cpu.sp as i16) as u16;
        cpu.hl().set(val);
    }) }

    // STOP
    macro_rules! stop { () => ( cpu.stop(); ) }

    // Fetch the next instruction from memory
    let op = mem.lb(cpu.bump());

    // Decode and execute the instruction, returning the number of cycles required to run it.
    // See http://imrannazar.com/Gameboy-Z80-Opcode-Map for decode table
    // See http://problemkaputt.de/pandocs.htm#cpuinstructionset for instruction timings
    match op {
        //      |  Instruction   | Cycles |
        0x00 => { nop!();               1 },
        0x01 => { ld_ddnn!(bc);         3 },
        0x02 => { ld_DDr!(bc, a);       2 },
        0x03 => { inc_ss!(bc);          2 },
        0x04 => { inc_r!(b);            1 },
        0x05 => { dec_r!(b);            1 },
        0x06 => { ld_rn!(b);            2 },
        0x07 => { rlca!();              1 },
        0x08 => { ld_NNsp!();           5 },
        0x09 => { add_hlss!(bc);        2 },
        0x0A => { ld_rDD!(a, bc);       2 },
        0x0B => { dec_ss!(bc);          2 },
        0x0C => { inc_r!(c);            1 },
        0x0D => { dec_r!(c);            1 },
        0x0E => { ld_rn!(c);            2 },
        0x0F => { rrca!();              1 },

        0x10 => { stop!();              1 },
        0x11 => { ld_ddnn!(de);         3 },
        0x12 => { ld_DDr!(de, a);       2 },
        0x13 => { inc_ss!(de);          2 },
        0x14 => { inc_r!(d);            1 },
        0x15 => { dec_r!(d);            1 },
        0x16 => { ld_rn!(d);            2 },
        0x17 => { rla!();               1 },
        0x18 => { jr_e!();              3 },
        0x19 => { add_hlss!(de);        2 },
        0x1A => { ld_rDD!(a, de);       2 },
        0x1B => { dec_ss!(de);          2 },
        0x1C => { inc_r!(e);            1 },
        0x1D => { dec_r!(e);            1 },
        0x1E => { ld_rn!(e);            2 },
        0x1F => { rra!();               1 },

        0x20 => { jr_ce!(!zflag!())       }, // 3;2
        0x21 => { ld_ddnn!(hl);         3 },
        0x22 => { ldi_HLa!();           2 },
        0x23 => { inc_ss!(hl);          2 },
        0x24 => { inc_r!(h);            1 },
        0x25 => { dec_r!(h);            1 },
        0x26 => { ld_rn!(h);            2 },
        0x27 => { daa!();               1 },
        0x28 => { jr_ce!(zflag!())        }, // 3;2
        0x29 => { add_hlss!(hl);        2 },
        0x2A => { ldi_aHL!();           2 },
        0x2B => { dec_ss!(hl);          2 },
        0x2C => { inc_r!(l);            1 },
        0x2D => { dec_r!(l);            1 },
        0x2E => { ld_rn!(l);            2 },
        0x2F => { cpl!();               1 },

        0x30 => { jr_ce!(!cflag!())       }, // 3;2
        0x31 => { ld_spnn!();           3 },
        0x32 => { ldd_HLa!();           2 },
        0x33 => { inc_sp!();            2 },
        0x34 => { inc_HL!();            3 },
        0x35 => { dec_HL!();            3 },
        0x36 => { ld_DDn!(hl);          3 },
        0x37 => { scf!();               1 },
        0x38 => { jr_ce!(cflag!())        }, // 3;2
        0x39 => { add_hlsp!();          2 },
        0x3A => { ldd_aHL!();           2 },
        0x3B => { dec_sp!();            1 },
        0x3C => { inc_r!(a);            1 },
        0x3D => { dec_r!(a);            1 },
        0x3E => { ld_rn!(a);            2 },
        0x3F => { ccf!();               1 },

        0x40 => { ld!(b, b);            1 },
        0x41 => { ld!(b, c);            1 },
        0x42 => { ld!(b, d);            1 },
        0x43 => { ld!(b, e);            1 },
        0x44 => { ld!(b, h);            1 },
        0x45 => { ld!(b, l);            1 },
        0x46 => { ld_rDD!(b, hl);       2 },
        0x47 => { ld!(b, a);            1 },
        0x48 => { ld!(c, b);            1 },
        0x49 => { ld!(c, c);            1 },
        0x4A => { ld!(c, d);            1 },
        0x4B => { ld!(c, e);            1 },
        0x4C => { ld!(c, h);            1 },
        0x4D => { ld!(c, l);            1 },
        0x4E => { ld_rDD!(c, hl);       2 },
        0x4F => { ld!(c, a);            1 },

        0x50 => { ld!(d, b);            1 },
        0x51 => { ld!(d, c);            1 },
        0x52 => { ld!(d, d);            1 },
        0x53 => { ld!(d, e);            1 },
        0x54 => { ld!(d, h);            1 },
        0x55 => { ld!(d, l);            1 },
        0x56 => { ld_rDD!(d, hl);       2 },
        0x57 => { ld!(d, a);            1 },
        0x58 => { ld!(e, b);            1 },
        0x59 => { ld!(e, c);            1 },
        0x5A => { ld!(e, d);            1 },
        0x5B => { ld!(e, e);            1 },
        0x5C => { ld!(e, h);            1 },
        0x5D => { ld!(e, l);            1 },
        0x5E => { ld_rDD!(e, hl);       2 },
        0x5F => { ld!(e, a);            1 },

        0x60 => { ld!(h, b);            1 },
        0x61 => { ld!(h, c);            1 },
        0x62 => { ld!(h, d);            1 },
        0x63 => { ld!(h, e);            1 },
        0x64 => { ld!(h, h);            1 },
        0x65 => { ld!(h, l);            1 },
        0x66 => { ld_rDD!(h, hl);       2 },
        0x67 => { ld!(h, a);            1 },
        0x68 => { ld!(l, b);            1 },
        0x69 => { ld!(l, c);            1 },
        0x6A => { ld!(l, d);            1 },
        0x6B => { ld!(l, e);            1 },
        0x6C => { ld!(l, h);            1 },
        0x6D => { ld!(l, l);            1 },
        0x6E => { ld_rDD!(l, hl);       2 },
        0x6F => { ld!(l, a);            1 },

        0x70 => { ld_DDr!(hl, b);       2 },
        0x71 => { ld_DDr!(hl, c);       2 },
        0x72 => { ld_DDr!(hl, d);       2 },
        0x73 => { ld_DDr!(hl, e);       2 },
        0x74 => { ld_DDr!(hl, h);       2 },
        0x75 => { ld_DDr!(hl, l);       2 },
        0x76 => { halt!();              1 },
        0x77 => { ld_DDr!(hl, a);       2 },
        0x78 => { ld!(a, b);            1 },
        0x79 => { ld!(a, c);            1 },
        0x7A => { ld!(a, d);            1 },
        0x7B => { ld!(a, e);            1 },
        0x7C => { ld!(a, h);            1 },
        0x7D => { ld!(a, l);            1 },
        0x7E => { ld_rDD!(a, hl);       2 },
        0x7F => { ld!(a, a);            1 },

        0x80 => { add_ar!(b);           1 },
        0x81 => { add_ar!(c);           1 },
        0x82 => { add_ar!(d);           1 },
        0x83 => { add_ar!(e);           1 },
        0x84 => { add_ar!(h);           1 },
        0x85 => { add_ar!(l);           1 },
        0x86 => { add_aHL!();           2 },
        0x87 => { add_ar!(a);           1 },
        0x88 => { adc_ar!(b);           1 },
        0x89 => { adc_ar!(c);           1 },
        0x8A => { adc_ar!(d);           1 },
        0x8B => { adc_ar!(e);           1 },
        0x8C => { adc_ar!(h);           1 },
        0x8D => { adc_ar!(l);           1 },
        0x8E => { adc_aHL!();           2 },
        0x8F => { adc_ar!(a);           1 },

        0x90 => { sub_ar!(b);           1 },
        0x91 => { sub_ar!(c);           1 },
        0x92 => { sub_ar!(d);           1 },
        0x93 => { sub_ar!(e);           1 },
        0x94 => { sub_ar!(h);           1 },
        0x95 => { sub_ar!(l);           1 },
        0x96 => { sub_aHL!();           2 },
        0x97 => { sub_ar!(a);           1 },
        0x98 => { sbc_ar!(b);           1 },
        0x99 => { sbc_ar!(c);           1 },
        0x9A => { sbc_ar!(d);           1 },
        0x9B => { sbc_ar!(e);           1 },
        0x9C => { sbc_ar!(h);           1 },
        0x9D => { sbc_ar!(l);           1 },
        0x9E => { sbc_aHL!();           2 },
        0x9F => { sbc_ar!(a);           1 },

        0xA0 => { and_ar!(b);           1 },
        0xA1 => { and_ar!(c);           1 },
        0xA2 => { and_ar!(d);           1 },
        0xA3 => { and_ar!(e);           1 },
        0xA4 => { and_ar!(h);           1 },
        0xA5 => { and_ar!(l);           1 },
        0xA6 => { and_aHL!();           2 },
        0xA7 => { and_ar!(a);           1 },
        0xA8 => { xor_ar!(b);           1 },
        0xA9 => { xor_ar!(c);           1 },
        0xAA => { xor_ar!(d);           1 },
        0xAB => { xor_ar!(e);           1 },
        0xAC => { xor_ar!(h);           1 },
        0xAD => { xor_ar!(l);           1 },
        0xAE => { xor_aHL!();           2 },
        0xAF => { xor_ar!(a);           1 },

        0xB0 => { or_ar!(b);            1 },
        0xB1 => { or_ar!(c);            1 },
        0xB2 => { or_ar!(d);            1 },
        0xB3 => { or_ar!(e);            1 },
        0xB4 => { or_ar!(h);            1 },
        0xB5 => { or_ar!(l);            1 },
        0xB6 => { or_aHL!();            2 },
        0xB7 => { or_ar!(a);            1 },
        0xB8 => { cp_ar!(b);            1 },
        0xB9 => { cp_ar!(c);            1 },
        0xBA => { cp_ar!(d);            1 },
        0xBB => { cp_ar!(e);            1 },
        0xBC => { cp_ar!(h);            1 },
        0xBD => { cp_ar!(l);            1 },
        0xBE => { cp_aHL!();            2 },
        0xBF => { cp_ar!(a);            1 },

        0xC0 => { ret_c!(!zflag!())       }, // 5;2
        0xC1 => { pop_qq!(bc);          3 },
        0xC2 => { jp_cnn!(!zflag!())      }, // 4;3
        0xC3 => { jp_nn!();             4 },
        0xC4 => { call_cnn!(!zflag!())    }, // 6;3
        0xC5 => { push_qq!(bc);         4 },
        0xC6 => { add_an!();            2 },
        0xC7 => { rst_p!(0x00);         4 },
        0xC8 => { ret_c!(zflag!())        }, // 5;2
        0xC9 => { ret!();               4 },
        0xCA => { jp_cnn!(zflag!())       }, // 4;3
        0xCB => exec_long(cpu, mem),
        0xCC => { call_cnn!(zflag!())     }, // 6;3
        0xCD => { call_nn!();           6 },
        0xCE => { adc_an!();            2 },
        0xCF => { rst_p!(0x08);         4 },

        0xD0 => { ret_c!(!cflag!())       }, // 5;2
        0xD1 => { pop_qq!(de);          3 },
        0xD2 => { jp_cnn!(!cflag!())      }, // 4;3
        0xD3 => cpu.invalid_inst(op),
        0xD4 => { call_cnn!(!cflag!())    }, // 6;3
        0xD5 => { push_qq!(de);         4 },
        0xD6 => { sub_an!();            2 },
        0xD7 => { rst_p!(0x10);         4 },
        0xD8 => { ret_c!(cflag!())        }, // 5;2
        0xD9 => { reti!();              4 },
        0xDA => { jp_cnn!(cflag!())       }, // 4;3
        0xDB => cpu.invalid_inst(op),
        0xDC => { call_cnn!(cflag!())     }, // 6;3
        0xDD => cpu.invalid_inst(op),
        0xDE => { sbc_an!();            2 },
        0xDF => { rst_p!(0x18);         4 },

        0xE0 => { ld_IOan!();           3 },
        0xE1 => { pop_qq!(hl);          3 },
        0xE2 => { ld_IOca!();           2 },
        0xE3 => cpu.invalid_inst(op),
        0xE4 => cpu.invalid_inst(op),
        0xE5 => { push_qq!(hl);         4 },
        0xE6 => { and_an!();            2 },
        0xE7 => { rst_p!(0x20);         4 },
        0xE8 => { add_spn!();           4 },
        0xE9 => { jp_hl!();             1 },
        0xEA => { ld_NNr!(a);           4 },
        0xEB => cpu.invalid_inst(op),
        0xEC => cpu.invalid_inst(op),
        // The GB processor does not include the ED set of instructions from the z80 processor
        0xED => cpu.invalid_inst(op),
        0xEE => { xor_an!();            2 },
        0xEF => { rst_p!(0x28);         4 },

        0xF0 => { ld_aIOn!();           3 },
        0xF1 => { pop_af!();            3 },
        0xF2 => { ld_aIOc!();           2 },
        0xF3 => { di!();                1 },
        0xF4 => cpu.invalid_inst(op),
        0xF5 => { push_qq!(af);         4 },
        0xF6 => { or_an!();             2 },
        0xF7 => { rst_p!(0x30);         4 },
        0xF8 => { ld_hlspn!();          3 },
        0xF9 => { ld_sphl!();           2 },
        0xFA => { ld_rNN!(a);           4 },
        0xFB => { ei!();                1 },
        0xFC => cpu.invalid_inst(op),
        0xFD => cpu.invalid_inst(op),
        0xFE => { cp_an!();             2 },
        0xFF => { rst_p!(0x38);         4 },

        _ => unreachable!()
    }
}

/// Execute 2-byte opcodes, returning the number of cycles required for executing the
/// instruction
fn exec_long(cpu: &mut Cpu, mem: &mut Memory) -> u8 {
    //
    // Implementation of z80 two byte instructions
    //

    // RLC r
    macro_rules! rlc_r { ($r: ident) => ({
        let copy_bit = cpu.$r >> 7;
        cpu.$r = (cpu.$r << 1) + copy_bit;
        cpu.f = if copy_bit == 0 { 0 } else { CARRY_FLAG };

        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // RLC (HL)
    macro_rules! rlc_HL { () => ({
        let addr = cpu.hl().get();
        let val = mem.lb(addr);

        let copy_bit = val >> 7;
        cpu.f = if copy_bit == 0 { 0 } else { CARRY_FLAG };

        mem.sb(addr, (val << 1) + copy_bit);

        if (val << 1) + copy_bit == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // RL r
    macro_rules! rl_r { ($r: ident) => ({
        let copy_bit = if cpu.f & CARRY_FLAG == 0 { 0 } else { 1 };
        cpu.f = if cpu.$r >> 7 == 0 { 0 } else { CARRY_FLAG };
        cpu.$r = (cpu.$r << 1) + copy_bit;

        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // RL (HL)
    macro_rules! rl_HL { () => ({
        let addr = cpu.hl().get();
        let val = mem.lb(addr);

        let copy_bit = if cpu.f & CARRY_FLAG == 0 { 0 } else { 1 };
        cpu.f = if val >> 7 == 0 { 0 } else { CARRY_FLAG };
        mem.sb(addr, (val << 1) + copy_bit);

        if (val << 1) + copy_bit == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // RRC r
    macro_rules! rrc_r { ($r: ident) => ({
        let copy_bit = cpu.$r & 0x1;
        cpu.f = if copy_bit == 0 { 0 } else { CARRY_FLAG };
        cpu.$r = (cpu.$r >> 1) + (copy_bit << 7);

        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // RRC (HL)
    macro_rules! rrc_HL { () => ({
        let addr = cpu.hl().get();
        let val = mem.lb(addr);

        let copy_bit = val & 0x1;
        cpu.f = if copy_bit == 0 { 0 } else { CARRY_FLAG };
        mem.sb(addr, (val >> 1) + (copy_bit << 7));

        if (val << 1) + copy_bit == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // RR r
    macro_rules! rr_r { ($r: ident) => ({
        let copy_bit = if cpu.f & CARRY_FLAG == 0 { 0 } else { 1 };
        cpu.f = if cpu.$r & 0x1 == 0 { 0 } else { CARRY_FLAG };
        cpu.$r = (cpu.$r >> 1) + (copy_bit << 7);

        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // RR (HL)
    macro_rules! rr_HL { () => ({
        let addr = cpu.hl().get();
        let val = mem.lb(addr);

        let copy_bit = if cpu.f & CARRY_FLAG == 0 { 0 } else { 1 };
        cpu.f = if val & 0x1 == 0 { 0 } else { CARRY_FLAG };
        mem.sb(addr, (val >> 1) + (copy_bit << 7));

        if (val >> 1) + (copy_bit << 7) == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // SLA r
    macro_rules! sla_r { ($r: ident) => ({
        cpu.f = if cpu.$r >> 7 == 0 { 0 } else { CARRY_FLAG };
        cpu.$r = cpu.$r << 1;
        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // SLA (HL)
    macro_rules! sla_HL { () => ({
        let addr = cpu.hl().get();
        let val = mem.lb(addr);

        cpu.f = if val >> 7 == 0 { 0 } else { CARRY_FLAG };
        mem.sb(addr, val << 1);
        if (val << 1) == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // SRA r
    macro_rules! sra_r { ($r: ident) => ({
        cpu.f = if cpu.$r & 1 == 0 { 0 } else { CARRY_FLAG };
        cpu.$r = (cpu.$r as i8 >> 1) as u8;
        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // SRA (HL)
    macro_rules! sra_HL { () => ({
        let addr = cpu.hl().get();
        let mut val = mem.lb(addr);

        cpu.f = if val & 1 == 0 { 0 } else { CARRY_FLAG };
        val = (val as i8 >> 1) as u8;
        mem.sb(addr, val);
        if val == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // SRL r
    macro_rules! srl_r { ($r: ident) => ({
        cpu.f = if cpu.$r & 1 == 0 { 0 } else { CARRY_FLAG };
        cpu.$r = cpu.$r >> 1;
        if cpu.$r == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // SRL (HL)
    macro_rules! srl_HL { () => ({
        let addr = cpu.hl().get();
        let mut val = mem.lb(addr);

        cpu.f = if val & 1 == 0 { 0 } else { CARRY_FLAG };
        val = val >> 1;
        mem.sb(addr, val);
        if val == 0 { cpu.f |= ZERO_FLAG; }
    }) }

    // BIT b, m
    macro_rules! bit_b { ($b: expr, $m: expr) => ({
        cpu.f |= HALF_CARRY_FLAG;
        cpu.f &= !ADD_SUB_FLAG;
        if $m & (1 << $b) == 0 { cpu.f |= ZERO_FLAG; } else { cpu.f &= !ZERO_FLAG; }
    }) }
    macro_rules! bit_br { ($b: expr, $r: ident) => (bit_b!($b, cpu.$r)) }
    macro_rules! bit_bHL { ($b: expr) => (bit_b!($b, mem.lb(cpu.hl().get()))) }

    // SET b, r
    macro_rules! set_br { ($b: expr, $r: ident) => ({
        cpu.$r |= 1 << $b;
    }) }

    // SET b, (HL)
    macro_rules! set_bHL { ($b: expr) => ({
        let addr = cpu.hl().get();
        let val = mem.lb(addr) | (1 << $b);
        mem.sb(addr, val);
    }) }

    // RES b, r
    macro_rules! res_br { ($b: expr, $r: ident) => ({
        cpu.$r &= !(1 << $b);
    }) }

    // RES b, (HL)
    macro_rules! res_bHL { ($b: expr) => ({
        let addr = cpu.hl().get();
        let val = mem.lb(addr) & !(1 << $b);
        mem.sb(addr, val);
    }) }

    // SWAP r
    macro_rules! swap_r { ($r: ident) => ({
        cpu.$r = (cpu.$r << 4) | (cpu.$r >> 4);
    }) }

    // SWAP HL
    macro_rules! swap_HL { () => ({
        let addr = cpu.hl().get();
        let val = mem.lb(addr);
        mem.sb(addr, (val << 4) | (val >> 4));
    }) }

    // Fetch the second byte of the instruction
    let op = mem.lb(cpu.bump());

    match op {
        0x00 => { rlc_r!(b);            2 },
        0x01 => { rlc_r!(c);            2 },
        0x02 => { rlc_r!(d);            2 },
        0x03 => { rlc_r!(e);            2 },
        0x04 => { rlc_r!(h);            2 },
        0x05 => { rlc_r!(l);            2 },
        0x06 => { rlc_HL!();            4 },
        0x07 => { rlc_r!(a);            2 },
        0x08 => { rrc_r!(b);            2 },
        0x09 => { rrc_r!(c);            2 },
        0x0A => { rrc_r!(d);            2 },
        0x0B => { rrc_r!(e);            2 },
        0x0C => { rrc_r!(h);            2 },
        0x0D => { rrc_r!(l);            2 },
        0x0E => { rrc_HL!();            4 },
        0x0F => { rrc_r!(a);            2 },

        0x10 => { rl_r!(b);             2 },
        0x11 => { rl_r!(c);             2 },
        0x12 => { rl_r!(d);             2 },
        0x13 => { rl_r!(e);             2 },
        0x14 => { rl_r!(h);             2 },
        0x15 => { rl_r!(l);             2 },
        0x16 => { rl_HL!();             4 },
        0x17 => { rl_r!(a);             2 },
        0x18 => { rr_r!(b);             2 },
        0x19 => { rr_r!(c);             2 },
        0x1A => { rr_r!(d);             2 },
        0x1B => { rr_r!(e);             2 },
        0x1C => { rr_r!(h);             2 },
        0x1D => { rr_r!(l);             2 },
        0x1E => { rr_HL!();             4 },
        0x1F => { rr_r!(a);             2 },

        0x20 => { sla_r!(b);            2 },
        0x21 => { sla_r!(c);            2 },
        0x22 => { sla_r!(d);            2 },
        0x23 => { sla_r!(e);            2 },
        0x24 => { sla_r!(h);            2 },
        0x25 => { sla_r!(l);            2 },
        0x26 => { sla_HL!();            4 },
        0x27 => { sla_r!(a);            2 },
        0x28 => { sra_r!(b);            2 },
        0x29 => { sra_r!(c);            2 },
        0x2A => { sra_r!(d);            2 },
        0x2B => { sra_r!(e);            2 },
        0x2C => { sra_r!(h);            2 },
        0x2D => { sra_r!(l);            2 },
        0x2E => { sra_HL!();            4 },
        0x2F => { sra_r!(a);            2 },

        0x30 => { swap_r!(b);           2 },
        0x31 => { swap_r!(c);           2 },
        0x32 => { swap_r!(d);           2 },
        0x33 => { swap_r!(e);           2 },
        0x34 => { swap_r!(h);           2 },
        0x35 => { swap_r!(l);           2 },
        0x36 => { swap_HL!();           4 },
        0x37 => { swap_r!(a);           2 },
        0x38 => { srl_r!(b);            2 },
        0x39 => { srl_r!(c);            2 },
        0x3A => { srl_r!(d);            2 },
        0x3B => { srl_r!(e);            2 },
        0x3C => { srl_r!(h);            2 },
        0x3D => { srl_r!(l);            2 },
        0x3E => { srl_HL!();            4 },
        0x3F => { srl_r!(a);            2 },

        0x40 => { bit_br!(0, b);        2 },
        0x41 => { bit_br!(0, c);        2 },
        0x42 => { bit_br!(0, d);        2 },
        0x43 => { bit_br!(0, e);        2 },
        0x44 => { bit_br!(0, h);        2 },
        0x45 => { bit_br!(0, l);        2 },
        0x46 => { bit_bHL!(0);          3 },
        0x47 => { bit_br!(0, a);        2 },
        0x48 => { bit_br!(1, b);        2 },
        0x49 => { bit_br!(1, c);        2 },
        0x4A => { bit_br!(1, d);        2 },
        0x4B => { bit_br!(1, e);        2 },
        0x4C => { bit_br!(1, h);        2 },
        0x4D => { bit_br!(1, l);        2 },
        0x4E => { bit_bHL!(1);          3 },
        0x4F => { bit_br!(1, a);        2 },

        0x50 => { bit_br!(2, b);        2 },
        0x51 => { bit_br!(2, c);        2 },
        0x52 => { bit_br!(2, d);        2 },
        0x53 => { bit_br!(2, e);        2 },
        0x54 => { bit_br!(2, h);        2 },
        0x55 => { bit_br!(2, l);        2 },
        0x56 => { bit_bHL!(2);          3 },
        0x57 => { bit_br!(2, a);        2 },
        0x58 => { bit_br!(3, b);        2 },
        0x59 => { bit_br!(3, c);        2 },
        0x5A => { bit_br!(3, d);        2 },
        0x5B => { bit_br!(3, e);        2 },
        0x5C => { bit_br!(3, h);        2 },
        0x5D => { bit_br!(3, l);        2 },
        0x5E => { bit_bHL!(3);          3 },
        0x5F => { bit_br!(3, a);        2 },

        0x60 => { bit_br!(4, b);        2 },
        0x61 => { bit_br!(4, c);        2 },
        0x62 => { bit_br!(4, d);        2 },
        0x63 => { bit_br!(4, e);        2 },
        0x64 => { bit_br!(4, h);        2 },
        0x65 => { bit_br!(4, l);        2 },
        0x66 => { bit_bHL!(4);          3 },
        0x67 => { bit_br!(4, a);        2 },
        0x68 => { bit_br!(5, b);        2 },
        0x69 => { bit_br!(5, c);        2 },
        0x6A => { bit_br!(5, d);        2 },
        0x6B => { bit_br!(5, e);        2 },
        0x6C => { bit_br!(5, h);        2 },
        0x6D => { bit_br!(5, l);        2 },
        0x6E => { bit_bHL!(5);          3 },
        0x6F => { bit_br!(5, a);        2 },

        0x70 => { bit_br!(6, b);        2 },
        0x71 => { bit_br!(6, c);        2 },
        0x72 => { bit_br!(6, d);        2 },
        0x73 => { bit_br!(6, e);        2 },
        0x74 => { bit_br!(6, h);        2 },
        0x75 => { bit_br!(6, l);        2 },
        0x76 => { bit_bHL!(6);          3 },
        0x77 => { bit_br!(6, a);        2 },
        0x78 => { bit_br!(7, b);        2 },
        0x79 => { bit_br!(7, c);        2 },
        0x7A => { bit_br!(7, d);        2 },
        0x7B => { bit_br!(7, e);        2 },
        0x7C => { bit_br!(7, h);        2 },
        0x7D => { bit_br!(7, l);        2 },
        0x7E => { bit_bHL!(7);          3 },
        0x7F => { bit_br!(7, a);        2 },

        0x80 => { res_br!(0, b);        2 },
        0x81 => { res_br!(0, c);        2 },
        0x82 => { res_br!(0, d);        2 },
        0x83 => { res_br!(0, e);        2 },
        0x84 => { res_br!(0, h);        2 },
        0x85 => { res_br!(0, l);        2 },
        0x86 => { res_bHL!(0);          4 },
        0x87 => { res_br!(0, a);        2 },
        0x88 => { res_br!(1, b);        2 },
        0x89 => { res_br!(1, c);        2 },
        0x8A => { res_br!(1, d);        2 },
        0x8B => { res_br!(1, e);        2 },
        0x8C => { res_br!(1, h);        2 },
        0x8D => { res_br!(1, l);        2 },
        0x8E => { res_bHL!(1);          4 },
        0x8F => { res_br!(1, a);        2 },

        0x90 => { res_br!(2, b);        2 },
        0x91 => { res_br!(2, c);        2 },
        0x92 => { res_br!(2, d);        2 },
        0x93 => { res_br!(2, e);        2 },
        0x94 => { res_br!(2, h);        2 },
        0x95 => { res_br!(2, l);        2 },
        0x96 => { res_bHL!(2);          4 },
        0x97 => { res_br!(2, a);        2 },
        0x98 => { res_br!(3, b);        2 },
        0x99 => { res_br!(3, c);        2 },
        0x9A => { res_br!(3, d);        2 },
        0x9B => { res_br!(3, e);        2 },
        0x9C => { res_br!(3, h);        2 },
        0x9D => { res_br!(3, l);        2 },
        0x9E => { res_bHL!(3);          4 },
        0x9F => { res_br!(3, a);        2 },

        0xA0 => { res_br!(4, b);        2 },
        0xA1 => { res_br!(4, c);        2 },
        0xA2 => { res_br!(4, d);        2 },
        0xA3 => { res_br!(4, e);        2 },
        0xA4 => { res_br!(4, h);        2 },
        0xA5 => { res_br!(4, l);        2 },
        0xA6 => { res_bHL!(4);          4 },
        0xA7 => { res_br!(4, a);        2 },
        0xA8 => { res_br!(5, b);        2 },
        0xA9 => { res_br!(5, c);        2 },
        0xAA => { res_br!(5, d);        2 },
        0xAB => { res_br!(5, e);        2 },
        0xAC => { res_br!(5, h);        2 },
        0xAD => { res_br!(5, l);        2 },
        0xAE => { res_bHL!(5);          4 },
        0xAF => { res_br!(5, a);        2 },

        0xB0 => { res_br!(6, b);        2 },
        0xB1 => { res_br!(6, c);        2 },
        0xB2 => { res_br!(6, d);        2 },
        0xB3 => { res_br!(6, e);        2 },
        0xB4 => { res_br!(6, h);        2 },
        0xB5 => { res_br!(6, l);        2 },
        0xB6 => { res_bHL!(6);          4 },
        0xB7 => { res_br!(6, a);        2 },
        0xB8 => { res_br!(7, b);        2 },
        0xB9 => { res_br!(7, c);        2 },
        0xBA => { res_br!(7, d);        2 },
        0xBB => { res_br!(7, e);        2 },
        0xBC => { res_br!(7, h);        2 },
        0xBD => { res_br!(7, l);        2 },
        0xBE => { res_bHL!(7);          4 },
        0xBF => { res_br!(7, a);        2 },

        0xC0 => { set_br!(0, b);        2 },
        0xC1 => { set_br!(0, c);        2 },
        0xC2 => { set_br!(0, d);        2 },
        0xC3 => { set_br!(0, e);        2 },
        0xC4 => { set_br!(0, h);        2 },
        0xC5 => { set_br!(0, l);        2 },
        0xC6 => { set_bHL!(0);          4 },
        0xC7 => { set_br!(0, a);        2 },
        0xC8 => { set_br!(1, b);        2 },
        0xC9 => { set_br!(1, c);        2 },
        0xCA => { set_br!(1, d);        2 },
        0xCB => { set_br!(1, e);        2 },
        0xCC => { set_br!(1, h);        2 },
        0xCD => { set_br!(1, l);        2 },
        0xCE => { set_bHL!(1);          4 },
        0xCF => { set_br!(1, a);        2 },

        0xD0 => { set_br!(2, b);        2 },
        0xD1 => { set_br!(2, c);        2 },
        0xD2 => { set_br!(2, d);        2 },
        0xD3 => { set_br!(2, e);        2 },
        0xD4 => { set_br!(2, h);        2 },
        0xD5 => { set_br!(2, l);        2 },
        0xD6 => { set_bHL!(2);          4 },
        0xD7 => { set_br!(2, a);        2 },
        0xD8 => { set_br!(3, b);        2 },
        0xD9 => { set_br!(3, c);        2 },
        0xDA => { set_br!(3, d);        2 },
        0xDB => { set_br!(3, e);        2 },
        0xDC => { set_br!(3, h);        2 },
        0xDD => { set_br!(3, l);        2 },
        0xDE => { set_bHL!(3);          4 },
        0xDF => { set_br!(3, a);        2 },

        0xE0 => { set_br!(4, b);        2 },
        0xE1 => { set_br!(4, c);        2 },
        0xE2 => { set_br!(4, d);        2 },
        0xE3 => { set_br!(4, e);        2 },
        0xE4 => { set_br!(4, h);        2 },
        0xE5 => { set_br!(4, l);        2 },
        0xE6 => { set_bHL!(4);          4 },
        0xE7 => { set_br!(4, a);        2 },
        0xE8 => { set_br!(5, b);        2 },
        0xE9 => { set_br!(5, c);        2 },
        0xEA => { set_br!(5, d);        2 },
        0xEB => { set_br!(5, e);        2 },
        0xEC => { set_br!(5, h);        2 },
        0xED => { set_br!(5, l);        2 },
        0xEE => { set_bHL!(5);          4 },
        0xEF => { set_br!(5, a);        2 },

        0xF0 => { set_br!(6, b);        2 },
        0xF1 => { set_br!(6, c);        2 },
        0xF2 => { set_br!(6, d);        2 },
        0xF3 => { set_br!(6, e);        2 },
        0xF4 => { set_br!(6, h);        2 },
        0xF5 => { set_br!(6, l);        2 },
        0xF6 => { set_bHL!(6);          4 },
        0xF7 => { set_br!(6, a);        2 },
        0xF8 => { set_br!(7, b);        2 },
        0xF9 => { set_br!(7, c);        2 },
        0xFA => { set_br!(7, d);        2 },
        0xFB => { set_br!(7, e);        2 },
        0xFC => { set_br!(7, h);        2 },
        0xFD => { set_br!(7, l);        2 },
        0xFE => { set_bHL!(7);          4 },
        0xFF => { set_br!(7, a);        2 },

        _ => unreachable!()
    }
}

/// Implementation of DAA instruction. This instruction conditionally adjusts the `a` register for
/// BCD addition and subtractions.
fn daa(cpu: &mut Cpu) {
    // DAA lookup table, we encode and decode the values in the following way:
    // |  0000  0NCH  AAAA AAAA | -> | AAAA AAAA FFFF FFFF |
    // Where N = ADD_SUB_FLAG, C = CARRY_FLAG, H = HALF_CARRY_FLAG, AAAA AAAA = register `a`,
    // FFFF FFFF = register `f`
    const DAA_TABLE: [u16, ..2048] = [
        0x0080,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700,
        0x0800,0x0900,0x1000,0x1100,0x1200,0x1300,0x1400,0x1500,
        0x1000,0x1100,0x1200,0x1300,0x1400,0x1500,0x1600,0x1700,
        0x1800,0x1900,0x2000,0x2100,0x2200,0x2300,0x2400,0x2500,
        0x2000,0x2100,0x2200,0x2300,0x2400,0x2500,0x2600,0x2700,
        0x2800,0x2900,0x3000,0x3100,0x3200,0x3300,0x3400,0x3500,
        0x3000,0x3100,0x3200,0x3300,0x3400,0x3500,0x3600,0x3700,
        0x3800,0x3900,0x4000,0x4100,0x4200,0x4300,0x4400,0x4500,
        0x4000,0x4100,0x4200,0x4300,0x4400,0x4500,0x4600,0x4700,
        0x4800,0x4900,0x5000,0x5100,0x5200,0x5300,0x5400,0x5500,
        0x5000,0x5100,0x5200,0x5300,0x5400,0x5500,0x5600,0x5700,
        0x5800,0x5900,0x6000,0x6100,0x6200,0x6300,0x6400,0x6500,
        0x6000,0x6100,0x6200,0x6300,0x6400,0x6500,0x6600,0x6700,
        0x6800,0x6900,0x7000,0x7100,0x7200,0x7300,0x7400,0x7500,
        0x7000,0x7100,0x7200,0x7300,0x7400,0x7500,0x7600,0x7700,
        0x7800,0x7900,0x8000,0x8100,0x8200,0x8300,0x8400,0x8500,
        0x8000,0x8100,0x8200,0x8300,0x8400,0x8500,0x8600,0x8700,
        0x8800,0x8900,0x9000,0x9100,0x9200,0x9300,0x9400,0x9500,
        0x9000,0x9100,0x9200,0x9300,0x9400,0x9500,0x9600,0x9700,
        0x9800,0x9900,0x0090,0x0110,0x0210,0x0310,0x0410,0x0510,
        0x0090,0x0110,0x0210,0x0310,0x0410,0x0510,0x0610,0x0710,
        0x0810,0x0910,0x1010,0x1110,0x1210,0x1310,0x1410,0x1510,
        0x1010,0x1110,0x1210,0x1310,0x1410,0x1510,0x1610,0x1710,
        0x1810,0x1910,0x2010,0x2110,0x2210,0x2310,0x2410,0x2510,
        0x2010,0x2110,0x2210,0x2310,0x2410,0x2510,0x2610,0x2710,
        0x2810,0x2910,0x3010,0x3110,0x3210,0x3310,0x3410,0x3510,
        0x3010,0x3110,0x3210,0x3310,0x3410,0x3510,0x3610,0x3710,
        0x3810,0x3910,0x4010,0x4110,0x4210,0x4310,0x4410,0x4510,
        0x4010,0x4110,0x4210,0x4310,0x4410,0x4510,0x4610,0x4710,
        0x4810,0x4910,0x5010,0x5110,0x5210,0x5310,0x5410,0x5510,
        0x5010,0x5110,0x5210,0x5310,0x5410,0x5510,0x5610,0x5710,
        0x5810,0x5910,0x6010,0x6110,0x6210,0x6310,0x6410,0x6510,
        0x6010,0x6110,0x6210,0x6310,0x6410,0x6510,0x6610,0x6710,
        0x6810,0x6910,0x7010,0x7110,0x7210,0x7310,0x7410,0x7510,
        0x7010,0x7110,0x7210,0x7310,0x7410,0x7510,0x7610,0x7710,
        0x7810,0x7910,0x8010,0x8110,0x8210,0x8310,0x8410,0x8510,
        0x8010,0x8110,0x8210,0x8310,0x8410,0x8510,0x8610,0x8710,
        0x8810,0x8910,0x9010,0x9110,0x9210,0x9310,0x9410,0x9510,
        0x9010,0x9110,0x9210,0x9310,0x9410,0x9510,0x9610,0x9710,
        0x9810,0x9910,0xA010,0xA110,0xA210,0xA310,0xA410,0xA510,
        0xA010,0xA110,0xA210,0xA310,0xA410,0xA510,0xA610,0xA710,
        0xA810,0xA910,0xB010,0xB110,0xB210,0xB310,0xB410,0xB510,
        0xB010,0xB110,0xB210,0xB310,0xB410,0xB510,0xB610,0xB710,
        0xB810,0xB910,0xC010,0xC110,0xC210,0xC310,0xC410,0xC510,
        0xC010,0xC110,0xC210,0xC310,0xC410,0xC510,0xC610,0xC710,
        0xC810,0xC910,0xD010,0xD110,0xD210,0xD310,0xD410,0xD510,
        0xD010,0xD110,0xD210,0xD310,0xD410,0xD510,0xD610,0xD710,
        0xD810,0xD910,0xE010,0xE110,0xE210,0xE310,0xE410,0xE510,
        0xE010,0xE110,0xE210,0xE310,0xE410,0xE510,0xE610,0xE710,
        0xE810,0xE910,0xF010,0xF110,0xF210,0xF310,0xF410,0xF510,
        0xF010,0xF110,0xF210,0xF310,0xF410,0xF510,0xF610,0xF710,
        0xF810,0xF910,0x0090,0x0110,0x0210,0x0310,0x0410,0x0510,
        0x0090,0x0110,0x0210,0x0310,0x0410,0x0510,0x0610,0x0710,
        0x0810,0x0910,0x1010,0x1110,0x1210,0x1310,0x1410,0x1510,
        0x1010,0x1110,0x1210,0x1310,0x1410,0x1510,0x1610,0x1710,
        0x1810,0x1910,0x2010,0x2110,0x2210,0x2310,0x2410,0x2510,
        0x2010,0x2110,0x2210,0x2310,0x2410,0x2510,0x2610,0x2710,
        0x2810,0x2910,0x3010,0x3110,0x3210,0x3310,0x3410,0x3510,
        0x3010,0x3110,0x3210,0x3310,0x3410,0x3510,0x3610,0x3710,
        0x3810,0x3910,0x4010,0x4110,0x4210,0x4310,0x4410,0x4510,
        0x4010,0x4110,0x4210,0x4310,0x4410,0x4510,0x4610,0x4710,
        0x4810,0x4910,0x5010,0x5110,0x5210,0x5310,0x5410,0x5510,
        0x5010,0x5110,0x5210,0x5310,0x5410,0x5510,0x5610,0x5710,
        0x5810,0x5910,0x6010,0x6110,0x6210,0x6310,0x6410,0x6510,
        0x0600,0x0700,0x0800,0x0900,0x0A00,0x0B00,0x0C00,0x0D00,
        0x0E00,0x0F00,0x1000,0x1100,0x1200,0x1300,0x1400,0x1500,
        0x1600,0x1700,0x1800,0x1900,0x1A00,0x1B00,0x1C00,0x1D00,
        0x1E00,0x1F00,0x2000,0x2100,0x2200,0x2300,0x2400,0x2500,
        0x2600,0x2700,0x2800,0x2900,0x2A00,0x2B00,0x2C00,0x2D00,
        0x2E00,0x2F00,0x3000,0x3100,0x3200,0x3300,0x3400,0x3500,
        0x3600,0x3700,0x3800,0x3900,0x3A00,0x3B00,0x3C00,0x3D00,
        0x3E00,0x3F00,0x4000,0x4100,0x4200,0x4300,0x4400,0x4500,
        0x4600,0x4700,0x4800,0x4900,0x4A00,0x4B00,0x4C00,0x4D00,
        0x4E00,0x4F00,0x5000,0x5100,0x5200,0x5300,0x5400,0x5500,
        0x5600,0x5700,0x5800,0x5900,0x5A00,0x5B00,0x5C00,0x5D00,
        0x5E00,0x5F00,0x6000,0x6100,0x6200,0x6300,0x6400,0x6500,
        0x6600,0x6700,0x6800,0x6900,0x6A00,0x6B00,0x6C00,0x6D00,
        0x6E00,0x6F00,0x7000,0x7100,0x7200,0x7300,0x7400,0x7500,
        0x7600,0x7700,0x7800,0x7900,0x7A00,0x7B00,0x7C00,0x7D00,
        0x7E00,0x7F00,0x8000,0x8100,0x8200,0x8300,0x8400,0x8500,
        0x8600,0x8700,0x8800,0x8900,0x8A00,0x8B00,0x8C00,0x8D00,
        0x8E00,0x8F00,0x9000,0x9100,0x9200,0x9300,0x9400,0x9500,
        0x9600,0x9700,0x9800,0x9900,0x9A00,0x9B00,0x9C00,0x9D00,
        0x9E00,0x9F00,0x0090,0x0110,0x0210,0x0310,0x0410,0x0510,
        0x0610,0x0710,0x0810,0x0910,0x0A10,0x0B10,0x0C10,0x0D10,
        0x0E10,0x0F10,0x1010,0x1110,0x1210,0x1310,0x1410,0x1510,
        0x1610,0x1710,0x1810,0x1910,0x1A10,0x1B10,0x1C10,0x1D10,
        0x1E10,0x1F10,0x2010,0x2110,0x2210,0x2310,0x2410,0x2510,
        0x2610,0x2710,0x2810,0x2910,0x2A10,0x2B10,0x2C10,0x2D10,
        0x2E10,0x2F10,0x3010,0x3110,0x3210,0x3310,0x3410,0x3510,
        0x3610,0x3710,0x3810,0x3910,0x3A10,0x3B10,0x3C10,0x3D10,
        0x3E10,0x3F10,0x4010,0x4110,0x4210,0x4310,0x4410,0x4510,
        0x4610,0x4710,0x4810,0x4910,0x4A10,0x4B10,0x4C10,0x4D10,
        0x4E10,0x4F10,0x5010,0x5110,0x5210,0x5310,0x5410,0x5510,
        0x5610,0x5710,0x5810,0x5910,0x5A10,0x5B10,0x5C10,0x5D10,
        0x5E10,0x5F10,0x6010,0x6110,0x6210,0x6310,0x6410,0x6510,
        0x6610,0x6710,0x6810,0x6910,0x6A10,0x6B10,0x6C10,0x6D10,
        0x6E10,0x6F10,0x7010,0x7110,0x7210,0x7310,0x7410,0x7510,
        0x7610,0x7710,0x7810,0x7910,0x7A10,0x7B10,0x7C10,0x7D10,
        0x7E10,0x7F10,0x8010,0x8110,0x8210,0x8310,0x8410,0x8510,
        0x8610,0x8710,0x8810,0x8910,0x8A10,0x8B10,0x8C10,0x8D10,
        0x8E10,0x8F10,0x9010,0x9110,0x9210,0x9310,0x9410,0x9510,
        0x9610,0x9710,0x9810,0x9910,0x9A10,0x9B10,0x9C10,0x9D10,
        0x9E10,0x9F10,0xA010,0xA110,0xA210,0xA310,0xA410,0xA510,
        0xA610,0xA710,0xA810,0xA910,0xAA10,0xAB10,0xAC10,0xAD10,
        0xAE10,0xAF10,0xB010,0xB110,0xB210,0xB310,0xB410,0xB510,
        0xB610,0xB710,0xB810,0xB910,0xBA10,0xBB10,0xBC10,0xBD10,
        0xBE10,0xBF10,0xC010,0xC110,0xC210,0xC310,0xC410,0xC510,
        0xC610,0xC710,0xC810,0xC910,0xCA10,0xCB10,0xCC10,0xCD10,
        0xCE10,0xCF10,0xD010,0xD110,0xD210,0xD310,0xD410,0xD510,
        0xD610,0xD710,0xD810,0xD910,0xDA10,0xDB10,0xDC10,0xDD10,
        0xDE10,0xDF10,0xE010,0xE110,0xE210,0xE310,0xE410,0xE510,
        0xE610,0xE710,0xE810,0xE910,0xEA10,0xEB10,0xEC10,0xED10,
        0xEE10,0xEF10,0xF010,0xF110,0xF210,0xF310,0xF410,0xF510,
        0xF610,0xF710,0xF810,0xF910,0xFA10,0xFB10,0xFC10,0xFD10,
        0xFE10,0xFF10,0x0090,0x0110,0x0210,0x0310,0x0410,0x0510,
        0x0610,0x0710,0x0810,0x0910,0x0A10,0x0B10,0x0C10,0x0D10,
        0x0E10,0x0F10,0x1010,0x1110,0x1210,0x1310,0x1410,0x1510,
        0x1610,0x1710,0x1810,0x1910,0x1A10,0x1B10,0x1C10,0x1D10,
        0x1E10,0x1F10,0x2010,0x2110,0x2210,0x2310,0x2410,0x2510,
        0x2610,0x2710,0x2810,0x2910,0x2A10,0x2B10,0x2C10,0x2D10,
        0x2E10,0x2F10,0x3010,0x3110,0x3210,0x3310,0x3410,0x3510,
        0x3610,0x3710,0x3810,0x3910,0x3A10,0x3B10,0x3C10,0x3D10,
        0x3E10,0x3F10,0x4010,0x4110,0x4210,0x4310,0x4410,0x4510,
        0x4610,0x4710,0x4810,0x4910,0x4A10,0x4B10,0x4C10,0x4D10,
        0x4E10,0x4F10,0x5010,0x5110,0x5210,0x5310,0x5410,0x5510,
        0x5610,0x5710,0x5810,0x5910,0x5A10,0x5B10,0x5C10,0x5D10,
        0x5E10,0x5F10,0x6010,0x6110,0x6210,0x6310,0x6410,0x6510,
        0x00C0,0x0140,0x0240,0x0340,0x0440,0x0540,0x0640,0x0740,
        0x0840,0x0940,0x0A40,0x0B40,0x0C40,0x0D40,0x0E40,0x0F40,
        0x1040,0x1140,0x1240,0x1340,0x1440,0x1540,0x1640,0x1740,
        0x1840,0x1940,0x1A40,0x1B40,0x1C40,0x1D40,0x1E40,0x1F40,
        0x2040,0x2140,0x2240,0x2340,0x2440,0x2540,0x2640,0x2740,
        0x2840,0x2940,0x2A40,0x2B40,0x2C40,0x2D40,0x2E40,0x2F40,
        0x3040,0x3140,0x3240,0x3340,0x3440,0x3540,0x3640,0x3740,
        0x3840,0x3940,0x3A40,0x3B40,0x3C40,0x3D40,0x3E40,0x3F40,
        0x4040,0x4140,0x4240,0x4340,0x4440,0x4540,0x4640,0x4740,
        0x4840,0x4940,0x4A40,0x4B40,0x4C40,0x4D40,0x4E40,0x4F40,
        0x5040,0x5140,0x5240,0x5340,0x5440,0x5540,0x5640,0x5740,
        0x5840,0x5940,0x5A40,0x5B40,0x5C40,0x5D40,0x5E40,0x5F40,
        0x6040,0x6140,0x6240,0x6340,0x6440,0x6540,0x6640,0x6740,
        0x6840,0x6940,0x6A40,0x6B40,0x6C40,0x6D40,0x6E40,0x6F40,
        0x7040,0x7140,0x7240,0x7340,0x7440,0x7540,0x7640,0x7740,
        0x7840,0x7940,0x7A40,0x7B40,0x7C40,0x7D40,0x7E40,0x7F40,
        0x8040,0x8140,0x8240,0x8340,0x8440,0x8540,0x8640,0x8740,
        0x8840,0x8940,0x8A40,0x8B40,0x8C40,0x8D40,0x8E40,0x8F40,
        0x9040,0x9140,0x9240,0x9340,0x9440,0x9540,0x9640,0x9740,
        0x9840,0x9940,0x9A40,0x9B40,0x9C40,0x9D40,0x9E40,0x9F40,
        0xA040,0xA140,0xA240,0xA340,0xA440,0xA540,0xA640,0xA740,
        0xA840,0xA940,0xAA40,0xAB40,0xAC40,0xAD40,0xAE40,0xAF40,
        0xB040,0xB140,0xB240,0xB340,0xB440,0xB540,0xB640,0xB740,
        0xB840,0xB940,0xBA40,0xBB40,0xBC40,0xBD40,0xBE40,0xBF40,
        0xC040,0xC140,0xC240,0xC340,0xC440,0xC540,0xC640,0xC740,
        0xC840,0xC940,0xCA40,0xCB40,0xCC40,0xCD40,0xCE40,0xCF40,
        0xD040,0xD140,0xD240,0xD340,0xD440,0xD540,0xD640,0xD740,
        0xD840,0xD940,0xDA40,0xDB40,0xDC40,0xDD40,0xDE40,0xDF40,
        0xE040,0xE140,0xE240,0xE340,0xE440,0xE540,0xE640,0xE740,
        0xE840,0xE940,0xEA40,0xEB40,0xEC40,0xED40,0xEE40,0xEF40,
        0xF040,0xF140,0xF240,0xF340,0xF440,0xF540,0xF640,0xF740,
        0xF840,0xF940,0xFA40,0xFB40,0xFC40,0xFD40,0xFE40,0xFF40,
        0xA050,0xA150,0xA250,0xA350,0xA450,0xA550,0xA650,0xA750,
        0xA850,0xA950,0xAA50,0xAB50,0xAC50,0xAD50,0xAE50,0xAF50,
        0xB050,0xB150,0xB250,0xB350,0xB450,0xB550,0xB650,0xB750,
        0xB850,0xB950,0xBA50,0xBB50,0xBC50,0xBD50,0xBE50,0xBF50,
        0xC050,0xC150,0xC250,0xC350,0xC450,0xC550,0xC650,0xC750,
        0xC850,0xC950,0xCA50,0xCB50,0xCC50,0xCD50,0xCE50,0xCF50,
        0xD050,0xD150,0xD250,0xD350,0xD450,0xD550,0xD650,0xD750,
        0xD850,0xD950,0xDA50,0xDB50,0xDC50,0xDD50,0xDE50,0xDF50,
        0xE050,0xE150,0xE250,0xE350,0xE450,0xE550,0xE650,0xE750,
        0xE850,0xE950,0xEA50,0xEB50,0xEC50,0xED50,0xEE50,0xEF50,
        0xF050,0xF150,0xF250,0xF350,0xF450,0xF550,0xF650,0xF750,
        0xF850,0xF950,0xFA50,0xFB50,0xFC50,0xFD50,0xFE50,0xFF50,
        0x00D0,0x0150,0x0250,0x0350,0x0450,0x0550,0x0650,0x0750,
        0x0850,0x0950,0x0A50,0x0B50,0x0C50,0x0D50,0x0E50,0x0F50,
        0x1050,0x1150,0x1250,0x1350,0x1450,0x1550,0x1650,0x1750,
        0x1850,0x1950,0x1A50,0x1B50,0x1C50,0x1D50,0x1E50,0x1F50,
        0x2050,0x2150,0x2250,0x2350,0x2450,0x2550,0x2650,0x2750,
        0x2850,0x2950,0x2A50,0x2B50,0x2C50,0x2D50,0x2E50,0x2F50,
        0x3050,0x3150,0x3250,0x3350,0x3450,0x3550,0x3650,0x3750,
        0x3850,0x3950,0x3A50,0x3B50,0x3C50,0x3D50,0x3E50,0x3F50,
        0x4050,0x4150,0x4250,0x4350,0x4450,0x4550,0x4650,0x4750,
        0x4850,0x4950,0x4A50,0x4B50,0x4C50,0x4D50,0x4E50,0x4F50,
        0x5050,0x5150,0x5250,0x5350,0x5450,0x5550,0x5650,0x5750,
        0x5850,0x5950,0x5A50,0x5B50,0x5C50,0x5D50,0x5E50,0x5F50,
        0x6050,0x6150,0x6250,0x6350,0x6450,0x6550,0x6650,0x6750,
        0x6850,0x6950,0x6A50,0x6B50,0x6C50,0x6D50,0x6E50,0x6F50,
        0x7050,0x7150,0x7250,0x7350,0x7450,0x7550,0x7650,0x7750,
        0x7850,0x7950,0x7A50,0x7B50,0x7C50,0x7D50,0x7E50,0x7F50,
        0x8050,0x8150,0x8250,0x8350,0x8450,0x8550,0x8650,0x8750,
        0x8850,0x8950,0x8A50,0x8B50,0x8C50,0x8D50,0x8E50,0x8F50,
        0x9050,0x9150,0x9250,0x9350,0x9450,0x9550,0x9650,0x9750,
        0x9850,0x9950,0x9A50,0x9B50,0x9C50,0x9D50,0x9E50,0x9F50,
        0xFA40,0xFB40,0xFC40,0xFD40,0xFE40,0xFF40,0x00C0,0x0140,
        0x0240,0x0340,0x0440,0x0540,0x0640,0x0740,0x0840,0x0940,
        0x0A40,0x0B40,0x0C40,0x0D40,0x0E40,0x0F40,0x1040,0x1140,
        0x1240,0x1340,0x1440,0x1540,0x1640,0x1740,0x1840,0x1940,
        0x1A40,0x1B40,0x1C40,0x1D40,0x1E40,0x1F40,0x2040,0x2140,
        0x2240,0x2340,0x2440,0x2540,0x2640,0x2740,0x2840,0x2940,
        0x2A40,0x2B40,0x2C40,0x2D40,0x2E40,0x2F40,0x3040,0x3140,
        0x3240,0x3340,0x3440,0x3540,0x3640,0x3740,0x3840,0x3940,
        0x3A40,0x3B40,0x3C40,0x3D40,0x3E40,0x3F40,0x4040,0x4140,
        0x4240,0x4340,0x4440,0x4540,0x4640,0x4740,0x4840,0x4940,
        0x4A40,0x4B40,0x4C40,0x4D40,0x4E40,0x4F40,0x5040,0x5140,
        0x5240,0x5340,0x5440,0x5540,0x5640,0x5740,0x5840,0x5940,
        0x5A40,0x5B40,0x5C40,0x5D40,0x5E40,0x5F40,0x6040,0x6140,
        0x6240,0x6340,0x6440,0x6540,0x6640,0x6740,0x6840,0x6940,
        0x6A40,0x6B40,0x6C40,0x6D40,0x6E40,0x6F40,0x7040,0x7140,
        0x7240,0x7340,0x7440,0x7540,0x7640,0x7740,0x7840,0x7940,
        0x7A40,0x7B40,0x7C40,0x7D40,0x7E40,0x7F40,0x8040,0x8140,
        0x8240,0x8340,0x8440,0x8540,0x8640,0x8740,0x8840,0x8940,
        0x8A40,0x8B40,0x8C40,0x8D40,0x8E40,0x8F40,0x9040,0x9140,
        0x9240,0x9340,0x9440,0x9540,0x9640,0x9740,0x9840,0x9940,
        0x9A40,0x9B40,0x9C40,0x9D40,0x9E40,0x9F40,0xA040,0xA140,
        0xA240,0xA340,0xA440,0xA540,0xA640,0xA740,0xA840,0xA940,
        0xAA40,0xAB40,0xAC40,0xAD40,0xAE40,0xAF40,0xB040,0xB140,
        0xB240,0xB340,0xB440,0xB540,0xB640,0xB740,0xB840,0xB940,
        0xBA40,0xBB40,0xBC40,0xBD40,0xBE40,0xBF40,0xC040,0xC140,
        0xC240,0xC340,0xC440,0xC540,0xC640,0xC740,0xC840,0xC940,
        0xCA40,0xCB40,0xCC40,0xCD40,0xCE40,0xCF40,0xD040,0xD140,
        0xD240,0xD340,0xD440,0xD540,0xD640,0xD740,0xD840,0xD940,
        0xDA40,0xDB40,0xDC40,0xDD40,0xDE40,0xDF40,0xE040,0xE140,
        0xE240,0xE340,0xE440,0xE540,0xE640,0xE740,0xE840,0xE940,
        0xEA40,0xEB40,0xEC40,0xED40,0xEE40,0xEF40,0xF040,0xF140,
        0xF240,0xF340,0xF440,0xF540,0xF640,0xF740,0xF840,0xF940,
        0x9A50,0x9B50,0x9C50,0x9D50,0x9E50,0x9F50,0xA050,0xA150,
        0xA250,0xA350,0xA450,0xA550,0xA650,0xA750,0xA850,0xA950,
        0xAA50,0xAB50,0xAC50,0xAD50,0xAE50,0xAF50,0xB050,0xB150,
        0xB250,0xB350,0xB450,0xB550,0xB650,0xB750,0xB850,0xB950,
        0xBA50,0xBB50,0xBC50,0xBD50,0xBE50,0xBF50,0xC050,0xC150,
        0xC250,0xC350,0xC450,0xC550,0xC650,0xC750,0xC850,0xC950,
        0xCA50,0xCB50,0xCC50,0xCD50,0xCE50,0xCF50,0xD050,0xD150,
        0xD250,0xD350,0xD450,0xD550,0xD650,0xD750,0xD850,0xD950,
        0xDA50,0xDB50,0xDC50,0xDD50,0xDE50,0xDF50,0xE050,0xE150,
        0xE250,0xE350,0xE450,0xE550,0xE650,0xE750,0xE850,0xE950,
        0xEA50,0xEB50,0xEC50,0xED50,0xEE50,0xEF50,0xF050,0xF150,
        0xF250,0xF350,0xF450,0xF550,0xF650,0xF750,0xF850,0xF950,
        0xFA50,0xFB50,0xFC50,0xFD50,0xFE50,0xFF50,0x00D0,0x0150,
        0x0250,0x0350,0x0450,0x0550,0x0650,0x0750,0x0850,0x0950,
        0x0A50,0x0B50,0x0C50,0x0D50,0x0E50,0x0F50,0x1050,0x1150,
        0x1250,0x1350,0x1450,0x1550,0x1650,0x1750,0x1850,0x1950,
        0x1A50,0x1B50,0x1C50,0x1D50,0x1E50,0x1F50,0x2050,0x2150,
        0x2250,0x2350,0x2450,0x2550,0x2650,0x2750,0x2850,0x2950,
        0x2A50,0x2B50,0x2C50,0x2D50,0x2E50,0x2F50,0x3050,0x3150,
        0x3250,0x3350,0x3450,0x3550,0x3650,0x3750,0x3850,0x3950,
        0x3A50,0x3B50,0x3C50,0x3D50,0x3E50,0x3F50,0x4050,0x4150,
        0x4250,0x4350,0x4450,0x4550,0x4650,0x4750,0x4850,0x4950,
        0x4A50,0x4B50,0x4C50,0x4D50,0x4E50,0x4F50,0x5050,0x5150,
        0x5250,0x5350,0x5450,0x5550,0x5650,0x5750,0x5850,0x5950,
        0x5A50,0x5B50,0x5C50,0x5D50,0x5E50,0x5F50,0x6050,0x6150,
        0x6250,0x6350,0x6450,0x6550,0x6650,0x6750,0x6850,0x6950,
        0x6A50,0x6B50,0x6C50,0x6D50,0x6E50,0x6F50,0x7050,0x7150,
        0x7250,0x7350,0x7450,0x7550,0x7650,0x7750,0x7850,0x7950,
        0x7A50,0x7B50,0x7C50,0x7D50,0x7E50,0x7F50,0x8050,0x8150,
        0x8250,0x8350,0x8450,0x8550,0x8650,0x8750,0x8850,0x8950,
        0x8A50,0x8B50,0x8C50,0x8D50,0x8E50,0x8F50,0x9050,0x9150,
        0x9250,0x9350,0x9450,0x9550,0x9650,0x9750,0x9850,0x9950,
    ];

    let index = ((cpu.f & (ADD_SUB_FLAG | CARRY_FLAG | HALF_CARRY_FLAG)) as u16 << 4) |
                  cpu.a as u16;
    cpu.af().set(DAA_TABLE[index as uint]);
}

include!("cpu_tests.rs");
