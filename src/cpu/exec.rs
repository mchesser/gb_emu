//! Here we define functions to emulate the GB instructions. Because many of the instructions
//! only differ by the registers they are acting on macros are used to reduce code duplication.
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
        mem.sb(cpu.sp-1, *cpu.$qq().high);
        mem.sb(cpu.sp-2, *cpu.$qq().low);
        cpu.sp -= 2;
    }) }
    // POP qq
    macro_rules! pop_qq { ($qq: ident) => ({
        *cpu.$qq().high = mem.lb(cpu.sp+1);
        *cpu.$qq().low = mem.lb(cpu.sp);
        cpu.sp += 2;
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
        cpu.jump(mem, addr);
    }) }
    // JP c, nn
    macro_rules! jp_cnn { ($c: expr) => (
        if $c { jp_nn!(); 4 } else { cpu.pc += 2; 3 }
    ) }
    // JP hl
    macro_rules! jp_hl { () => ({
        let addr = cpu.hl().get();
        cpu.jump(mem, addr);
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
        cpu.jump(mem, addr);
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
    // println!("0x{:04X}:\t{}", cpu.pc, disasm::disasm(cpu.pc, mem));
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
        0x46 => { ld_rDD!(b, hl)        2 },
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
        0x56 => { ld_rDD!(d, hl)        2 },
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
        0x66 => { ld_rDD!(h, hl)        2 },
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
        0xF1 => { pop_qq!(af);          3 },
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
    // |  0000  0NCH  AAAA AAAA | -> | FFFF FFFF AAAA AAAA |
    // Where N = ADD_SUB_FLAG, C = CARRY_FLAG, H = HALF_CARRY_FLAG, AAAA AAAA = register `a`,
    // FFFF FFFF = register `f`
    const DAA_TABLE: [u16, ..2048] = [
        0x8000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007,
        0x0008,0x0009,0x2010,0x2011,0x2012,0x2013,0x2014,0x2015,
        0x0010,0x0011,0x0012,0x0013,0x0014,0x0015,0x0016,0x0017,
        0x0018,0x0019,0x2020,0x2021,0x2022,0x2023,0x2024,0x2025,
        0x0020,0x0021,0x0022,0x0023,0x0024,0x0025,0x0026,0x0027,
        0x0028,0x0029,0x2030,0x2031,0x2032,0x2033,0x2034,0x2035,
        0x0030,0x0031,0x0032,0x0033,0x0034,0x0035,0x0036,0x0037,
        0x0038,0x0039,0x2040,0x2041,0x2042,0x2043,0x2044,0x2045,
        0x0040,0x0041,0x0042,0x0043,0x0044,0x0045,0x0046,0x0047,
        0x0048,0x0049,0x2050,0x2051,0x2052,0x2053,0x2054,0x2055,
        0x0050,0x0051,0x0052,0x0053,0x0054,0x0055,0x0056,0x0057,
        0x0058,0x0059,0x2060,0x2061,0x2062,0x2063,0x2064,0x2065,
        0x0060,0x0061,0x0062,0x0063,0x0064,0x0065,0x0066,0x0067,
        0x0068,0x0069,0x2070,0x2071,0x2072,0x2073,0x2074,0x2075,
        0x0070,0x0071,0x0072,0x0073,0x0074,0x0075,0x0076,0x0077,
        0x0078,0x0079,0x2080,0x2081,0x2082,0x2083,0x2084,0x2085,
        0x0080,0x0081,0x0082,0x0083,0x0084,0x0085,0x0086,0x0087,
        0x0088,0x0089,0x2090,0x2091,0x2092,0x2093,0x2094,0x2095,
        0x0090,0x0091,0x0092,0x0093,0x0094,0x0095,0x0096,0x0097,
        0x0098,0x0099,0xB000,0x3001,0x3002,0x3003,0x3004,0x3005,
        0x9000,0x1001,0x1002,0x1003,0x1004,0x1005,0x1006,0x1007,
        0x1008,0x1009,0x3010,0x3011,0x3012,0x3013,0x3014,0x3015,
        0x1010,0x1011,0x1012,0x1013,0x1014,0x1015,0x1016,0x1017,
        0x1018,0x1019,0x3020,0x3021,0x3022,0x3023,0x3024,0x3025,
        0x1020,0x1021,0x1022,0x1023,0x1024,0x1025,0x1026,0x1027,
        0x1028,0x1029,0x3030,0x3031,0x3032,0x3033,0x3034,0x3034,
        0x1030,0x1031,0x1032,0x1032,0x1034,0x1035,0x1035,0x1037,
        0x1038,0x1039,0x3040,0x3041,0x3042,0x3043,0x3044,0x3045,
        0x1040,0x1041,0x1042,0x1042,0x1044,0x1045,0x1046,0x1047,
        0x1048,0x1049,0x3050,0x3050,0x3052,0x3053,0x3054,0x3055,
        0x1050,0x1051,0x1052,0x1053,0x1054,0x1055,0x1056,0x1057,
        0x1058,0x1059,0x3060,0x3060,0x3062,0x3063,0x3064,0x3065,
        0x1060,0x1061,0x1062,0x1063,0x1064,0x1065,0x1066,0x1067,
        0x1068,0x1069,0x3070,0x3071,0x3072,0x3073,0x3074,0x3075,
        0x1070,0x1071,0x1072,0x1073,0x1074,0x1075,0x1076,0x1077,
        0x1078,0x1079,0x3080,0x3081,0x3082,0x3083,0x3084,0x3085,
        0x1080,0x1081,0x1082,0x1083,0x1084,0x1085,0x1086,0x1087,
        0x1088,0x1089,0x3090,0x3090,0x3092,0x3093,0x3094,0x3095,
        0x1090,0x1091,0x1092,0x1093,0x1094,0x1095,0x1096,0x1097,
        0x1098,0x1099,0x30A0,0x30A1,0x30A2,0x30A3,0x30A4,0x30A5,
        0x10A0,0x10A1,0x10A2,0x10A3,0x10A4,0x10A5,0x10A6,0x10A7,
        0x10A8,0x10A9,0x30B0,0x30B1,0x30B2,0x30B3,0x30B4,0x30B5,
        0x10B0,0x10B1,0x10B2,0x10B3,0x10B4,0x10B5,0x10B6,0x10B7,
        0x10B8,0x10B9,0x30C0,0x30C1,0x30C2,0x30C3,0x30C4,0x30C5,
        0x10C0,0x10C1,0x10C2,0x10C3,0x10C4,0x10C5,0x10C5,0x10C7,
        0x10C8,0x10C9,0x10D0,0x10D1,0x10D2,0x10D3,0x10D4,0x10D5,
        0x10D0,0x10D1,0x10D2,0x10D2,0x10D4,0x10D5,0x10D6,0x10D7,
        0x10D8,0x10D9,0x30E0,0x30E1,0x30E2,0x30E3,0x30E4,0x30E5,
        0x10E0,0x10E1,0x10E2,0x10E3,0x10E4,0x10E5,0x10E6,0x10E7,
        0x10E8,0x10E9,0x30F0,0x30F1,0x30F2,0x30F3,0x30F4,0x30F5,
        0x10F0,0x10F1,0x10F2,0x10F3,0x10F4,0x10F5,0x10F6,0x10F7,
        0x10F8,0x10F9,0xB000,0x3001,0x3002,0x3003,0x3004,0x3005,
        0x9000,0x1001,0x1002,0x1002,0x1004,0x1005,0x1005,0x1007,
        0x1008,0x1009,0x3010,0x3011,0x3012,0x3013,0x3014,0x3015,
        0x1010,0x1011,0x1012,0x1013,0x1014,0x1015,0x1016,0x1017,
        0x1018,0x1019,0x3020,0x3020,0x3022,0x3023,0x3024,0x3025,
        0x1020,0x1021,0x1022,0x1023,0x1024,0x1025,0x1026,0x1027,
        0x1028,0x1029,0x3030,0x3031,0x3032,0x3033,0x3033,0x3035,
        0x1030,0x1031,0x1032,0x1033,0x1034,0x1034,0x1036,0x1037,
        0x1038,0x1039,0x3040,0x3040,0x3042,0x3043,0x3044,0x3045,
        0x1040,0x1041,0x1042,0x1043,0x1044,0x1045,0x1046,0x1047,
        0x1048,0x1049,0x3050,0x3051,0x3052,0x3053,0x3054,0x3055,
        0x1050,0x1051,0x1052,0x1052,0x1054,0x1055,0x1056,0x1057,
        0x1058,0x1059,0x3060,0x3061,0x3062,0x3063,0x3064,0x3065,
        0x0006,0x0007,0x0008,0x0008,0x000A,0x000B,0x000B,0x000D,
        0x000E,0x000F,0x2010,0x2011,0x2012,0x2013,0x2014,0x2015,
        0x0016,0x0017,0x0018,0x0019,0x001A,0x001B,0x001C,0x001D,
        0x001E,0x001F,0x2020,0x2021,0x2022,0x2023,0x2023,0x2025,
        0x0026,0x0027,0x0028,0x0029,0x002A,0x002B,0x002C,0x002D,
        0x002E,0x002F,0x2030,0x2031,0x2031,0x2033,0x2034,0x2035,
        0x0036,0x0037,0x0038,0x0038,0x003A,0x003B,0x003C,0x003D,
        0x003E,0x003F,0x2040,0x2041,0x2042,0x2043,0x2044,0x2045,
        0x0046,0x0047,0x0048,0x0049,0x004A,0x004B,0x004B,0x004D,
        0x004E,0x004F,0x2050,0x2051,0x2052,0x2053,0x2054,0x2055,
        0x0056,0x0057,0x0058,0x0059,0x005A,0x005B,0x005C,0x005D,
        0x005E,0x005F,0x2060,0x2061,0x2062,0x2063,0x2064,0x2064,
        0x0066,0x0067,0x0068,0x0068,0x006A,0x006B,0x006C,0x006D,
        0x006E,0x006F,0x2070,0x2071,0x2072,0x2073,0x2074,0x2075,
        0x0076,0x0077,0x0078,0x0079,0x007A,0x007B,0x007B,0x007D,
        0x007E,0x007F,0x2080,0x2081,0x2082,0x2083,0x2084,0x2085,
        0x0086,0x0087,0x0088,0x0089,0x008A,0x008A,0x008C,0x008D,
        0x008E,0x008F,0x2090,0x2091,0x2091,0x2093,0x2094,0x2095,
        0x0096,0x0097,0x0098,0x0099,0x009A,0x009B,0x009C,0x009D,
        0x009E,0x009F,0xB000,0x3001,0x3002,0x3003,0x3004,0x3005,
        0x1006,0x1007,0x1008,0x1009,0x100A,0x100B,0x100C,0x100D,
        0x100E,0x100F,0x3010,0x3011,0x3012,0x3013,0x3014,0x3015,
        0x1016,0x1017,0x1018,0x1019,0x101A,0x101B,0x101B,0x101D,
        0x101E,0x101F,0x3020,0x3021,0x3022,0x3023,0x3023,0x3025,
        0x1026,0x1027,0x1028,0x1029,0x102A,0x102B,0x102C,0x102D,
        0x102E,0x102F,0x3030,0x3031,0x3032,0x3033,0x3034,0x3035,
        0x1036,0x1037,0x1038,0x1039,0x103A,0x103B,0x103C,0x103D,
        0x103E,0x103F,0x3040,0x3041,0x3042,0x3043,0x3043,0x3045,
        0x1046,0x1047,0x1048,0x1049,0x104A,0x104B,0x104C,0x104D,
        0x104E,0x104F,0x3050,0x3051,0x3052,0x3053,0x3054,0x3055,
        0x1056,0x1057,0x1058,0x1059,0x105A,0x105B,0x105C,0x105C,
        0x105E,0x105F,0x3060,0x3060,0x3062,0x3063,0x3064,0x3065,
        0x1066,0x1067,0x1068,0x1069,0x106A,0x106B,0x106C,0x106D,
        0x106E,0x106F,0x3070,0x3070,0x3072,0x3073,0x3074,0x3075,
        0x1076,0x1077,0x1078,0x1079,0x107A,0x107B,0x107C,0x107D,
        0x107E,0x107F,0x3080,0x3081,0x3082,0x3083,0x3084,0x3085,
        0x1086,0x1087,0x1088,0x1089,0x108A,0x108B,0x108C,0x108D,
        0x108E,0x108F,0x3090,0x3091,0x3092,0x3093,0x3094,0x3095,
        0x1096,0x1097,0x1098,0x1099,0x109A,0x109B,0x109C,0x109D,
        0x109E,0x109F,0x30A0,0x30A1,0x30A2,0x30A3,0x30A4,0x30A5,
        0x10A6,0x10A7,0x10A8,0x10A9,0x10AA,0x10AB,0x10AC,0x10AD,
        0x10AE,0x10AF,0x30B0,0x30B1,0x30B2,0x30B3,0x30B4,0x30B5,
        0x10B6,0x10B7,0x10B8,0x10B9,0x10BA,0x10BB,0x10BC,0x10BD,
        0x10BE,0x10BF,0x30C0,0x30C1,0x30C2,0x30C3,0x30C3,0x30C5,
        0x10C6,0x10C7,0x10C8,0x10C9,0x10CA,0x10CB,0x10CC,0x10CD,
        0x10CE,0x10CF,0x30D0,0x30D0,0x30D2,0x30D3,0x30D4,0x30D5,
        0x10D6,0x10D7,0x10D8,0x10D9,0x10DA,0x10DB,0x10DC,0x10DD,
        0x10DE,0x10DF,0x30E0,0x30E1,0x30E2,0x30E3,0x30E4,0x30E5,
        0x10E6,0x10E7,0x10E8,0x10E9,0x10EA,0x10EB,0x10EC,0x10ED,
        0x10EE,0x10EF,0x30F0,0x30F0,0x30F2,0x30F3,0x30F4,0x30F5,
        0x10F6,0x10F7,0x10F8,0x10F9,0x10FA,0x10FB,0x10FC,0x10FD,
        0x10FE,0x10FF,0xB000,0x3001,0x3002,0x3003,0x3004,0x3005,
        0x1006,0x1007,0x1008,0x1009,0x100A,0x100B,0x100C,0x100D,
        0x100E,0x100F,0x3010,0x3011,0x3012,0x3013,0x3014,0x3015,
        0x1016,0x1017,0x1018,0x1018,0x101A,0x101B,0x101C,0x101D,
        0x101E,0x101F,0x3020,0x3021,0x3022,0x3023,0x3024,0x3024,
        0x1026,0x1027,0x1028,0x1029,0x102A,0x102B,0x102B,0x102D,
        0x102E,0x102F,0x3030,0x3031,0x3032,0x3033,0x3034,0x3035,
        0x1036,0x1037,0x1038,0x1039,0x103A,0x103B,0x103C,0x103D,
        0x103E,0x103F,0x3040,0x3041,0x3042,0x3043,0x3044,0x3045,
        0x1046,0x1047,0x1048,0x1049,0x104A,0x104B,0x104C,0x104D,
        0x104E,0x104F,0x3050,0x3051,0x3052,0x3053,0x3054,0x3055,
        0x1056,0x1057,0x1058,0x1059,0x105A,0x105B,0x105C,0x105D,
        0x105E,0x105F,0x3060,0x3061,0x3062,0x3063,0x3064,0x3065,
        0xC000,0x4001,0x4002,0x4003,0x4004,0x4004,0x4006,0x4007,
        0x4008,0x4009,0x4004,0x4005,0x4006,0x4007,0x4008,0x4009,
        0x4010,0x4011,0x4012,0x4013,0x4014,0x4015,0x4016,0x4017,
        0x4018,0x4019,0x4014,0x4015,0x4016,0x4017,0x4018,0x4019,
        0x4020,0x4021,0x4022,0x4023,0x4024,0x4025,0x4026,0x4027,
        0x4028,0x4029,0x4024,0x4025,0x4026,0x4027,0x4028,0x4029,
        0x4030,0x4031,0x4032,0x4033,0x4034,0x4035,0x4035,0x4037,
        0x4038,0x4039,0x4034,0x4035,0x4036,0x4037,0x4038,0x4039,
        0x4040,0x4041,0x4042,0x4043,0x4044,0x4045,0x4046,0x4047,
        0x4048,0x4049,0x4044,0x4045,0x4046,0x4046,0x4048,0x4049,
        0x4050,0x4051,0x4052,0x4053,0x4054,0x4055,0x4056,0x4057,
        0x4058,0x4059,0x4054,0x4055,0x4056,0x4057,0x4058,0x4059,
        0x4060,0x4061,0x4062,0x4063,0x4064,0x4065,0x4066,0x4067,
        0x4068,0x4069,0x4064,0x4064,0x4066,0x4067,0x4068,0x4069,
        0x4070,0x4071,0x4072,0x4073,0x4074,0x4075,0x4076,0x4077,
        0x4078,0x4079,0x4074,0x4075,0x4076,0x4077,0x4077,0x4079,
        0x4080,0x4081,0x4082,0x4083,0x4084,0x4085,0x4086,0x4087,
        0x4088,0x4089,0x4084,0x4085,0x4086,0x4087,0x4088,0x4089,
        0x4090,0x4091,0x4092,0x4093,0x4094,0x4095,0x4096,0x4097,
        0x4098,0x4099,0x5034,0x5035,0x5036,0x5037,0x5037,0x5039,
        0x5040,0x5041,0x5042,0x5043,0x5044,0x5045,0x5046,0x5047,
        0x5048,0x5049,0x5044,0x5045,0x5046,0x5047,0x5048,0x5049,
        0x5050,0x5051,0x5052,0x5053,0x5054,0x5055,0x5056,0x5057,
        0x5058,0x5059,0x5054,0x5054,0x5056,0x5057,0x5058,0x5059,
        0x5060,0x5061,0x5062,0x5063,0x5064,0x5065,0x5066,0x5067,
        0x5068,0x5069,0x5069,0x5065,0x5066,0x5067,0x5068,0x5069,
        0x5070,0x5071,0x5072,0x5073,0x5074,0x5075,0x5076,0x5077,
        0x5078,0x5079,0x5074,0x5075,0x5076,0x5077,0x5078,0x5079,
        0x5080,0x5081,0x5082,0x5083,0x5084,0x5085,0x5086,0x5087,
        0x5088,0x5089,0x5084,0x5084,0x5086,0x5087,0x5088,0x5089,
        0x5090,0x5091,0x5092,0x5093,0x5094,0x5095,0x5095,0x5097,
        0x5098,0x5099,0x5094,0x5095,0x5096,0x5097,0x5098,0x5099,
        0x50A0,0x50A1,0x50A2,0x50A3,0x50A4,0x50A5,0x50A5,0x50A7,
        0x50A8,0x50A9,0x50A4,0x50A5,0x50A6,0x50A7,0x50A8,0x50A9,
        0x50B0,0x50B1,0x50B2,0x50B3,0x50B4,0x50B5,0x50B6,0x50B7,
        0x50B8,0x50B9,0x50B4,0x50B4,0x50B6,0x50B7,0x50B8,0x50B9,
        0x50C0,0x50C1,0x50C2,0x50C3,0x50C4,0x50C5,0x50C6,0x50C7,
        0x50C8,0x50C9,0x50C4,0x50C5,0x50C6,0x50C7,0x50C8,0x50C9,
        0x50D0,0x50D1,0x50D2,0x50D3,0x50D4,0x50D5,0x50D5,0x50D7,
        0x50D8,0x50D9,0x50D4,0x50D5,0x50D6,0x50D7,0x50D8,0x50D9,
        0x50E0,0x50E1,0x50E2,0x50E3,0x50E4,0x50E5,0x50E6,0x50E7,
        0x50E8,0x50E9,0x50E4,0x50E5,0x50E6,0x50E7,0x50E8,0x50E9,
        0x50F0,0x50F1,0x50F2,0x50F3,0x50F4,0x50F5,0x50F6,0x50F7,
        0x50F8,0x50F9,0x50F4,0x50F5,0x50F6,0x50F7,0x50F8,0x50F9,
        0xD000,0x5001,0x5002,0x5003,0x5004,0x5005,0x5006,0x5007,
        0x5008,0x5009,0x5004,0x5005,0x5006,0x5007,0x5008,0x5009,
        0x5010,0x5011,0x5012,0x5013,0x5014,0x5015,0x5016,0x5017,
        0x5018,0x5019,0x5014,0x5015,0x5016,0x5017,0x5017,0x5019,
        0x5020,0x5021,0x5022,0x5022,0x5024,0x5025,0x5026,0x5027,
        0x5028,0x5029,0x5024,0x5025,0x5026,0x5027,0x5028,0x5029,
        0x5030,0x5031,0x5032,0x5033,0x5034,0x5035,0x5036,0x5037,
        0x5038,0x5039,0x5034,0x5034,0x5036,0x5037,0x5037,0x5039,
        0x5040,0x5041,0x5042,0x5043,0x5044,0x5045,0x5045,0x5047,
        0x5048,0x5049,0x5044,0x5045,0x5046,0x5047,0x5047,0x5049,
        0x5050,0x5051,0x5052,0x5053,0x5054,0x5055,0x5056,0x5057,
        0x5058,0x5059,0x5059,0x5055,0x5056,0x5057,0x5058,0x5059,
        0x5060,0x5061,0x5062,0x5063,0x5064,0x5065,0x5066,0x5067,
        0x5068,0x5069,0x5064,0x5065,0x5066,0x5067,0x5068,0x5069,
        0x5070,0x5071,0x5072,0x5073,0x5074,0x5075,0x5076,0x5077,
        0x5078,0x5079,0x5074,0x5075,0x5076,0x5077,0x5078,0x5079,
        0x5080,0x5081,0x5082,0x5083,0x5084,0x5085,0x5086,0x5087,
        0x5088,0x5089,0x5084,0x5085,0x5086,0x5086,0x5088,0x5089,
        0x5090,0x5091,0x5092,0x5093,0x5094,0x5095,0x5096,0x5097,
        0x5098,0x5099,0x5094,0x5094,0x5096,0x5097,0x5098,0x5099,
        0x60FA,0x60FB,0x60FC,0x60FD,0x60FE,0x60FF,0xC000,0x4001,
        0x4002,0x4003,0x4004,0x4005,0x4006,0x4007,0x4008,0x4009,
        0x600A,0x600B,0x600C,0x600D,0x600E,0x600F,0x400F,0x4011,
        0x4012,0x4013,0x4014,0x4015,0x4016,0x4017,0x4018,0x4019,
        0x601A,0x601B,0x601C,0x601D,0x601E,0x601F,0x4020,0x4021,
        0x4022,0x4023,0x4024,0x4025,0x4026,0x4027,0x4027,0x4029,
        0x602A,0x602B,0x602C,0x602D,0x602E,0x602F,0x4030,0x4031,
        0x4032,0x4033,0x4034,0x4035,0x4036,0x4037,0x4038,0x4039,
        0x603A,0x603B,0x603C,0x603D,0x603E,0x603F,0x403F,0x4041,
        0x4042,0x4043,0x4044,0x4045,0x4046,0x4047,0x4048,0x4049,
        0x604A,0x604B,0x604C,0x604D,0x604E,0x604E,0x4050,0x4051,
        0x4052,0x4053,0x4054,0x4054,0x4056,0x4057,0x4058,0x4059,
        0x605A,0x605B,0x605C,0x605D,0x605E,0x605F,0x4060,0x4061,
        0x4062,0x4063,0x4064,0x4065,0x4066,0x4067,0x4068,0x4069,
        0x606A,0x606B,0x606C,0x606D,0x606E,0x606F,0x4070,0x4071,
        0x4072,0x4073,0x4074,0x4075,0x4076,0x4077,0x4078,0x4079,
        0x607A,0x607B,0x607C,0x607D,0x607E,0x607F,0x4080,0x4081,
        0x4082,0x4083,0x4084,0x4085,0x4086,0x4087,0x4088,0x4088,
        0x608A,0x608B,0x608C,0x608C,0x608E,0x608F,0x4090,0x4091,
        0x4092,0x4093,0x5034,0x5035,0x5035,0x5037,0x5038,0x5039,
        0x703A,0x703B,0x703C,0x703C,0x703E,0x703F,0x5040,0x5041,
        0x5042,0x5043,0x5044,0x5045,0x5046,0x5047,0x5048,0x5049,
        0x704A,0x704B,0x704C,0x704C,0x704E,0x704F,0x5050,0x5051,
        0x5052,0x5053,0x5054,0x5055,0x5056,0x5057,0x5058,0x5059,
        0x705A,0x705B,0x705C,0x705D,0x705E,0x705F,0x5060,0x5061,
        0x5062,0x5063,0x5064,0x5064,0x5066,0x5067,0x5067,0x5069,
        0x706A,0x706B,0x706C,0x706C,0x706E,0x706F,0x5070,0x5071,
        0x5072,0x5073,0x5074,0x5075,0x5076,0x5077,0x5078,0x5079,
        0x707A,0x707B,0x707C,0x707D,0x707E,0x707F,0x507F,0x5081,
        0x5082,0x5083,0x5084,0x5085,0x5086,0x5087,0x5088,0x5089,
        0x708A,0x708B,0x708C,0x708D,0x708E,0x708F,0x508F,0x5091,
        0x5092,0x5093,0x5094,0x5095,0x5096,0x5097,0x5097,0x5099,
        0x709A,0x709B,0x709C,0x709D,0x709E,0x709F,0x509F,0x50A1,
        0x50A2,0x50A3,0x50A4,0x50A5,0x50A6,0x50A7,0x50A8,0x50A9,
        0x70AA,0x70AB,0x70AC,0x70AD,0x70AE,0x70AF,0x50B0,0x50B1,
        0x50B2,0x50B3,0x50B4,0x50B4,0x50B6,0x50B7,0x50B8,0x50B9,
        0x70BA,0x70BB,0x70BC,0x70BD,0x70BE,0x70BF,0x50C0,0x50C1,
        0x50C2,0x50C3,0x50C4,0x50C5,0x50C6,0x50C7,0x50C8,0x50C9,
        0x70CA,0x70CB,0x70CC,0x70CD,0x70CE,0x70CF,0x50CF,0x50D1,
        0x50D2,0x50D3,0x50D4,0x50D5,0x50D6,0x50D7,0x50D8,0x50D9,
        0x70DA,0x70DB,0x70DC,0x70DD,0x70DE,0x70DF,0x50E0,0x50E1,
        0x50E2,0x50E3,0x50E4,0x50E5,0x50E5,0x50E7,0x50E8,0x50E8,
        0x70EA,0x70EB,0x70EC,0x70ED,0x70ED,0x70EF,0x50F0,0x50F1,
        0x50F2,0x50F3,0x50F4,0x50F5,0x50F6,0x50F7,0x50F8,0x50F9,
        0x70FA,0x70FB,0x70FC,0x70FD,0x70FE,0x70FF,0xD000,0x5001,
        0x5002,0x5003,0x5004,0x5005,0x5006,0x5007,0x5007,0x5009,
        0x700A,0x700B,0x700C,0x700D,0x700E,0x700F,0x500F,0x5011,
        0x5012,0x5013,0x5014,0x5015,0x5016,0x5017,0x5017,0x5019,
        0x701A,0x701B,0x701C,0x701D,0x701E,0x701F,0x5020,0x5021,
        0x5022,0x5023,0x5024,0x5025,0x5026,0x5027,0x5028,0x5029,
        0x702A,0x702B,0x702C,0x702D,0x702E,0x702F,0x502F,0x5031,
        0x5032,0x5033,0x5034,0x5035,0x5036,0x5037,0x5037,0x5039,
        0x703A,0x703B,0x703C,0x703D,0x703E,0x703F,0x5040,0x5041,
        0x5042,0x5043,0x5044,0x5045,0x5046,0x5047,0x5048,0x5049,
        0x704A,0x704B,0x704C,0x704D,0x704E,0x704F,0x5050,0x5051,
        0x5052,0x5053,0x5054,0x5054,0x5056,0x5057,0x5058,0x5059,
        0x705A,0x705B,0x705C,0x705D,0x705E,0x705F,0x5060,0x5061,
        0x5062,0x5063,0x5064,0x5065,0x5066,0x5067,0x5068,0x5069,
        0x706A,0x706B,0x706C,0x706D,0x706E,0x706F,0x5070,0x5071,
        0x5072,0x5073,0x5074,0x5074,0x5076,0x5077,0x5078,0x5079,
        0x707A,0x707B,0x707C,0x707C,0x707E,0x707F,0x5080,0x5081,
        0x5082,0x5083,0x5084,0x5085,0x5086,0x5087,0x5088,0x5089,
        0x708A,0x708B,0x708C,0x708D,0x708E,0x708F,0x5090,0x5091,
        0x5092,0x5093,0x5094,0x5095,0x5096,0x5097,0x5098,0x5099,
    ];

    let index = ((cpu.f & (ADD_SUB_FLAG | CARRY_FLAG | HALF_CARRY_FLAG)) as u16 << 4) |
                  cpu.a as u16;
    let lookup = DAA_TABLE[index as uint];
    cpu.a = lookup as u8;
    cpu.f = (lookup >> 8) as u8;
}

include!("cpu_tests.rs")
