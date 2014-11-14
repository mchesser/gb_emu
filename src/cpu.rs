#![feature(macro_rules)]

struct Memory {
    _ram: u32,
}
impl Memory {
    pub fn new() -> Memory {
        Memory {
            _ram: 0,
        }
    }
    
    fn lb(&mut self, _addr: u16) -> u8 {
        0
    }

    fn lw(&mut self, _addr: u16) -> u16 {
        0
    }

    fn sb(&mut self, _addr: u16, _value: u8) {
        
    }

    fn sw(&mut self, _addr: u16, _value: u16) {
        
    }
}

fn main() {
    let mut cpu = Cpu::new();
    let mut mem = Memory::new();
    cpu.fetch_exec(&mut mem);
}

// Flags for the GB processor:

pub const ZERO_FLAG: u8 = 0b1000_0000;
pub const ADD_SUB_FLAG: u8 = 0b0100_0000;
pub const HALF_CARRY_FLAG: u8 = 0b0010_0000;
pub const CARRY_FLAG: u8 = 0b0001_0000;
// Note: the lower 4 bits are unused.

/// The clock speed of the CPU in nanoseconds
pub const CLOCK_SPEED: f32 = 238.418; // == 4,194,304Hz

/// The main processor for GB/C. The processor is similar to the Z80 processor, with a few
/// differences (see: http://www.devrs.com/gb/files/gbspec.txt)
///
/// # Registers
///
/// * a,b,c,d,e,f,h,l : main register set (8-bit)
/// * sp : Stack pointer (16-bit)
/// * pc : Program counter (16-bit)
/// * ime : Interrupt flags register (8-bit)
///
/// In addition, the main register set can be combined into 16-bit registers: af, bc, de, hl
///
/// # Other
/// 
/// * stop: true if the processor is stopped and is waiting for a button press, otherwise false
/// * halt: true if the processor has been suspended, otherwise false
struct Cpu {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub f: u8,
    pub h: u8,
    pub l: u8,
    
    pub ime: u8,

    pub sp: u16,
    pub pc: u16,

    pub stop: bool,
    pub halt: bool,
}

// Several of the registers can be joint to form 16-bit registers. This macro allows us to easily
// join two registers together.
macro_rules! join_regs { (($r1: ident, $r2: ident) as $name: ident) => (
    impl Cpu {
        #[inline(always)]
        pub fn $name(&mut self) -> JointReg {
            JointReg {
                high: &mut self.$r1,
                low: &mut self.$r2,
            }
        }
    }
) }

join_regs!((b, c) as bc)
join_regs!((d, e) as de)
join_regs!((h, l) as hl)
join_regs!((a, f) as af)

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            f: 0,
            h: 0,
            l: 0,

            ime: 0,

            sp: 0,
            pc: 0,

            stop: false,
            halt: false,
        }
    }
    
    /// Fetch and execute the next instruction, returning the number of cycles used to complete the
    /// operation. 
    pub fn fetch_exec(&mut self, mem: &mut Memory) -> u8 {
        // Read the next byte after pc, useful for instructions requiring a `n` parameter
        macro_rules! get_n { () => (mem.lb(self.bump())) }
        // Read the next two bytes after pc, useful for instructions requiring a `nn` parameter
        macro_rules! get_nn { () => (
            (get_n!() as u16 << 8) + get_n!() as u16
        ) }
        // Check if the zero flag is set
        macro_rules! zflag { () => ( self.f & ZERO_FLAG != 0 ) }
        // Check if the carry flag is set
        macro_rules! cflag { () => ( self.f & CARRY_FLAG != 0 ) }
        
        
        // Here we define functions to emulate the GB instructions. Because many of the instructions
        // only differ by the registers they are acting on, so macros are used to reduce code
        // duplication. (Inspired by alexchricton's GB emulator)

        //
        // Instructions inherited from the Z80 processor
        //
        
        // LD r1, r2
        macro_rules! ld { ($r1: ident, $r2: ident) => ({
            self.$r1 = self.$r2;
        }) }
        // LD r, n
        macro_rules! ld_rn { ($r: ident) => ({
            self.$r = get_n!();
        }) }
        // LD r, (DD)
        macro_rules! ld_rDD { ($r: ident, $DD: ident) => ({
            self.$r = mem.lb(self.$DD().get());
        }) }
        // LD (DD), r
        macro_rules! ld_DDr { ($DD: ident, $r: ident) => ({
            mem.sb(self.$DD().get(), self.$r);
        }) }
        // LD (DD), n
        macro_rules! ld_DDn { ($DD: ident) => ({
            let val =  mem.lb(self.bump());
            mem.sb(self.$DD().get(), val);
        }) }
        // LD r, (NN)
        macro_rules! ld_rNN { ($r: ident) => ({
            let addr = get_nn!();
            self.$r = mem.lb(addr);
        }) }
        // LD (NN), r
        macro_rules! ld_NNr { ($r: ident) => ({
            let addr = get_nn!();
            mem.sb(addr, self.$r);
        }) }
        // LD dd, nn
        macro_rules! ld_ddnn { ($dd: ident) => ({
            let val = get_nn!();
            self.$dd().set(val);
        }) }
        // LD dd, (nn)
        macro_rules! ld_ddNN { ($dd: ident) => ({
            let addr = get_nn!();
            self.$dd().high = mem.lb(addr + 1);
            self.$dd().low = mem.lb(addr);
        }) }
        // LD (nn), dd
        macro_rules! ld_NNdd { ($dd: ident) => ({
            let addr = get_nn!();
            mem.sb(addr + 1, self.$dd().high);
            mem.sb(addr, self.$dd().low);
        }) }
        // LD SP, HL
        macro_rules! ld_sphl { () => (
            self.sp = self.hl().get();
        ) }
        // LD (nn), SP
        macro_rules! ld_NNsp { () => ({
            let addr = get_nn!();
            mem.sw(addr, self.sp);
        }) }
        // LD A,(FF00+n)
        macro_rules! ld_aIOn { () => ({
            let addr = 0xFF00 | get_n!() as u16;
            self.a = mem.lb(addr);
        }) }
        // LD A,(FF00+C)
        macro_rules! ld_aIOc { () => ({
            let addr = 0xFF00 | self.c as u16;
            self.a = mem.lb(addr);
        }) }
        // LD (FF00+n),A
        macro_rules! ld_IOan { () => ({
            let addr = 0xFF00 | get_n!() as u16;
            mem.sb(addr, self.a);
        }) }
        // LD (FF00+C),A
        macro_rules! ld_IOca { () => ({
            let addr = 0xFF00 | self.c as u16;
            mem.sb(addr, self.a);
        }) }
        // PUSH qq
        macro_rules! push_qq { ($qq: ident) => ({
            mem.sb(self.sp-1, *self.$qq().high);
            mem.sb(self.sp-2, *self.$qq().low);
            self.sp -= 2;
        }) }
        // POP qq
        macro_rules! pop_qq { ($qq: ident) => ({
            *self.$qq().high = mem.lb(self.sp+1);
            *self.$qq().low = mem.lb(self.sp);
            self.sp += 2;
        }) }

        // ADD A, s
        macro_rules! add_a { ($s: expr) => ({
            self.f = 0;
            if (self.a & 0xf) + ($s & 0xf) > 0xf { self.f |= HALF_CARRY_FLAG; }
            if self.a as u16 + $s as u16 > 0xff { self.f |= CARRY_FLAG; }
            self.a += $s;
            if self.a == 0 { self.f |= ZERO_FLAG; }
        }) }
        macro_rules! add_an { () => (add_a!(get_n!())) }
        macro_rules! add_ar { ($r: ident) => (add_a!(self.$r)) }
        macro_rules! add_aHL { () => (add_a!(mem.lb(self.hl().get()))) }

        // ADD HL, ..
        macro_rules! add_hl { ($s: expr) => ({
            let a = self.hl().get() as u32;
            let b = $s as u32;

            self.f &= ZERO_FLAG;
            if (a & 0xfff) + (b & 0xfff) > 0xfff { self.f |= HALF_CARRY_FLAG; }
            if a + b > 0xffff { self.f |= CARRY_FLAG; }

            self.hl().set((a + b) as u16);
        }) }
        macro_rules! add_hlss { ($ss: ident) => (add_hl!(self.$ss().get())) }
        macro_rules! add_hlsp { () => (add_hl!(self.sp)) }

        // ADD SP, n
        macro_rules! add_spn { () => ({
            let offset = get_n!() as i16;
            self.sp = (self.sp as i16 + offset) as u16;
        }) }
        
        // ADC A, s
        macro_rules! adc_a { ($s: expr) => (
            add_a!($s + (self.f >> 7) & 0x1)
        ) }
        macro_rules! adc_an { () => (adc_a!(get_n!())) }
        macro_rules! adc_ar { ($r: ident) => (adc_a!(self.$r)) }
        macro_rules! adc_aHL { () => (adc_a!(mem.lb(self.hl().get()))) }
        
        // ADC HL, ss
        macro_rules! adc_hlss { ($ss: ident) => (
            add_hl!(self.$ss.get() + (self.f >> 7) & 0x1)
        ) }

        // CP A, s
        macro_rules! cp_a { ($s: expr) => ({
            self.f = ADD_SUB_FLAG;
            if (self.a & 0xf) < ($s & 0xf) { self.f |= HALF_CARRY_FLAG; }
            if self.a < $s { self.f |= CARRY_FLAG; }
            if self.a == $s { self.f |= ZERO_FLAG; }
        }) }
        macro_rules! cp_an { () => (cp_a!(get_n!())) }
        macro_rules! cp_ar { ($r: ident) => (cp_a!(self.$r)) }
        macro_rules! cp_aHL { () => (cp_a!(mem.lb(self.hl().get()))) }

        // SUB A, s
        macro_rules! sub_a { ($s: expr) => ({
            cp_a!($s);
            self.a -= $s;
        }) }
        macro_rules! sub_an { () => (sub_a!(get_n!())) }
        macro_rules! sub_ar { ($r: ident) => (sub_a!(self.$r)) }
        macro_rules! sub_aHL { () => (sub_a!(mem.lb(self.hl().get()))) }

        // SBC A, s
        macro_rules! sbc_a { ($s: expr) => (
            sub_a!($s + (self.f >> 7))
        ) }
        macro_rules! sbc_an { () => (sbc_a!(get_n!())) }
        macro_rules! sbc_ar { ($r: ident) => (sbc_a!(self.$r)) }
        macro_rules! sbc_aHL { () => (sbc_a!(mem.lb(self.hl().get()))) }

        // AND A, s
        macro_rules! and_a { ($s: expr) => ({
            self.a &= $s;
            self.f = HALF_CARRY_FLAG;
            if self.a == 0 { self.f |= ZERO_FLAG; }
        }) }
        macro_rules! and_an { () => (and_a!(get_n!())) }
        macro_rules! and_ar { ($r: ident) => (and_a!(self.$r)) }
        macro_rules! and_aHL { () => (and_a!(mem.lb(self.hl().get()))) }

        // OR A, s
        macro_rules! or_a { ($s: expr) => ({
            self.a |= $s;
            self.f = 0;
            if self.a == 0 { self.f |= ZERO_FLAG; }
        }) }
        macro_rules! or_an { () => (or_a!(get_n!())) }
        macro_rules! or_ar { ($r: ident) => (or_a!(self.$r)) }
        macro_rules! or_aHL { () => (or_a!(mem.lb(self.hl().get()))) }

        // XOR A, s
        macro_rules! xor_a { ($s: expr) => ({
            self.a ^= $s;
            self.f = 0;
            if self.a == 0 { self.f |= ZERO_FLAG; }
        }) }
        macro_rules! xor_an { () => (xor_a!(get_n!())) }
        macro_rules! xor_ar { ($r: ident) => (xor_a!(self.$r)) }
        macro_rules! xor_aHL { () => (xor_a!(mem.lb(self.hl().get()))) }

        // INC r
        macro_rules! inc_r { ($r: ident) => ({
            self.$r += 1;
            self.f &= CARRY_FLAG;
            if self.$r & 0xf == 0 { self.f |= HALF_CARRY_FLAG; }
            if self.$r == 0 { self.f |= ZERO_FLAG; }
        }) }
        // INC (HL)
        macro_rules! inc_HL { () => ({
            let val = mem.lb(self.hl().get()) + 1;
            mem.sb(self.hl().get(), val);
            
            self.f &= CARRY_FLAG;
            if val & 0xfu8 == 0 { self.f |= HALF_CARRY_FLAG; }
            if val == 0 { self.f |= ZERO_FLAG; }
        }) }
        // INC ss
        macro_rules! inc_ss { ($ss: ident) => ({
            let val = self.$ss().get() + 1;
            self.$ss().set(val);
        }) }
        // INC sp
        macro_rules! inc_sp { () => ({
            self.sp += 1;
        }) }
        // DEC r
        macro_rules! dec_r { ($r: ident) => ({
            self.$r -= 1;
            self.f &= CARRY_FLAG;
            self.f |= ADD_SUB_FLAG;
            if self.$r & 0xf == 0 { self.f |= HALF_CARRY_FLAG; }
            if self.$r == 0 { self.f |= ZERO_FLAG; }
        }) }
        // DEC (HL)
        macro_rules! dec_HL { () => ({
            let val = mem.lb(self.hl().get()) - 1;
            mem.sb(self.hl().get(), val);
            
            self.f &= CARRY_FLAG;
            self.f |= ADD_SUB_FLAG;
            if val & 0xFu8 == 0 { self.f |= HALF_CARRY_FLAG; }
            if val == 0 { self.f |= ZERO_FLAG; }
        }) }
        // DEC ss
        macro_rules! dec_ss { ($ss: ident) => ({
            let val = self.$ss().get() - 1;
            self.$ss().set(val);
        }) }
        // DEC sp
        macro_rules! dec_sp { () => ({
            self.sp -= 1;
        }) }

        // DAA
        macro_rules! daa { () => ({
            // Unimplemented
        }) }

        // CPL
        macro_rules! cpl { () => ({
            self.a ^= 0xFF;
            self.f |= HALF_CARRY_FLAG | ADD_SUB_FLAG;
        }) }
        // CCF
        macro_rules! ccf { () => ({
            // Not sure if we should copy the carry flag to the half carry flag
            if cflag!() { self.f |= HALF_CARRY_FLAG } else { self.f &= !HALF_CARRY_FLAG }
            self.f ^= CARRY_FLAG
        }) }
        // SCF
        macro_rules! scf { () => ({
            self.f |= CARRY_FLAG
        }) }
        // NOP
        macro_rules! nop { () => ( {} ) }
        // HALT
        macro_rules! halt { () => ( self.halt = true; ) }
        // DI
        macro_rules! di { () => ( self.disable_interrupts(mem); ) }
        // EI
        macro_rules! ei { () => ( self.enable_interrupts(mem); ) }

        // RLCA
        macro_rules! rlca { () => ({
            let copy_bit = self.a >> 7;
            self.a = (self.a << 1) + copy_bit;
            if copy_bit == 0 { self.f &= !CARRY_FLAG } else { self.f |= CARRY_FLAG };     
        }) }
        // RLA
        macro_rules! rla { () => ({
            let copy_bit = if self.f & CARRY_FLAG == 0 { 0 } else { 1 };
            if self.a >> 7 == 0 { self.f &= !CARRY_FLAG } else { self.f |= CARRY_FLAG };   
            self.a = (self.a << 1) + copy_bit;
        }) }
        // RRCA
        macro_rules! rrca { () => ({
            let copy_bit = self.a & 0x1;
            if copy_bit == 0 { self.f &= !CARRY_FLAG } else { self.f |= CARRY_FLAG };   
            self.a = (self.a >> 1) + (copy_bit << 7);
        }) }
        // RRA
        macro_rules! rra { () => ({
            let copy_bit = if self.f & CARRY_FLAG == 0 { 0 } else { 1 };
            if self.a & 0x1 == 0 { self.f &= !CARRY_FLAG } else { self.f |= CARRY_FLAG };   
            self.a = (self.a >> 1) + (copy_bit << 7);
        }) }

        // JP nn
        macro_rules! jp_nn { () => (
            self.pc = get_nn!();
        ) }
        // JP c, nn
        macro_rules! jp_cnn { ($c: expr) => (
            if $c { jp_nn!(); 4 } else { self.pc += 2; 3 }
        ) }
        // JP hl
        macro_rules! jp_hl { () => (
            self.pc = self.hl().get()
        ) }
        // JR e
        macro_rules! jr_e { () => ({
            // Get offset as a 2s-complement integer
            let offset = (mem.lb(self.bump()) as i8) as i16;
            self.pc = (self.pc as i16 + offset) as u16;
        }) }
        // JR c, e
        macro_rules! jr_ce { ($c: expr) => (
            if $c { jr_e!(); 3 } else { self.pc += 1; 2 }
        ) }
        // JP (HL)
        macro_rules! jp_HL { () => (
            self.pc = mem.lw(self.hl().get());
        ) }

        // CALL nn
        macro_rules! call_nn { () => ({
            self.sp -= 2;
            mem.sw(self.sp, self.pc);
            self.pc = get_nn!();
        }) }
        // CALL c, nn
        macro_rules! call_cnn { ($cc: expr) => (
            if $cc { call_nn!(); 6 } else { self.pc += 2; 3 }
        ) }
        // RET
        macro_rules! ret { () => ({
            self.pc = mem.lw(self.sp);
            self.sp += 2;
        }) }
        // RET c
        macro_rules! ret_c { ($c: expr) => (
            if $c { ret!(); 3 } else { 1 }
        ) }
        // RETI
        macro_rules! reti { () => ({
            self.enable_interrupts(mem);
            ret!();
        }) }
        // RST p
        macro_rules! rst_p { ($p: expr) => ({
            self.sp -= 2;
            mem.sw(self.sp, self.pc);
            self.pc = $p;
        }) }
        
        //
        // Added instructions for GB
        //

        // ADD SP, nn
        macro_rules! add_spnn { () => ({
            let offset = get_nn!() as i16;
            self.sp = (self.sp as i16 + offset) as u16;
        }) }
        // LDI A, (HL)
        macro_rules! ldi_aHL { () => ({
            self.a = self.hl().get() as u8;
            inc_HL!();
        }) }
        // LDI (HL), A
        macro_rules! ldi_HLa { () => ({
            self.hl().set(self.a as u16);
            inc_HL!();
        }) }
        // LDD A, (HL)
        macro_rules! ldd_aHL { () => ({
            self.a = self.hl().get() as u8;
            dec_HL!();
        }) }
        // LDD (HL), A
        macro_rules! ldd_HLa { () => ({
            let a = self.a as u16;
            self.hl().set(a);
            dec_HL!();
        }) }
        // LD HL,SP+n
        macro_rules! ld_hlspn { () => ({
            let val = ((get_n!() as i8) as i32 + self.sp as i32) as u16;
            self.hl().set(val);
        }) }
        
        // STOP
        macro_rules! stop { () => ( self.stop = true; ) }
        
        // Fetch the next instruction from memory
        let op = mem.lb(self.bump());
        
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
            0x07 => { rlca!();              2 },
            0x08 => { ld_NNsp!();           5 },
            0x09 => { add_hlss!(bc);        2 },
            0x0A => { ld_rDD!(a, bc);       2 },
            0x0B => { dec_ss!(bc);          2 },
            0x0C => { inc_r!(c);            1 },
            0x0D => { dec_r!(c);            1 },
            0x0E => { ld_rn!(c);            2 },
            0x0F => { rrca!();              2 },

            0x10 => { stop!();              1 },
            0x11 => { ld_ddnn!(de);         3 },
            0x12 => { ld_DDr!(de, a);       2 },
            0x13 => { inc_ss!(de);          2 },
            0x14 => { inc_r!(d);            1 },
            0x15 => { dec_r!(d);            1 },
            0x16 => { ld_rn!(d);            2 },
            0x17 => { rla!();               2 },
            0x18 => { jr_e!();              3 },
            0x19 => { add_hlss!(de);        2 },
            0x1A => { ld_rDD!(a, de);       2 },
            0x1B => { dec_ss!(de);          2 },
            0x1C => { inc_r!(e);            1 },
            0x1D => { dec_r!(e);            1 },
            0x1E => { ld_rn!(e);            2 },
            0x1F => { rra!();               2 },

            0x20 => { jr_ce!(!zflag!())       }, // 3;2 
            0x21 => { ld_ddnn!(hl);         3 },
            0x22 => { ld_DDr!(hl, a);       2 },
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
            0x31 => { ld_NNsp!();           3 },
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
            0xC3 => { call_cnn!(!zflag!())    }, // 6;3   
            0xC4 => { jp_nn!();             4 },
            0xC5 => { push_qq!(bc);         3 },
            0xC6 => { add_an!();            2 },
            0xC7 => { rst_p!(0x00);         4 },
            0xC8 => { ret_c!(zflag!())        }, // 5;2
            0xC9 => { ret!();               4 },
            0xCA => { jp_cnn!(zflag!())       }, // 4;3
            0xCB => self.exec_long(mem),
            0xCC => { call_cnn!(zflag!())     }, // 6;3           
            0xCD => { call_nn!();           6 },
            0xCE => { adc_an!();            2 },
            0xCF => { rst_p!(0x08);         4 },

            0xD0 => { ret_c!(!cflag!())       }, // 5;2
            0xD1 => { pop_qq!(de);          3 },
            0xD2 => { jp_cnn!(!cflag!())      }, // 4;3
            0xD3 => self.invalid_inst(op),
            0xD4 => { call_cnn!(!cflag!())    }, // 6;3
            0xD5 => { push_qq!(de);         3 },
            0xD6 => { sub_an!();            2 },
            0xD7 => { rst_p!(0x10);         4 },
            0xD8 => { ret_c!(cflag!())        }, // 5;2
            0xD9 => { reti!();              4 },
            0xDA => { jp_cnn!(cflag!())       }, // 4;3
            0xDB => self.invalid_inst(op),
            0xDC => { call_cnn!(cflag!())     }, // 6;3           
            0xDD => self.invalid_inst(op),
            0xDE => { sbc_an!();            2 },
            0xDF => { rst_p!(0x18);         4 },

            0xE0 => { ld_IOan!();           3 },
            0xE1 => { pop_qq!(hl);          3 },
            0xE2 => { ld_IOca!();           2 },
            0xE3 => self.invalid_inst(op),
            0xE4 => self.invalid_inst(op),
            0xE5 => { push_qq!(hl);         3 },
            0xE6 => { and_an!();            2 },
            0xE7 => { rst_p!(0x20);         4 },
            0xE8 => { add_spn!();           4 },
            0xE9 => { jp_hl!();             1 },
            0xEA => { ld_NNr!(a);           4 },            
            0xEB => self.invalid_inst(op),
            0xEC => self.invalid_inst(op),
            // The GB processor does not include the ED set of instructions from the z80 processor
            0xED => self.invalid_inst(op),
            0xEE => { xor_an!();            2 },
            0xEF => { rst_p!(0x28);         4 },

            0xF0 => { ld_aIOn!();           3 },
            0xF1 => { pop_qq!(af);          3 },
            0xF2 => { ld_aIOc!();           2 },
            0xF3 => { di!();                1 },
            0xF4 => self.invalid_inst(op),
            0xF5 => { push_qq!(af);         3 },
            0xF6 => { or_an!();             2 },
            0xF7 => { rst_p!(0x30);         4 },
            0xF8 => { ld_hlspn!();          3 },
            0xF9 => { ld_sphl!();           2 },
            0xFA => { ld_rNN!(a);           4 },             
            0xFB => { ei!();                1 },
            0xFC => self.invalid_inst(op),       
            0xFD => self.invalid_inst(op),
            0xFE => { cp_an!();             2 },
            0xFF => { rst_p!(0x38);         4 },
            
            _ => unreachable!()
        }
    }

    /// Execute 2-byte opcodes, returning the number of cycles required for executing the
    /// instruction
    fn exec_long(&mut self, mem: &mut Memory) -> u8 {
        //
        // Implementation of z80 two byte instructions
        //

        // RLC r
        macro_rules! rlc_r { ($r: ident) => ({
            let copy_bit = self.$r >> 7;
            self.$r = (self.$r << 1) + copy_bit;
            self.f = if copy_bit == 0 { 0 } else { CARRY_FLAG };
            
            if self.$r == 0 { self.f |= ZERO_FLAG; }            
        }) }

        // RLC (HL)
        macro_rules! rlc_HL { () => ({
            let addr = self.hl().get();
            let val = mem.lb(addr);
            
            let copy_bit = val >> 7;
            self.f = if copy_bit == 0 { 0 } else { CARRY_FLAG };
                        
            mem.sb(addr, (val << 1) + copy_bit);
            
            if (val << 1) + copy_bit == 0 { self.f |= ZERO_FLAG; }
        }) }

        // RL r
        macro_rules! rl_r { ($r: ident) => ({
            let copy_bit = if self.f & CARRY_FLAG == 0 { 0 } else { 1 };
            self.f = if self.$r >> 7 == 0 { 0 } else { CARRY_FLAG };
            self.$r = (self.$r << 1) + copy_bit;
            
            if self.$r == 0 { self.f |= ZERO_FLAG; }            
        }) }

        // RL (HL)
        macro_rules! rl_HL { () => ({
            let addr = self.hl().get();
            let val = mem.lb(addr);
            
            let copy_bit = if self.f & CARRY_FLAG == 0 { 0 } else { 1 };
            self.f = if val >> 7 == 0 { 0 } else { CARRY_FLAG };
            mem.sb(addr, (val << 1) + copy_bit);
            
            if (val << 1) + copy_bit == 0 { self.f |= ZERO_FLAG; }
        }) }

        // RRC r
        macro_rules! rrc_r { ($r: ident) => ({
            let copy_bit = self.$r & 0x1;
            self.f = if copy_bit == 0 { 0 } else { CARRY_FLAG };
            self.$r = (self.$r >> 1) + (copy_bit << 7);
                        
            if self.$r == 0 { self.f |= ZERO_FLAG; }      
        }) }

        // RRC (HL)
        macro_rules! rrc_HL { () => ({
            let addr = self.hl().get();
            let val = mem.lb(addr);
            
            let copy_bit = val & 0x1;
            self.f = if copy_bit == 0 { 0 } else { CARRY_FLAG };
            mem.sb(addr, (val >> 1) + (copy_bit << 7));
            
            if (val << 1) + copy_bit == 0 { self.f |= ZERO_FLAG; }   
        }) }

        // RR r
        macro_rules! rr_r { ($r: ident) => ({
            let copy_bit = if self.f & CARRY_FLAG == 0 { 0 } else { 1 };
            self.f = if self.$r & 0x1 == 0 { 0 } else { CARRY_FLAG };
            self.$r = (self.$r >> 1) + (copy_bit << 7);
                        
            if self.$r == 0 { self.f |= ZERO_FLAG; }      
        }) }

        // RR (HL)
        macro_rules! rr_HL { () => ({
            let addr = self.hl().get();
            let val = mem.lb(addr);
            
            let copy_bit = if self.f & CARRY_FLAG == 0 { 0 } else { 1 };
            self.f = if val & 0x1 == 0 { 0 } else { CARRY_FLAG };
            mem.sb(addr, (val >> 1) + (copy_bit << 7));
                        
            if (val >> 1) + (copy_bit << 7) == 0 { self.f |= ZERO_FLAG; }      
        }) }

        // SLA r
        macro_rules! sla_r { ($r: ident) => ({
            self.f = if self.$r >> 7 == 0 { 0 } else { CARRY_FLAG };
            self.$r = self.$r << 1;
            if self.$r == 0 { self.f |= ZERO_FLAG; }        
        }) }

        // SLA (HL)
        macro_rules! sla_HL { () => ({
            let addr = self.hl().get();
            let val = mem.lb(addr);
            
            self.f = if val >> 7 == 0 { 0 } else { CARRY_FLAG };
            mem.sb(addr, val << 1);
            if (val << 1) == 0 { self.f |= ZERO_FLAG; }        
        }) }
        
        // SRA r
        macro_rules! sra_r { ($r: ident) => ({
            self.f = if self.$r & 1 == 0 { 0 } else { CARRY_FLAG };
            self.$r = (self.$r as i8 >> 1) as u8;
            if self.$r == 0 { self.f |= ZERO_FLAG; }        
        }) }

        // SRA (HL)
        macro_rules! sra_HL { () => ({
            let addr = self.hl().get();
            let mut val = mem.lb(addr);
            
            self.f = if val & 1 == 0 { 0 } else { CARRY_FLAG };
            val = (val as i8 >> 1) as u8;
            mem.sb(addr, val);
            if val == 0 { self.f |= ZERO_FLAG; }        
        }) }
        
        // SRL r
        macro_rules! srl_r { ($r: ident) => ({
            self.f = if self.$r & 1 == 0 { 0 } else { CARRY_FLAG };
            self.$r = self.$r >> 1;
            if self.$r == 0 { self.f |= ZERO_FLAG; }        
        }) }
        
        // SRL (HL)
        macro_rules! srl_HL { () => ({
            let addr = self.hl().get();
            let mut val = mem.lb(addr);
            
            self.f = if val & 1 == 0 { 0 } else { CARRY_FLAG };
            val = val >> 1;
            mem.sb(addr, val);
            if val == 0 { self.f |= ZERO_FLAG; }        
        }) }

        // BIT b, m
        macro_rules! bit_b { ($b: expr, $m: expr) => ({
            self.f |= HALF_CARRY_FLAG;
            self.f &= !ADD_SUB_FLAG;
            if $m & (1 << $b) == 0 { self.f |= ZERO_FLAG; } else { self.f &= !ZERO_FLAG; }
        }) }
        macro_rules! bit_br { ($b: expr, $r: ident) => (bit_b!($b, self.$r)) }
        macro_rules! bit_bHL { ($b: expr) => (bit_b!($b, mem.lb(self.hl().get()))) }

        // SET b, r
        macro_rules! set_br { ($b: expr, $r: ident) => ({
            self.$r |= 1 << $b;
        }) }

        // SET b, (HL)
        macro_rules! set_bHL { ($b: expr) => ({
            let addr = self.hl().get();
            let val = mem.lb(addr) | (1 << $b);
            mem.sb(addr, val);
        }) }

        // RES b, r
        macro_rules! res_br { ($b: expr, $r: ident) => ({
            self.$r &= !(1 << $b);
        }) }

        // RES b, (HL)
        macro_rules! res_bHL { ($b: expr) => ({
            let addr = self.hl().get();
            let val = mem.lb(addr) & !(1 << $b);
            mem.sb(addr, val);
        }) }

        // SWAP r
        macro_rules! swap_r { ($r: ident) => ({
            self.$r = (self.$r << 4) | (self.$r >> 4); 
        }) }

        // SWAP HL
        macro_rules! swap_HL { () => ({
            let addr = self.hl().get();
            let val = mem.lb(addr);
            mem.sb(addr, (val << 4) | (val >> 4));
        }) }
        
        // Fetch the second byte of the instruction
        let op = mem.lb(self.bump());

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

    /// Enables CPU interrupts
    fn enable_interrupts(&mut self, _mem: &mut Memory) {
        self.ime = 1;
    }

    /// Disables CPU interrupts
    fn disable_interrupts(&mut self, _mem: &mut Memory) {
        self.ime = 0;
    }

    /// Handle instructions corresponding to invalid opcodes
    fn invalid_inst(&mut self, _opcode: u8) -> u8 {
        panic!("invalid instruction");
    }

    /// Increments pc, returning the old value of pc
    pub fn bump(&mut self) -> u16 {
        let old = self.pc;
        self.pc += 1;
        old
    }
}

struct JointReg<'a> {
    high: &'a mut u8,
    low: &'a mut u8,
}

impl<'a> JointReg<'a> {
    #[inline(always)]
    pub fn set(&mut self, value: u16) {
        *self.high = (value >> 8) as u8;
        *self.low = value as u8;
    }
    
    #[inline(always)]
    pub fn get(&self) -> u16 {
        (*self.high as u16 << 8) + *self.low as u16
    }
}
