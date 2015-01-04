use std::num::Int;

use cpu::exec::fetch_exec;
use mmu::Memory;

pub mod exec;

const INTERRUPT_TABLE: &'static [u16] = &[
    0x0040,   // V-Blank
    0x0048,   // LCD STAT
    0x0050,   // Timer
    0x0058,   // Serial
    0x0060,   // Joypad
];

#[derive(Copy)]
#[repr(u8)]
pub enum Interrupt {
    VBlank = 0b00000001,
    Stat   = 0b00000010,
    Timer  = 0b00000100,
    Serial = 0b00001000,
    Joypad = 0b00010000,
}

#[derive(PartialEq, Copy)]
pub enum State {
    Running,
    Stopped,
    Halted,
}

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
#[allow(missing_copy_implementations)]
pub struct Cpu {
    pub crashed: bool,

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

    pub state: State,
    pub clock: u8,
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            crashed: true,

            a: 0, b: 0, c: 0, d: 0, e: 0, f: 0, h: 0, l: 0,

            ime: 0,

            sp: 0,
            pc: 0,

            state: State::Running,
            clock: 0,
        }
    }

    pub fn start_up(&mut self) {
        // TODO(major): Add support for SGB and CGB
        self.af().set(0x01B0);
        self.bc().set(0x0013);
        self.de().set(0x00D8);
        self.hl().set(0x014D);
        self.sp = 0xFFFE;
        self.pc = 0x0100;

        self.crashed = false;
    }

    /// Steps the CPU returning the number of elapsed cycles
    pub fn step(&mut self, mem: &mut Memory) -> u8 {
        if self.crashed {
            // If the CPU has crashed, then we can't do anything
            return 0xFF;
        }

        let mut elapsed_cycles = 0;
        if self.state == State::Running {
            elapsed_cycles += fetch_exec(self, mem);
        }
        else {
            elapsed_cycles += 1;
        }

        if self.ime != 0 || self.state == State::Halted {
            elapsed_cycles += self.handle_interrupts(mem);
        }

        // TODO: add support for CGB double speed mode
        self.clock += elapsed_cycles * 4;
        elapsed_cycles * 4
    }

    /// Handles interrupts
    /// Returns the amount of elapsed cycles (if any) as a result of processing interrupts.
    fn handle_interrupts(&mut self, mem: &mut Memory) -> u8 {
        let interrupts = mem.if_reg & mem.ie_reg;
        if interrupts == 0 {
            // No interrupts to handle
            return 0;
        }

        // Get the highest priority interrupt
        let i = interrupts.trailing_zeros();

        // Clear the interrupt bit if the CPU is not halted
        if self.state != State::Halted {
            mem.if_reg &= !(1 << i);
        }

        self.disable_interrupts(mem);
        self.call(mem, INTERRUPT_TABLE[i]);
        self.state = State::Running;

        1
    }

    /// Enables CPU interrupts
    pub fn enable_interrupts(&mut self, _mem: &mut Memory) {
        self.ime = 1;
    }

    /// Disables CPU interrupts
    pub fn disable_interrupts(&mut self, _mem: &mut Memory) {
        self.ime = 0;
    }

    /// Jumps to a location
    pub fn jump(&mut self, addr: u16) {
        self.pc = addr;
    }

    pub fn push16(&mut self, mem: &mut Memory, val: u16) {
        self.sp -= 2;
        mem.sw(self.sp, val);
    }

    pub fn pop16(&mut self, mem: &mut Memory) -> u16 {
        self.sp += 2;
        mem.lw(self.sp - 2)
    }

    pub fn call(&mut self, mem: &mut Memory, addr: u16) {
        self.sp -= 2;
        mem.sw(self.sp, self.pc);
        self.pc = addr;
    }

    pub fn ret(&mut self, mem: &mut Memory) {
        self.pc = mem.lw(self.sp);
        self.sp += 2;
    }

    /// Halt the cpu
    pub fn halt(&mut self) {
        self.state = State::Halted;
    }

    /// Stop the cpu
    pub fn stop(&mut self) {
        self.state = State::Stopped;
    }

    /// Increments the program counter, returning the old value
    pub fn bump(&mut self) -> u16 {
        self.pc += 1;
        self.pc - 1
    }

    /// Handle instructions corresponding to invalid opcodes
    fn invalid_inst(&mut self, _opcode: u8) -> u8 {
        self.crashed = true;
        0
    }
}

// The 8-bit registers in the main register set can be joint to form 16-bit registers.
macro_rules! join_regs { (($r1: ident, $r2: ident) as $name: ident) => (
    impl Cpu {
        pub fn $name(&mut self) -> JointReg {
            JointReg {
                high: &mut self.$r1,
                low: &mut self.$r2,
            }
        }
    }
) }

join_regs!((b, c) as bc);
join_regs!((d, e) as de);
join_regs!((h, l) as hl);
join_regs!((a, f) as af);

pub struct JointReg<'a> {
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
        ((*self.high as u16) << 8) + *self.low as u16
    }
}
