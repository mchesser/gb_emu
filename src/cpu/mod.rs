use cpu::exec::fetch_exec;
use mmu::MemMap;

pub mod exec;

/// The clock speed of the CPU in nanoseconds.
pub const T_CLOCK_SPEED: f32 = 238.418; // == 4,194,304Hz


#[deriving(PartialEq)]
pub enum State {
    Running,
    Stopped,
    Halted,
    Crashed,
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
pub struct Cpu {
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
            a: 0, b: 0, c: 0, d: 0, e: 0, f: 0, h: 0, l: 0,

            ime: 0,

            sp: 0,
            pc: 0,

            state: Running,
            clock: 0,
        }
    }

    /// Steps the CPU.
    pub fn step(&mut self, mem: &mut MemMap) {
        if self.state == Crashed {
            return;
        }

        // Check for interrupts

        let cost = fetch_exec(self, mem);
        self.clock += cost;
    }

    /// Enables CPU interrupts
    fn enable_interrupts(&mut self, _mem: &mut MemMap) {
        self.ime = 1;
    }

    /// Disables CPU interrupts
    fn disable_interrupts(&mut self, _mem: &mut MemMap) {
        self.ime = 0;
    }

    /// Handle instructions corresponding to invalid opcodes
    fn invalid_inst(&mut self, _opcode: u8) -> u8 {
        self.state = Crashed;
        0
    }

    /// Halt the cpu
    fn halt(&mut self) {
        self.state = Halted;
    }

    /// Stop the cpu
    fn stop(&mut self) {
        self.state = Stopped;
    }

    /// Increments pc, returning the old value of pc
    pub fn bump(&mut self) -> u16 {
        let old = self.pc;
        self.pc += 1;
        old
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

join_regs!((b, c) as bc)
join_regs!((d, e) as de)
join_regs!((h, l) as hl)
join_regs!((a, f) as af)

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
