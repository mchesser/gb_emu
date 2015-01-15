use mmu::Memory;
use cpu::Interrupt;

#[allow(missing_copy_implementations)]
pub struct Timer {
    /// Divider register (mapped to: 0xFF04)
    pub div: u8,
    /// Timer counter register (mapped to: 0xFF05)
    pub tima: u8,
    /// Timer modulo register (mapped to: 0xFF06)
    pub tma: u8,
    /// Timer control register (mapped to: 0xFF07)
    pub tac: u8,

    // Internal timers used for synchronizing with the CPU
    internal_div: u16,
    internal_tima: u16,
}

impl Timer {
    pub fn new() -> Timer {
        Timer {
            div: 0,
            tima: 0,
            tma: 0,
            tac: 0,

            internal_div: 0,
            internal_tima: 0,
        }
    }

    fn get_tima_speed(&self) -> u16 {
         match self.tac & 0x3 {
            0x0 => 256,
            0x1 => 4,
            0x2 => 16,
            0x3 => 64,
            _ => unreachable!(),
        }
    }
}


pub fn step(mem: &mut Memory, ticks: u8) {
    mem.timer.internal_div += ticks as u16 / 4;
    mem.timer.internal_tima += ticks as u16 / 4;

    if mem.timer.internal_div >= 64 {
        tick_divider(mem);
        mem.timer.internal_div -= 64;
    }
    if mem.timer.tac & 0x4 != 0 {
        let speed = mem.timer.get_tima_speed();
        if mem.timer.internal_tima >= speed {
            tick_counter(mem);
            mem.timer.internal_tima -= speed;
        }
    }
}


/// Ticks the divider register by 1. This function should be called at a rate of `16,384`.
fn tick_divider(mem: &mut Memory) {
    mem.timer.div += 1;
}

/// Ticks the timer counter register by 1. This function should be called at the frequency specified
/// in the timer control register.
fn tick_counter(mem: &mut Memory) {
    mem.timer.tima += 1;

    // Set interrupt bit if the value overflowed
    if mem.timer.tima == 0 {
        mem.if_reg |= Interrupt::Timer as u8;
    }
}
