use std::slice::bytes::copy_memory;

use mmu::Memory;
use cpu::Cpu;
use graphics;
use timer;

#[deriving(Copy)]
pub enum DeviceMode {
    GameBoy,
    SuperGameBoy,
    GameBoyColor,
}

pub struct Emulator<F> {
    pub mem: Memory,
    cpu: Cpu,
    cycles: i32,
    extern_fn: F,
}

impl<F> Emulator<F> where F: FnMut(&mut Cpu, &mut Memory) {
    pub fn new(extern_fn: F) -> Emulator<F> {
        Emulator {
            mem: Memory::new(),
            cpu: Cpu::new(),
            cycles: 0,
            extern_fn: extern_fn,
        }
    }

    /// Load a cart into the emulator rom
    pub fn load_cart(&mut self, data: &[u8]) {
        for (i, chunk) in data.chunks(0x4000).enumerate() {
            copy_memory(&mut self.mem.rom[i], chunk);
        }
    }

    /// Initialise the emulator with the expected startup values
    pub fn start(&mut self) {
        self.cpu.start_up();
        self.mem.start_up(DeviceMode::GameBoy);
    }

    /// Run the emulator for one frame. To emulate the gameboy system realistically this should be
    /// called at 60 Hz
    pub fn frame(&mut self) {
        if self.mem.crashed | self.cpu.crashed {
            return;
        }

        while self.cycles <= graphics::timings::FULL_FRAME as i32 {
            (self.extern_fn)(&mut self.cpu, &mut self.mem);

            let instruction_time = self.cpu.step(&mut self.mem);
            timer::step(&mut self.mem, instruction_time);
            graphics::step(&mut self.mem, instruction_time);
            self.cycles += instruction_time as i32;
        }
        self.cycles -= graphics::timings::FULL_FRAME as i32;
    }

    pub fn framebuffer(&self) -> &[u8] {
        &self.mem.gpu.framebuffer
    }

    pub fn framebuffer_mut(&mut self) -> &mut [u8] {
        &mut self.mem.gpu.framebuffer
    }
}

