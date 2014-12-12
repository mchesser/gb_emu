use std::slice::bytes::copy_memory;

use mmu::Memory;
use cpu::Cpu;
use graphics;
use timer;

#[allow(dead_code)] // This code is not dead
pub enum DeviceMode {
    GameBoy,
    SuperGameBoy,
    GameBoyColor,
}

pub struct Emulator {
    pub mem: Memory,
    cpu: Cpu,
    cycles: i32,
}

impl Emulator {
    pub fn new() -> Emulator {
        Emulator {
            mem: Memory::new(),
            cpu: Cpu::new(),
            cycles: 0,
        }
    }

    pub fn load_cart(&mut self, data: &[u8]) {
        for (i, chunk) in data.chunks(0x4000).enumerate() {
            copy_memory(&mut self.mem.rom[i], chunk);
        }
    }

    pub fn start(&mut self) {
        self.cpu.start_up();
        self.mem.start_up(DeviceMode::GameBoy);
    }

    /// Run the emulator for one frame
    pub fn frame(&mut self) {
        if self.mem.crashed | self.cpu.crashed {
            return;
        }

        while self.cycles <= graphics::timings::FULL_FRAME as i32 {
            let instruction_time = self.cpu.step(&mut self.mem);
            timer::step(&mut self.mem, instruction_time);
            graphics::step(&mut self.mem, instruction_time);
            self.cycles += instruction_time as i32;
        }
        self.cycles -= graphics::timings::FULL_FRAME as i32;
    }

    /// Returns the internal display
    pub fn display(&mut self) -> &[u8] {
        &self.mem.gpu.framebuffer
    }
}

