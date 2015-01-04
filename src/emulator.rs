use mmu::Memory;
use cart::SaveFile;
use cpu::Cpu;
use graphics;
use timer;

#[derive(Copy)]
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

    pub fn load_cart(&mut self, data: &[u8], save_file: Option<Box<SaveFile>>) {
        self.mem.cart.load(data, save_file);
    }

    /// Initialise the emulator with the expected startup values
    pub fn start(&mut self) {
        self.cpu.start_up();
        self.mem.start_up(DeviceMode::GameBoy);
    }

    /// Run the emulator for one frame. To emulate the Game Boy system realistically this should be
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

    /// Returns true if the framebuffer has been flipped since the last call to this function
    pub fn poll_screen(&mut self) -> bool {
        if self.mem.gpu.ready_flag {
            self.mem.gpu.ready_flag = false;
            true
        }
        else {
            false
        }
    }

    pub fn front_buffer(&self) -> &[u8] {
        &self.mem.gpu.framebuffer[1 - self.mem.gpu.backbuffer]
    }

    pub fn front_buffer_mut(&mut self) -> &mut [u8] {
        &mut self.mem.gpu.framebuffer[1 - self.mem.gpu.backbuffer]
    }
}

