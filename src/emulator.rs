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
    pub fn frame<F, G>(&mut self, mut on_tick: F, mut on_vblank: G)
        where F: FnMut(&mut Cpu, &mut Memory),
              G: FnMut(&mut Cpu, &mut Memory)
    {
        if self.mem.crashed | self.cpu.crashed {
            return;
        }

        while self.cycles <= graphics::timings::FULL_FRAME as i32 {
            on_tick(&mut self.cpu, &mut self.mem);

            let instruction_time = self.cpu.step(&mut self.mem);
            timer::step(&mut self.mem, instruction_time);
            graphics::step(&mut self.mem, instruction_time);
            if self.mem.gpu.vblank_flag {
               self.mem.gpu.vblank_flag = false;
               on_vblank(&mut self.cpu, &mut self.mem);
            }

            self.cycles += instruction_time as i32;
        }
        self.cycles -= graphics::timings::FULL_FRAME as i32;
    }
}

