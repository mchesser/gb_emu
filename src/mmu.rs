use emulator::DeviceMode;
use cart::Cartridge;
use graphics;
use graphics::Gpu;
use sound::SoundController;
use joypad::Joypad;
use timer::Timer;

/// The GB/C memory mapper
pub struct Memory {
    pub crashed: bool,

    pub sb: u8,
    pub sc: u8,

    /// Interrupt enabled register (mapped to: 0xFFFF)
    pub ie_reg: u8,
    /// Interrupt flags register (mapped to: 0xFF0F)
    pub if_reg: u8,

    // The inserted cartridge
    pub cart: Cartridge,

    /// Working ram (mapped to: 0xC000-0xDFFF, shadow: 0xE000-0xFDFF)
    pub working_ram: [u8; 0x2000],
    /// The currently mapped working ram bank (CGB only)
    pub wram_bank: usize,

    /// Zero-page high speed ram (mapped to: 0xFF80-0xFFFE)
    pub fast_ram: [u8; 0x7F],

    /// GB graphics processor
    pub gpu: Gpu,

    /// GB sound controller
    pub sound: SoundController,

    /// GB joypad
    pub joypad: Joypad,

    /// GB Timer
    pub timer: Timer,
}

impl Memory {
    pub fn new() -> Memory {
        Memory {
            crashed: false,

            sb: 0,
            sc: 0,

            ie_reg: 0,
            if_reg: 0,

            cart: Cartridge::new(),

            working_ram: [0; 0x2000],
            wram_bank: 0,
            fast_ram: [0; 0x7F],

            gpu: Gpu::new(),
            sound: SoundController::new(),
            joypad: Joypad::new(),
            timer: Timer::new(),
        }
    }

    /// Initialise the memory with the expected data
    pub fn start_up(&mut self, mode: DeviceMode) {
        // GB start up sequence, see: http://problemkaputt.de/pandocs.htm#powerupsequence
        self.sb(0xFF05, 0x00); // TIMA - Reset timer counter
        self.sb(0xFF06, 0x00); // TMA  - Reset timer modulo
        self.sb(0xFF07, 0x00); // TAC  - Reset timer controller
        self.sb(0xFF10, 0x80); // NR10 - Sound channel 1 configuration
        self.sb(0xFF11, 0xBF); // NR11
        self.sb(0xFF12, 0xF3); // NR12
        self.sb(0xFF14, 0xBF); // NR14
        self.sb(0xFF16, 0x3F); // NR21 - Sound channel 2 configuration
        self.sb(0xFF17, 0x00); // NR22
        self.sb(0xFF19, 0xBF); // NR24
        self.sb(0xFF1A, 0x7F); // NR30 - Sound channel 2 configuration
        self.sb(0xFF1B, 0xFF); // NR31
        self.sb(0xFF1C, 0x9F); // NR32
        self.sb(0xFF1E, 0xBF); // NR33
        self.sb(0xFF20, 0xFF); // NR41 - Sound channel 4 configuration
        self.sb(0xFF21, 0x00); // NR42
        self.sb(0xFF22, 0x00); // NR43
        self.sb(0xFF23, 0xBF); // NR44
        self.sb(0xFF24, 0x77); // NR50 - Set sound master volume (Left = 7, Right = 7)
        self.sb(0xFF25, 0xF3); // NR51 - Set sound output settings
        self.sb(0xFF26, 0xF1); // NR52 - More sound settings (sound enabled = true)
        self.sb(0xFF40, 0x91); // LCDC - Set lcd control register
        self.sb(0xFF42, 0x00); // SCY  - Set screen scroll y = 0
        self.sb(0xFF43, 0x00); // SCX  - Set screen scroll x = 0
        self.sb(0xFF45, 0x00); // LYC  - Set LY compare register
        self.sb(0xFF47, 0xFC); // BGP  - Set background palette = White White White Black
        self.sb(0xFF48, 0xFF); // OBP0 - Set object palette 0 = Black Black Black Black
        self.sb(0xFF49, 0xFF); // OBP1 - Set object palette 1 = Black Black Black Black
        self.sb(0xFF4A, 0x00); // WY   - Set window y position
        self.sb(0xFF4B, 0x00); // WX   - Set window x position
        self.sb(0xFFFF, 0x00); // IE   - Disable all interrupts

        // Handle device specific settings
        match mode {
            DeviceMode::SuperGameBoy => {
                self.sb(0xFF26, 0xF0); // NR52 - More sound settings
            },

            DeviceMode::GameBoyColor => {
                self.sb(0xFF68, 0xC0); // BCPS/BGPI - Set background palette index (00, auto inc.)
                self.sb(0xFF6A, 0xC0); // BCPD/GBPD - Set sprite palette index (00, auto inc.)
            },

            DeviceMode::GameBoy => {},
        }
    }

    /// Handle invalid memory accesses
    fn invalid_memory(&mut self, addr: u16) {
        self.crashed = true;
        println!("Invalid write to address: 0x{:4X}", addr);
    }

    /// Load a byte from memory
    pub fn lb(&self, addr: u16) -> u8 {
        if self.crashed { return 0; };

        // Map memory reads to their correct bytes. (See: http://problemkaputt.de/pandocs.htm)
        match addr {
            0x0000 ... 0x7FFF | 0xA000 ... 0xBFFF => self.cart.read(addr),
            0x8000 ... 0x9FFF => self.gpu.vram()[(addr & 0x1FFF) as usize],
            // NOTE: Check this mapping when adding CGB support
            0xC000 ... 0xDFFF => self.working_ram[(addr & 0x1FFF) as usize],
            0xE000 ... 0xFDFF => self.working_ram[(addr & 0x1FFF) as usize],
            0xFE00 ... 0xFE9F => self.gpu.oam[(addr & 0x00FF) as usize],
            0xFEA0 ... 0xFEFF => 0xDE,
            0xFF00 ... 0xFF7F => self.read_io(addr),
            0xFF80 ... 0xFFFE => self.fast_ram[(addr & 0x7F) as usize],
            0xFFFF            => self.ie_reg,

            _ => unreachable!(),
        }
    }

    /// Read a byte from an IO device (maps: [0xFF00-0xFF7F])
    fn read_io(&self, addr: u16) -> u8 {
        match addr {
            0xFF00 => self.joypad.read(),
            0xFF01 => self.sb, // TODO: map to serial transfer data
            0xFF02 => self.sc, // TODO: map to serial transfer control

            0xFF04 => self.timer.div,
            0xFF05 => self.timer.tima,
            0xFF06 => self.timer.tma,
            0xFF07 => self.timer.tac,

            0xFF0F => self.if_reg,

            // TODO: move this into sound controller file
            0xFF10 => self.sound.nr10,
            0xFF11 => self.sound.nr11,
            0xFF12 => self.sound.nr12,
            0xFF13 => self.sound.nr13,
            0xFF14 => self.sound.nr14,
            0xFF16 => self.sound.nr21,
            0xFF17 => self.sound.nr22,
            0xFF18 => self.sound.nr23,
            0xFF19 => self.sound.nr24,
            0xFF1A => self.sound.nr30,
            0xFF1B => self.sound.nr31,
            0xFF1C => self.sound.nr32,
            0xFF1D => self.sound.nr33,
            0xFF1E => self.sound.nr34,
            0xFF20 => self.sound.nr41,
            0xFF21 => self.sound.nr42,
            0xFF22 => self.sound.nr43,
            0xFF23 => self.sound.nr44,
            0xFF24 => self.sound.nr50,
            0xFF25 => self.sound.nr51,
            0xFF26 => self.sound.nr52,
            0xFF30 ... 0xFF3F => self.sound.wave_pattern_ram[(addr - 0xFF30) as usize],

            0xFF40 => self.gpu.lcdc,
            0xFF41 => self.gpu.get_stat(),
            0xFF42 => self.gpu.scy,
            0xFF43 => self.gpu.scx,
            0xFF44 => self.gpu.ly,
            0xFF45 => self.gpu.lyc,
            0xFF47 => self.gpu.bgp,
            0xFF48 => self.gpu.obp0,
            0xFF49 => self.gpu.obp1,
            0xFF4A => self.gpu.wy,
            0xFF4B => self.gpu.wx,
            // 0xFF4D => 0, // TODO: Prepare switch speed
            0xFF4F => self.gpu.vram_bank,

            // 0xFF56 => 0, // TODO: Infrared communications port
            0xFF57 => 0, // Undocumented

            // TODO: Map CGB registers
            // 0xFF68 => self.gpu.bcps,
            // 0xFF69 => self.gpu.bcpd,
            // 0xFF6A => self.gpu.ocps,
            // 0xFF6B => self.gpu.ocpd,
            // 0xFF68 => self.gpu.bcps,

            0xFF70 => panic!("Should not be here in gb mode"), // TODO: WRAM Bank in cgb mode
            0xFF72 => 0, // Undocumented
            0xFF73 => 0, // Undocumented
            0xFF74 => 0, // Undocumented
            0xFF75 => 0, // Undocumented
            0xFF76 => 0, // Undocumented - Always 0
            0xFF77 => 0, // Undocumented - Always 0

            _      => 0,
        }
    }

    /// Load a word from memory
    pub fn lw(&self, addr: u16) -> u16 {
        (self.lb(addr) as u16) + ((self.lb(addr + 1) as u16) << 8)
    }

    /// Store a byte in memory
    pub fn sb(&mut self, addr: u16, value: u8) {
        if self.crashed { return; }

        // Map memory writes to their correct bytes. (See: http://problemkaputt.de/pandocs.htm)
        match addr {
            0x0000 ... 0x7FFF | 0xA000 ... 0xBFFF => self.cart.write(addr, value),
            0x8000 ... 0x9FFF => self.gpu.vram_mut()[(addr & 0x1FFF) as usize] = value,
            // NOTE: Check this mapping when adding CGB support
            0xC000 ... 0xDFFF => self.working_ram[(addr & 0x1FFF) as usize] = value,
            0xE000 ... 0xFDFF => self.working_ram[(addr & 0x1FFF) as usize] = value,
            0xFE00 ... 0xFE9F => self.gpu.oam[(addr & 0x00FF) as usize] = value,
            0xFEA0 ... 0xFEFF => self.invalid_memory(addr),
            0xFF00 ... 0xFF7F => self.write_io(addr, value),
            0xFF80 ... 0xFFFE => self.fast_ram[(addr & 0x7F) as usize] = value,
            0xFFFF            => self.ie_reg = value,

            _ => unreachable!(),
        }
    }

    pub fn write_io(&mut self, addr: u16, value: u8) {
        match addr {
            0xFF00 => self.joypad.write(value),
            0xFF01 => self.sb = value,
            0xFF02 => self.sc = value,

            0xFF04 => self.timer.div = value,
            0xFF05 => self.timer.tima = value,
            0xFF06 => self.timer.tma = value,
            0xFF07 => self.timer.tac = value,
            0xFF0F => self.if_reg = value,

            0xFF10 => self.sound.nr10 = value,
            0xFF11 => self.sound.nr11 = value,
            0xFF12 => self.sound.nr12 = value,
            0xFF13 => self.sound.nr13 = value,
            0xFF14 => self.sound.nr14 = value,
            0xFF16 => self.sound.nr21 = value,
            0xFF17 => self.sound.nr22 = value,
            0xFF18 => self.sound.nr23 = value,
            0xFF19 => self.sound.nr24 = value,
            0xFF1A => self.sound.nr30 = value,
            0xFF1B => self.sound.nr31 = value,
            0xFF1C => self.sound.nr32 = value,
            0xFF1D => self.sound.nr33 = value,
            0xFF1E => self.sound.nr34 = value,
            0xFF20 => self.sound.nr41 = value,
            0xFF21 => self.sound.nr42 = value,
            0xFF22 => self.sound.nr43 = value,
            0xFF23 => self.sound.nr44 = value,
            0xFF24 => self.sound.nr50 = value,
            0xFF25 => self.sound.nr51 = value,
            0xFF26 => self.sound.nr52 = value,
            0xFF30 ... 0xFF3F => self.sound.wave_pattern_ram[(addr - 0xFF30) as usize] = value,

            0xFF40 => self.gpu.lcdc = value,
            0xFF41 => self.gpu.set_stat(value),
            0xFF42 => self.gpu.scy = value,
            0xFF43 => self.gpu.scx = value,
            0xFF44 => self.gpu.ly = value,
            0xFF45 => self.gpu.lyc = value,
            0xFF46 => {
                self.gpu.dma = value;
                graphics::oam_dma_transfer(self);
            },
            0xFF47 => self.gpu.bgp = value,
            0xFF48 => self.gpu.obp0 = value,
            0xFF49 => self.gpu.obp1 = value,
            0xFF4A => self.gpu.wy = value,
            0xFF4B => self.gpu.wx = value,

            // 0xFF4D => {}, // TODO: Prepare switch speed

            0xFF4F => self.gpu.vram_bank = value,

            0xFF56 => {}, // TODO: Infrared communications port
            0xFF57 => {}, // Undocumented

            // TODO: Map CGB registers
            // 0xFF68 => self.gpu.bcps,
            // 0xFF69 => self.gpu.bcpd,
            // 0xFF6A => self.gpu.ocps,
            // 0xFF6B => self.gpu.ocpd,
            // 0xFF68 => self.gpu.bcps,

            0xFF70 => { panic!("Should not be here in gb mode") }, // TODO: WRAM Bank
            0xFF72 => {}, // Undocumented
            0xFF73 => {}, // Undocumented
            0xFF74 => {}, // Undocumented
            0xFF75 => {}, // Undocumented
            0xFF76 => {}, // Undocumented - Always 0
            0xFF77 => {}, // Undocumented - Always 0

            _      => {},
        }
    }

    /// Store a word in memory. Low order bits are stored in [addr] and high order bits are stored
    /// in [addr+1].
    pub fn sw(&mut self, addr: u16, value: u16) {
        self.sb(addr, value as u8);
        self.sb(addr + 1, (value >> 8) as u8);
    }
}
