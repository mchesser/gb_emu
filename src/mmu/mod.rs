#![allow(dead_code)]

use gb::{Mode, GameBoy, SuperGameBoy, GameBoyColor};
use graphics::Gpu;

#[deriving(PartialEq)]
enum MemoryBankController {
    NoMbc, // No memory bank controller (32Kbyte ROM only)
    Mbc1,  // Max 2MBbyte ROM and/or 32KByte RAM
    Mbc2,  // Max 256Kbyte ROM and 512x4 bits RAM
    Mbc3,  // Max 2MByte ROM and/or 32KByte RAM and Timer

    // FIXME(minor): add support for Huc1 controller
    // Huc1,  // MBC with Infrared Controller
}

#[deriving(PartialEq)]
enum BankingMode {
    RomBankingMode,
    RamBankingMode,
}

/// The GB memory mapper.
///
/// Memory layout:
///
/// * [0x0000-0x3FFF] 16KB - Cartridge ROM, bank 00
/// * [0x4000-0x7FFF] 16KB - Cartridge ROM, bank 01..NN (switchable)
/// * [0x8000-0x9FFF] 8KB - Video RAM (VRAM) (switchable)
/// * [0xA000-0xBFFF] 8KB - Cartridge RAM (switchable)
/// * [0xC000-0xDFFF] 8KB - Working RAM
/// * [0xE000-0xFDFF] Working RAM shadow (limited exact copy of working ram)
/// * [0xFE00-0xFF9F] Sprite Attribute Table (OAM)
/// * [0xFEA0-0xFEFF] Unusable
/// * [0xFF00-0xFF7F] I/O Ports
/// * [0xFF80-0xFFFE] Zero-page RAM (high-speed RAM)
/// * [0xFFFF       ] Interrupt enable register
pub struct MemMap {
    /// Interrupt enabled register
    ie_reg: u8,
    /// Interrupt flags register
    if_reg: u8,

    /// The cart memory bank controller
    mbc: MemoryBankController,
    /// The current banking mode of the system
    banking_mode: BankingMode,

    /// Rom banks (maximum of 128 banks = 2MB)
    rom: [[u8, ..0x4000], ..128],
    /// The currently mapped rom bank (bank 0 is always mapped)
    rom_bank: uint,

    /// External ram banks (maximum of 4 banks = 32KB)
    external_ram: [[u8, ..0x2000], ..4],
    /// The the currently mapped external ram bank (bank 0 is always mapped)
    ram_bank: uint,
    // Indicates if the ram has been enabled
    ram_enabled: bool,

    /// Working ram
    working_ram: [u8, ..0x2000],

    /// Zero-page high speed ram
    fast_ram: [u8, ..0x7F],

    /// GB graphics processor
    gpu: Gpu,
}

impl MemMap {
    pub fn new() -> MemMap {
        MemMap {
            ie_reg: 0,
            if_reg: 0,

            mbc: NoMbc,
            banking_mode: RomBankingMode,

            rom: [[0, ..0x4000], ..128],
            rom_bank: 1,

            external_ram: [[0, ..0x2000], ..4],
            ram_bank: 1,
            ram_enabled: false,

            working_ram: [0, ..0x2000],
            fast_ram: [0, ..0x7F],

            gpu: Gpu::new(),
        }
    }

    /// Initialise the memory with the expected data
    pub fn start_up(&mut self, mode: Mode) {
        // GB start up sequence, see: http://problemkaputt.de/pandocs.htm#powerupsequence
        // (I have no idea what any of these values actually mean)
        self.sb(0xFF05, 0x00); // TIMA
        self.sb(0xFF06, 0x00); // TMA
        self.sb(0xFF07, 0x00); // TAC
        self.sb(0xFF10, 0x80); // NR10
        self.sb(0xFF11, 0xBF); // NR11
        self.sb(0xFF12, 0xF3); // NR12
        self.sb(0xFF14, 0xBF); // NR14
        self.sb(0xFF16, 0x3F); // NR21
        self.sb(0xFF17, 0x00); // NR22
        self.sb(0xFF19, 0xBF); // NR24
        self.sb(0xFF1A, 0x7F); // NR30
        self.sb(0xFF1B, 0xFF); // NR31
        self.sb(0xFF1C, 0x9F); // NR32
        self.sb(0xFF1E, 0xBF); // NR33
        self.sb(0xFF20, 0xFF); // NR41
        self.sb(0xFF21, 0x00); // NR42
        self.sb(0xFF22, 0x00); // NR43
        self.sb(0xFF23, 0xBF); // NR30
        self.sb(0xFF24, 0x77); // NR50
        self.sb(0xFF25, 0xF3); // NR51
        self.sb(0xFF26, 0xF1); // NR52
        self.sb(0xFF40, 0x91); // LCDC
        self.sb(0xFF42, 0x00); // SCY
        self.sb(0xFF43, 0x00); // SCX
        self.sb(0xFF45, 0x00); // LYC
        self.sb(0xFF47, 0xFC); // BGP
        self.sb(0xFF48, 0xFF); // OBP0
        self.sb(0xFF49, 0xFF); // OBP1
        self.sb(0xFF4A, 0x00); // WY
        self.sb(0xFF4B, 0x00); // WX
        self.sb(0xFFFF, 0x00); // IE

        // Handle mode specific settings
        match mode {
            SuperGameBoy => {
                self.sb(0xFF26, 0xF0); // NR52
            },

            GameBoyColor => {
                self.sb(0xFF68, 0xC0);
                self.sb(0xFF6A, 0xC0);
            },

            GameBoy => {},
        }
    }

    /// Read a byte from an IO device
    fn read_io(&mut self, addr: u16) -> u8 {
        unimplemented!()
    }

    /// Write a byte to an IO device
    fn write_io(&mut self, addr: u16, value: u8) {
        unimplemented!()
    }

    /// Handle invalid memory accesses
    fn invalid_memory(&mut self, addr: u16) -> ! {
        panic!("Invalid memory access to address: {}", addr);
    }

    /// Load a byte from memory
    pub fn lb(&mut self, addr: u16) -> u8 {
        // Map memory reads to their correct bytes. (See: http://problemkaputt.de/pandocs.htm)
        // FIXME(minor): should probably handle invalid memory accesses depending on the memory bank
        // controller more carefully.
        match addr {
            0x0000 ... 0x3FFF => self.rom[0][(addr & 0x1FFF) as uint],
            0x4000 ... 0x7FFF => self.rom[self.rom_bank][(addr & 0x3FFF) as uint],
            0x8000 ... 0x9FFF => self.gpu.vram()[(addr & 0x1FFF) as uint],
            0xA000 ... 0xBFFF => self.external_ram[self.ram_bank][(addr & 0x1FFF) as uint],
            0xC000 ... 0xDFFF => self.working_ram[(addr & 0x0FFF) as uint],
            0xE000 ... 0xFDFF => self.working_ram[(addr & 0x0FFF) as uint],
            0xFE00 ... 0xFE9F => *self.gpu.map_oam(addr),
            0xFEA0 ... 0xFEFF => self.invalid_memory(addr),
            0xFF00 ... 0xFF7F => self.read_io(addr),
            0xFF80 ... 0xFFFE => self.fast_ram[(addr & 0x7F) as uint],
            0xFFFF            => self.ie_reg,

            _ => unreachable!(),
        }
    }

    /// Load a word from memory
    pub fn lw(&mut self, addr: u16) -> u16 {
        self.lb(addr) as u16 + (self.lb(addr + 1) as u16 << 8)
    }

    /// Store a byte in memory
    pub fn sb(&mut self, addr: u16, value: u8) {
        // Map memory writes to their correct bytes. (See: http://problemkaputt.de/pandocs.htm)
        // Writes may have different effects depending on the active memory bank controller
        match addr {
            // Set RAM and RTC enabled/disabled
            0x0000 ... 0x1FFF => match self.mbc {
                NoMbc => {},
                Mbc1 => self.ram_enabled = value == 0x0A,
                Mbc2 => {
                    if addr & 0x0100 == 0 {
                        self.ram_enabled = !self.ram_enabled;
                    }
                },
                Mbc3 => {
                    self.ram_enabled = value == 0x0A;

                    // FIXME(major): uncomment the following line when the real-time-clock is added.
                    // self.rtc.enabled = value == 0x0A;
                },
            },

            // Set ROM bank number
            0x2000 ... 0x3FFF => match self.mbc {
                NoMbc => {},
                Mbc1 => {
                    let high_bits = self.rom_bank as u8 & 0x60;
                    self.rom_bank =
                        if (value & 0x1F) == 0 { (high_bits | 1) as uint }
                        else { (high_bits | (value & 0x1F)) as uint };
                },
                Mbc2 => self.rom_bank = (value & 0x0F) as uint,
                Mbc3 => {
                    self.rom_bank =
                        if value == 0x00 { 1 }
                        else { (value & 0x7F) as uint };
                },
            },

            // Set RAM bank number, upper bits of ROM bank number or real-time-clock register select
            0x4000 ... 0x5FFF => match self.mbc {
                NoMbc | Mbc2 => {}
                Mbc1 => {
                    let bits = value & 0x3;
                    if self.banking_mode == RomBankingMode {
                        self.rom_bank = (bits << 5 | (self.rom_bank as u8 & 0x1F)) as uint;
                    }
                    else {
                        self.ram_bank = bits as uint;
                    }
                },
                Mbc3 => {
                    // FIXME(major): handle real time clock here
                    self.ram_bank = (value & 0x3) as uint;
                },
            },

            // Set ROM/RAM mode select or latch clock data
            0x6000 ... 0x7FFF => match self.mbc {
                NoMbc | Mbc2 => {},
                Mbc1 => {
                    self.banking_mode =
                        if value & 0x1 == 0 { RomBankingMode }
                        else { RamBankingMode };
                },
                Mbc3 => {
                    // FIXME(major): Handle real-time-clock latch clock data
                },
            },

            0x8000 ... 0x9FFF => self.gpu.vram()[(addr & 0x1FFF) as uint] = value,
            // FIXME(minor): We don't actually check if the ram has been enabled berfore writing
            // to it
            0xA000 ... 0xBFFF => self.external_ram[self.ram_bank][(addr & 0x1FFF) as uint] = value,
            0xC000 ... 0xDFFF => self.working_ram[(addr & 0x0FFF) as uint] = value,
            0xE000 ... 0xFDFF => self.working_ram[(addr & 0x0FFF) as uint] = value,

            0xFE00 ... 0xFE9F => *self.gpu.map_oam(addr) = value,
            0xFEA0 ... 0xFEFF => self.invalid_memory(addr),
            0xFF00 ... 0xFF7F => self.write_io(addr, value),
            0xFF80 ... 0xFFFE => self.fast_ram[(addr & 0x7F) as uint] = value,
            0xFFFF            => self.ie_reg = value,

            _ => unreachable!(),
        }
    }

    /// Store a word in memory. Low order bits are stored in [addr] and high order bits are stored
    /// in [addr+1].
    pub fn sw(&mut self, addr: u16, value: u16) {
        self.sb(addr, value as u8);
        self.sb(addr, (value >> 8) as u8);
    }

    /// Write an entire array into memory starting at the address specified. If the amount of data
    /// exceeds the avalible memory then only the start of the data will be wrtten.
    ///
    /// Returns the number of bytes written.
    pub fn write_array(&mut self, start_addr: u16, data: &[u8]) -> u16 {
        let mut addr = start_addr;
        for &byte in data.iter() {
            // Check if this is the last address
            if addr == 0xFFFF {
                break;
            }

            self.sb(addr, byte);
            addr += 1;
        }

        addr - start_addr
    }

    /// Reads data from memory into a provided buffer. Keeps reading until either the buffer is full
    /// or we have reached the end of the avaliable memory.
    ///
    /// Returns the number of bytes read.
    pub fn read_array(&mut self, start_addr: u16, buffer: &mut [u8]) -> u16 {
        let mut addr = start_addr;
        for byte in buffer.iter_mut() {
            // Check if this is the last address
            if addr == 0xFFFF {
                break;
            }

            *byte = self.lb(addr);
            addr += 1;
        }

        addr - start_addr
    }
}
