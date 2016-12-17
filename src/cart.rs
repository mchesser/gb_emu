//! Emulates the various functionality of the cartridges

use cart::MemoryBankController::{NoMbc, Mbc1, Mbc2, Mbc3};
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MemoryBankController {
    NoMbc, // No memory bank controller (32Kbyte ROM only)
    Mbc1,  // Max 2MBbyte ROM and/or 32KByte RAM
    Mbc2,  // Max 256Kbyte ROM and 512x4 bits RAM
    Mbc3,  // Max 2MByte ROM and/or 32KByte RAM and Timer
    // FIXME(minor): add support for Huc1 controller
    // Huc1,  // MBC with Infrared Controller
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum BankingMode {
    Rom,
    Ram,
}

pub trait SaveFile: Send {
    fn load(&mut self, data: &mut [u8]);
    fn save(&mut self, data: &[u8]);
}

pub struct RealTimeClock {
    state: u8,
    carry: bool,
    halt: bool,

    seconds: u8,
    minutes: u8,
    hours: u8,
    days: u16,
}

impl RealTimeClock {
    fn new() -> RealTimeClock {
        RealTimeClock {
            state: 0,
            carry: false,
            halt: false,

            seconds: 0,
            minutes: 0,
            hours: 0,
            days: 0,
        }
    }

    fn read(&self) -> u8 {
        match self.state & 0x7 {
            0 => self.seconds,
            1 => self.minutes,
            2 => self.hours,
            3 => self.days as u8,
            4 => {
                ((self.days >> 8) as u8) |
                if self.halt { 0x40 } else { 0 } |
                if self.carry { 0x80 } else { 0 }
            },

            _ => 0x00,
        }
    }

    fn write(&mut self, value: u8) {
        match self.state & 0x7 {
            0 => self.seconds = value % 60,
            1 => self.minutes = value % 60,
            2 => self.hours = value % 24,
            3 => self.days = (self.days & 0x100) | (value as u16),
            4 => {
                self.days = (((value & 1) as u16) << 8) | (self.days & 0xFF);
                self.halt = (value & 0x40) != 0;
                self.carry = (value & 0x80) != 0;
            },
            _ => {},
        }
    }

    fn is_enabled(&self) -> bool {
        self.state & 0x8 != 0
    }

    pub fn tick(&mut self) {
        if self.halt {
            return;
        }

        self.seconds += 1;
        if self.seconds >= 60 {
            self.seconds = 0;

            self.minutes += 1;
            if self.minutes >= 60 {
                self.minutes = 0;

                self.hours += 1;
                if self.hours >= 24 {
                    self.hours = 0;

                    self.days += 1;
                    if self.days >= 356 {
                        self.days = 0;

                        self.carry = true;
                    }
                }
            }
        }
    }

}

pub struct Cartridge {
    mbc: MemoryBankController,
    selected_banking_mode: BankingMode,

    /// Rom banks (maximum of 128 banks = 2MB, mapped to: 0x0000-0x7FFFF)
    pub rom: [[u8; 0x4000]; 128],
    /// The currently mapped rom bank (bank 0 is always mapped)
    pub rom_bank: usize,

    /// External ram banks (maximum of 4 banks = 32KB, mapped to: 0xA000-0xBFFF)
    pub ram: [u8; 0x2000 * 4],
    /// The the currently mapped external ram bank (bank 0 is always mapped)
    pub ram_bank: usize,
    // Indicates if the ram has been enabled
    pub ram_enabled: bool,

    /// The save file attached to the cartridge
    pub save_file: Option<Box<SaveFile>>,

    /// The realtime clock attached to the cartridge
    pub rtc: RealTimeClock,
}

impl Cartridge {
    /// Create a new cartridge with empty data.
    pub fn new() -> Cartridge {
        Cartridge {
            mbc: NoMbc,
            selected_banking_mode: BankingMode::Rom,

            rom: [[0; 0x4000]; 128],
            rom_bank: 1,

            ram: [0; 0x2000 * 4],
            ram_bank: 0,
            ram_enabled: false,

            save_file: None,
            rtc: RealTimeClock::new(),
        }
    }

    /// Convert the specified address into an index into the ram array, adjusting the index
    /// according to the currently enabled ram bank
    pub fn ram_index(&self, addr: u16) -> usize {
        0x2000 * self.ram_bank + (addr & 0x1FFF) as usize
    }

    /// Load cartridge data from a data buffer, and optimally load a save file.
    pub fn load(&mut self, data: &[u8], save_file: Option<Box<SaveFile>>) {
        for (i, chunk) in data.chunks(0x4000).enumerate() {
            self.rom[i].copy_from_slice(chunk);
        }

        // Get the type of memory bank controller from the cart
        // TODO: Handle more of the settings
        let mbc_type = self.rom[0][0x0147];
        // 00h  ROM ONLY                 13h  MBC3+RAM+BATTERY
        // 01h  MBC1                     15h  MBC4
        // 02h  MBC1+RAM                 16h  MBC4+RAM
        // 03h  MBC1+RAM+BATTERY         17h  MBC4+RAM+BATTERY
        // 05h  MBC2                     19h  MBC5
        // 06h  MBC2+BATTERY             1Ah  MBC5+RAM
        // 08h  ROM+RAM                  1Bh  MBC5+RAM+BATTERY
        // 09h  ROM+RAM+BATTERY          1Ch  MBC5+RUMBLE
        // 0Bh  MMM01                    1Dh  MBC5+RUMBLE+RAM
        // 0Ch  MMM01+RAM                1Eh  MBC5+RUMBLE+RAM+BATTERY
        // 0Dh  MMM01+RAM+BATTERY        FCh  POCKET CAMERA
        // 0Fh  MBC3+TIMER+BATTERY       FDh  BANDAI TAMA5
        // 10h  MBC3+TIMER+RAM+BATTERY   FEh  HuC3
        // 11h  MBC3                     FFh  HuC1+RAM+BATTERY
        // 12h  MBC3+RAM
        match mbc_type {
            0x00 | 0x08 | 0x09 => { self.mbc = NoMbc; },
            0x01 | 0x02 | 0x03 => { self.mbc = Mbc1; },
            0x05 | 0x06 => { self.mbc = Mbc2; },
            0x11 | 0x12 | 0x0F | 0x10 | 0x13 => { self.mbc = Mbc3; }

            x => { panic!("Unsupported cartridge: {:X}", x); }
        }

        // Load the save file if there is one
        self.save_file = save_file;
        if let Some(ref mut save) = self.save_file {
            save.load(&mut self.ram);
        }
    }

    /// Read data from a memory mapped address
    pub fn read(&self, addr: u16) -> u8 {
        // FIXME(minor): should probably handle invalid memory accesses depending on the memory bank
        // controller more carefully.
        match addr {
            0x0000 ... 0x3FFF => self.rom[0][(addr & 0x3FFF) as usize],
            0x4000 ... 0x7FFF => self.rom[self.rom_bank][(addr & 0x3FFF) as usize],
            0xA000 ... 0xBFFF => {
                if self.rtc.is_enabled() { self.rtc.read() }
                else { self.ram[self.ram_index(addr)] }
            },

            _ => unreachable!(),
        }
    }

    /// Write data to a memory mapped address
    pub fn write(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000 ... 0x1FFF => match self.mbc {
                NoMbc => {},
                Mbc1 => self.ram_enabled = value & 0xF == 0x0A,
                Mbc2 => {
                    if addr & 0x0100 == 0 {
                        self.ram_enabled = !self.ram_enabled;
                    }
                },
                Mbc3 => {
                    self.ram_enabled = value & 0xF == 0x0A;
                    // CHECKME: Not sure if this also enables the rtc
                    // self.rtc.enabled = value == 0x0A;
                },
            },

            // Set ROM bank number
            0x2000 ... 0x3FFF => match self.mbc {
                NoMbc => {},
                Mbc1 => {
                    let high_bits = self.rom_bank as u8 & 0x60;
                    self.rom_bank =
                        if (value & 0x1F) == 0 { (high_bits | 1) as usize }
                        else { (high_bits | (value & 0x1F)) as usize };
                },
                Mbc2 => self.rom_bank = (value & 0x0F) as usize,
                Mbc3 => {
                    self.rom_bank =
                        if value == 0x00 { 1 }
                        else { (value & 0x7F) as usize };
                },
            },

            // Set RAM bank number, upper bits of ROM bank number or real-time-clock register select
            0x4000 ... 0x5FFF => match self.mbc {
                NoMbc | Mbc2 => {}
                Mbc1 => {
                    let bits = value & 0x3;
                    if self.selected_banking_mode == BankingMode::Rom {
                        self.rom_bank = (bits << 5 | (self.rom_bank as u8 & 0x1F)) as usize;
                    }
                    else {
                        self.ram_bank = bits as usize;
                    }
                },
                Mbc3 => {
                    self.rtc.state = value & 0xF;
                    self.ram_bank = (value & 0x3) as usize;
                },
            },

            // Set ROM/RAM mode select or latch clock data
            0x6000 ... 0x7FFF => match self.mbc {
                NoMbc | Mbc2 => {},
                Mbc1 => {
                    self.selected_banking_mode =
                        if value & 0x1 == 0 { BankingMode::Rom }
                        else { BankingMode::Ram };
                },
                Mbc3 => {
                    // FIXME(major): Handle real-time-clock latch clock data
                },
            },

            // Write to external ram
            0xA000 ... 0xBFFF => self.ram[self.ram_index(addr)] = value,

            _ => unreachable!(),
        }
    }
}

impl Drop for Cartridge {
    fn drop(&mut self) {
        if let Some(ref mut save_file) = self.save_file {
            // TODO: Save the rtc
            save_file.save(&self.ram);
        }
    }
}
