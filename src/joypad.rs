//! Manages the gameboy joypad.

#[deriving(Copy)]
pub enum ReadMode {
    Button,
    Direction,
}

#[deriving(Copy, PartialEq)]
pub enum State {
    Pressed = 0,
    Released = 1,
}

#[allow(missing_copy_implementations)]
pub struct Joypad {
    pub start: State,
    pub select: State,
    pub b: State,
    pub a: State,

    pub down: State,
    pub up: State,
    pub left: State,
    pub right: State,

    read_mode: ReadMode,
}

impl Joypad {
    pub fn new() -> Joypad {
        Joypad {
            select: State::Released,
            start: State::Released,
            b: State::Released,
            a: State::Released,

            down: State::Released,
            up: State::Released,
            left: State::Released,
            right: State::Released,

            read_mode: ReadMode::Button,
        }
    }

    /// Read the value of the joypad register
    pub fn read(&self) -> u8 {
        match self.read_mode {
            ReadMode::Button => {
                self.start as u8 << 3 | self.select as u8 << 2 | self.b as u8 << 1 | self.a as u8
            },
            ReadMode::Direction => {
                self.down as u8 << 3 | self.up as u8 << 2 | self.left as u8 << 1 | self.right as u8
            },
        }
    }

    /// Write a value to the joypad register
    pub fn write(&mut self, value: u8) {
        match !value & 0x30 {
            0x20 => self.read_mode = ReadMode::Button,
            0x10 => self.read_mode = ReadMode::Direction,
            0x00 => {},
            _ => {},
        }
    }
}
