pub enum ReadMode {
    Button,
    Direction,
}

pub enum State {
    Pressed = 0,
    Released = 1,
}

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
        let val = match self.read_mode {
            ReadMode::Button => {
                self.start as u8 << 3 | self.select as u8 << 2 | self.b as u8 << 1 | self.a as u8
            },
            ReadMode::Direction => {
                self.down as u8 << 3 | self.up as u8 << 2 | self.left as u8 << 1 | self.right as u8
            },
        };

        // println!("reading joypad, value is: {:4b}", val);
        val
    }

    /// Write a value to the joypad register
    pub fn write(&mut self, value: u8) {
        match (value & 0x10 == 0, value & 0x8 == 0) {
            (true, true) => {}, // Not sure how we should handle this case
            (false, true) => self.read_mode = ReadMode::Button,
            (true, false) => self.read_mode = ReadMode::Direction,
            (false, false) => {},
        }
    }
}
