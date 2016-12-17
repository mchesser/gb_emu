//! Manages the gameboy sound controller
//! This is mostly incomplete, at the moment all this does is manage the sound registers.

pub struct SoundController {
    /// Channel 1 Sweep register
    pub nr10: u8,
    /// Channel 1 Sound length/Wave pattern duty
    pub nr11: u8,
    /// Channel 1 Volume Envelope (R/W)
    pub nr12: u8,
    /// Channel 1 Frequency lo
    pub nr13: u8,
    /// Channel 1 Frequency hi
    pub nr14: u8,

    /// Channel 2 Sound length/Wave pattern duty
    pub nr21: u8,
    /// Channel 2 Volume Envelope (R/W)
    pub nr22: u8,
    /// Channel 2 Frequency lo
    pub nr23: u8,
    /// Channel 2 Frequency hi
    pub nr24: u8,

    pub nr30: u8,
    pub nr31: u8,
    pub nr32: u8,
    pub nr33: u8,
    pub nr34: u8,
    pub wave_pattern_ram: [u8; 32],

    pub nr41: u8,
    pub nr42: u8,
    pub nr43: u8,
    pub nr44: u8,

    pub nr50: u8,
    pub nr51: u8,
    pub nr52: u8,
}

impl SoundController {
    pub fn new() -> SoundController {
        SoundController {
            nr10: 0,
            nr11: 0,
            nr12: 0,
            nr13: 0,
            nr14: 0,

            nr21: 0,
            nr22: 0,
            nr23: 0,
            nr24: 0,

            nr30: 0,
            nr31: 0,
            nr32: 0,
            nr33: 0,
            nr34: 0,
            wave_pattern_ram: [0; 32],

            nr41: 0,
            nr42: 0,
            nr43: 0,
            nr44: 0,

            nr50: 0,
            nr51: 0,
            nr52: 0,
        }
    }
}
