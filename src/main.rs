#![feature(macro_rules)]
extern crate sdl2;
extern crate time;

use std::io::File;
use gb::Emulator;

mod cpu;
mod mmu;
mod gb;
mod graphics;
mod timer;
mod client;
mod client_timer;
mod symbols;
mod sound;
mod joypad;

fn main() {
    // Create a new emulator (note the emulator object is quite large, so it is not allocated on
    // on the stack).
    let mut emulator = box Emulator::new();

    // Load a cart
    let cart = File::open(&Path::new("testGame5.gb")).read_to_end().ok().unwrap();
    emulator.load_cart(cart.as_slice());
    emulator.start();

    client::run(emulator);
}
