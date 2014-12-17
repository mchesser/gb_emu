#![feature(macro_rules, unboxed_closures)]
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
mod sound;
mod joypad;
mod debug;

fn main() {
    let mut logger = debug::Logger::new();

    // Create a new emulator (note the emulator object is quite large, so it is not allocated on
    // on the stack).
    let mut emulator = box Emulator::new(|cpu, mem| debug::stack_traces(&mut logger, cpu, mem));

    // Load a cart
    let cart = File::open(&Path::new("testGame5.gb")).read_to_end().unwrap();
    emulator.load_cart(cart.as_slice());
    emulator.start();

    client::run(emulator);
}
