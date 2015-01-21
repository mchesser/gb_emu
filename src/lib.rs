#![feature(slicing_syntax, unboxed_closures)]
#![allow(unstable)] // This generates a lot of unnecessary warnings at the moment

pub mod emulator;
pub mod cpu;
pub mod mmu;
pub mod cart;
pub mod graphics;
pub mod timer;
pub mod sound;
pub mod joypad;
pub mod debug;
