#![allow(dead_code)]
use std::slice::bytes::copy_memory;

use sdl2;
use sdl2::event;
use sdl2::video::{Window, PosCentered, OPENGL};
use sdl2::event::poll_event;
use sdl2::surface::Surface;

use gb;
use graphics;
// use client_timer::Timer;

const SCALE: int = 1;
const WIDTH: int = graphics::WIDTH as int * SCALE;
const HEIGHT: int = graphics::HEIGHT as int * SCALE;

const SRC_WIDTH: uint = graphics::WIDTH as uint;
const SRC_HEIGHT: uint = graphics::HEIGHT as uint;

pub fn run(mut emulator: Box<gb::Emulator>) {
    sdl2::init(sdl2::INIT_EVERYTHING);

    let window = match Window::new("GameBoy Emulator", PosCentered, PosCentered,
        WIDTH, HEIGHT, OPENGL)
    {
        Ok(window) => window,
        Err(err) => panic!(format!("failed to create window: {}", err))
    };

    let mut surface = match window.get_surface() {
        Ok(surface) => surface,
        Err(err) => panic!(format!("failed to get window surface: {}", err))
    };

    // FIXME(major): use timers to run at the correct frame rate
    // let mut cpu_timer = Timer::new();
    // let mut timer = Timer::new();

    'main: loop {
        'event: loop {
            match event::poll_event() {
                event::Quit(_) => break 'main,

                event::KeyDown(_, _, code, _, _) => {
                    println!("You pressed {}", code);
                }

                event::KeyUp(_, _, code, _, _) => {
                    println!("You released {}", code);
                }

                event::None => break,
                _ => continue,
            }
        }

        emulator.frame();
        if emulator.mem.gpu.ready_flag {
            emulator.mem.gpu.ready_flag = false;
            render_screen(&mut surface, emulator.display());
            window.update_surface();
        }
    }
}

fn render_screen(surface: &mut Surface, display: &[u8]) {
    surface.with_lock(|pixels| copy_memory(pixels, display));
}
