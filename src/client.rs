#![allow(dead_code)]
use std::slice::bytes::copy_memory;

use sdl2;
use sdl2::event::Event;
use sdl2::event::poll_event;
use sdl2::keycode::KeyCode;
use sdl2::video::{Window, OPENGL};
use sdl2::video::WindowPos::PosCentered;
use sdl2::surface::Surface;

use cpu::Cpu;
use mmu::Memory;
use gb;
use graphics;
use joypad;

use client_timer::Timer;

const SCALE: int = 1;
const WIDTH: int = graphics::WIDTH as int * SCALE;
const HEIGHT: int = graphics::HEIGHT as int * SCALE;

const SRC_WIDTH: uint = graphics::WIDTH as uint;
const SRC_HEIGHT: uint = graphics::HEIGHT as uint;

pub fn run<F>(mut emulator: Box<gb::Emulator<F>>) where F: FnMut(&mut Cpu, &mut Memory) {
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

    let mut fast_mode = false;

    let mut timer = Timer::new();
    'main: loop {
        'event: loop {
            match poll_event() {
                Event::Quit(_) => break 'main,

                Event::KeyDown(_, _, code, _, _, _) => {
                    handle_joypad_event(&mut emulator.mem.joypad, code, joypad::State::Pressed);
                    if code == KeyCode::Space { fast_mode = true; }
                },

                Event::KeyUp(_, _, code, _, _, _) => {
                    handle_joypad_event(&mut emulator.mem.joypad, code, joypad::State::Released);
                    if code == KeyCode::Space { fast_mode = false; }
                },

                Event::None => break,
                _ => continue,
            }
        }


        if fast_mode || timer.elapsed_seconds() >= 1.0 / 60.0 {
            timer.reset();
            emulator.frame();
        }

        if emulator.mem.gpu.ready_flag {
            emulator.mem.gpu.ready_flag = false;
            render_screen(&mut surface, emulator.display());
            window.update_surface();
        }
    }
}

fn handle_joypad_event(joypad: &mut joypad::Joypad, keycode: KeyCode, state: joypad::State) {
    // TODO: Add custom keybindings
    match keycode {
        KeyCode::Up => joypad.up = state,
        KeyCode::Down => joypad.down = state,
        KeyCode::Left => joypad.left = state,
        KeyCode::Right => joypad.right = state,

        KeyCode::Z => joypad.a = state,
        KeyCode::X => joypad.b = state,
        KeyCode::KpEnter => joypad.start = state,
        KeyCode::RShift => joypad.select = state,

        _ => {},
    }
}

fn render_screen(surface: &mut Surface, display: &[u8]) {
    surface.with_lock(|pixels| copy_memory(pixels, display));
}
