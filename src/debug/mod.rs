//! This module implements some very basic debugging tools useful for finding bugs in games and the
//! emulator itself.
//! Currently is main purpose is to print subroutine calls with some structure.

use cpu::{Cpu, State};
use mmu::Memory;
use debug::symbols::{SymbolTable, build_symbol_table};
use debug::disasm::disasm;

pub mod symbols;
pub mod disasm;

const HIDDEN_FUNCTIONS: &'static [(u8, u16)] = &[
];


pub struct Logger {
    symbols: SymbolTable,
    call_stack: Vec<u16>,
    hide_output: bool,
    reveal_at: uint,
    show_individual: bool,
}

impl Logger {
    pub fn new(symbols_file: &str) -> Logger {
        Logger {
            symbols: build_symbol_table(symbols_file),
            call_stack: Vec::new(),
            hide_output: false,
            reveal_at: 0,
            show_individual: false,
        }
    }
}

pub fn stack_traces(logger: &mut Logger, cpu: &Cpu, mem: &Memory) {
    if cpu.crashed { return }

    if logger.hide_output && logger.call_stack.len() <= logger.reveal_at {
        logger.hide_output = false;
        print_text(logger, "...");
    }

    check_interrupts(logger, cpu, mem);
    if cpu.state == State::Running {
        let addr = cpu.pc;
        if logger.show_individual && !logger.hide_output {
            print_tabbing(logger);
            println!("{:04X}:\t{}", addr, disasm(addr, mem));
        }
        check_branch_instructions(logger, cpu, mem);
    }
}

/// Check if we have started handling an interrupt by seeing if the current address is an interrupt
/// handler.
fn check_interrupts(logger: &mut Logger, cpu: &Cpu, mem: &Memory) {
    if 0x0040 <= cpu.pc && cpu.pc <= 0x0060 {
        if !logger.hide_output {
            print_tabbing(logger);
            match cpu.pc {
                0x0040 => println!("----| VBLANK |----"),
                0x0048 => println!("----| STAT   |----"),
                0x0050 => println!("----| TIMER  |----"),
                0x0058 => println!("----| SERIAL |----"),
                0x0060 => println!("----| JOYPAD |----"),
                _ => return,
            }
        }

        logger.call_stack.push(mem.lw(cpu.sp));
        hide_check(logger, (0, cpu.pc));
    }
}

fn check_branch_instructions(logger: &mut Logger, cpu: &Cpu, mem: &Memory) {
    match branch_decoder(cpu.pc, mem, cpu.f) {
        Branch::AbsoluteJump(addr) => {
            let bank_num = if addr < 0x4000 { 0 } else { mem.rom_bank as u8 };
            print_jump(logger, (bank_num, addr));
        }

        Branch::Call(addr) => {
            let bank_num = if addr < 0x4000 { 0 } else { mem.rom_bank as u8 };
            print_call(logger, (bank_num, addr));
            logger.call_stack.push(cpu.pc + 3);
            hide_check(logger, (bank_num, addr));
        },

        Branch::Return => {
            let ret_location = mem.lw(cpu.sp);
            if let Some(i) = logger.call_stack.iter().rev().position(|&addr| addr == ret_location) {
                for _ in range(0, i + 1) {
                    logger.call_stack.pop();
                }
                print_text(logger, "RETURN");
            }
            else {
                print_text(logger, "RETURN from manually created stackframe");
            }
        },

        Branch::ReturnFromException => {
            if logger.call_stack.last() == Some(&mem.lw(cpu.sp)) {
                logger.call_stack.pop();
                print_text(logger, "----| RETURN |----");
            }
        },

        _ => {},
    }
}

fn print_jump(logger: &Logger, key: (u8, u16)) {
    if logger.hide_output { return }

    print_tabbing(logger);
    print!("->");
    print_symbol(logger, key);
    println!("");
}

fn print_call(logger: &Logger, key: (u8, u16)) {
    if logger.hide_output { return }

    print_tabbing(logger);
    print!("CALL: ");
    print_symbol(logger, key);
    println!("");
}

fn print_text(logger: &Logger, text: &str) {
    if logger.hide_output { return }

    print_tabbing(logger);
    println!("{}", text);
}

fn print_tabbing(logger: &Logger) {
    for _ in range(0, logger.call_stack.len()) {
        print!("    ")
    }
}

fn print_symbol(logger: &Logger, key: (u8, u16)) {
    match logger.symbols.get(&key) {
        Some(name) => print!("({:02X}:{:04X}){}", key.0, key.1, name),
        None => print!("{:02X}:{:04X}", key.0, key.1),
    }
}

fn hide_check(logger: &mut Logger, key: (u8, u16)) {
    if logger.hide_output { return }

    if HIDDEN_FUNCTIONS.contains(&key) {
        logger.hide_output = true;
        logger.reveal_at = logger.call_stack.len() - 1;
    }
}


enum Branch {
    RelativeJump(i8),
    AbsoluteJump(u16),
    Call(u16),
    Return,
    ReturnFromException,
    None,
}

fn branch_decoder(mut addr: u16, mem: &Memory, flags: u8) -> Branch {
    macro_rules! get_n { () => ({ addr += 1; mem.lb(addr - 1) }) }
    macro_rules! get_ni { () => (get_n!() as i8) }
    macro_rules! get_nn { () => (get_n!() as u16 + (get_n!() as u16 << 8)) }

    let z = flags & 0b1000_0000 != 0;
    let c = flags & 0b0001_0000 != 0;

    let op = mem.lb(addr); addr += 1;
    match op {
        0x18 => Branch::RelativeJump(get_ni!()),
        0x20 => if !z { Branch::RelativeJump(get_ni!()) } else { Branch::None },
        0x28 => if z { Branch::RelativeJump(get_ni!()) } else { Branch::None },
        0x30 => if !c { Branch::RelativeJump(get_ni!()) } else { Branch::None },
        0x38 => if c { Branch::RelativeJump(get_ni!()) } else { Branch::None },

        0xC3 => Branch::AbsoluteJump(get_nn!()),
        0xC2 => if !z { Branch::AbsoluteJump(get_nn!()) } else { Branch::None },
        0xCA => if z { Branch::AbsoluteJump(get_nn!()) } else { Branch::None },
        0xD2 => if !c { Branch::AbsoluteJump(get_nn!()) } else { Branch::None },
        0xDA => if c { Branch::AbsoluteJump(get_nn!()) } else { Branch::None },

        0xCD => Branch::Call(get_nn!()),
        0xC4 => if !z { Branch::Call(get_nn!()) } else { Branch::None },
        0xCC => if z { Branch::Call(get_nn!()) } else { Branch::None },
        0xD4 => if !c { Branch::Call(get_nn!()) } else { Branch::None },
        0xDC => if c { Branch::Call(get_nn!()) } else { Branch::None },

        0xC9 => Branch::Return,
        0xC0 => if !z { Branch::Return } else { Branch::None },
        0xC8 => if z { Branch::Return } else { Branch::None },
        0xD0 => if !c { Branch::Return } else { Branch::None },
        0xD8 => if c { Branch::Return } else { Branch::None },
        0xD9 => Branch::ReturnFromException,

        _ => Branch::None,
    }
}
