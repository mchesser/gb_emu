#[cfg(test)]
#[allow(non_snake_case)]
mod cpu_tests {
    use crate::{cpu::Cpu, emulator::Emulator, mmu::Memory};
    // use debug::disasm;

    const Z: u8 = super::ZERO_FLAG;
    const N: u8 = super::ADD_SUB_FLAG;
    const H: u8 = super::HALF_CARRY_FLAG;
    const C: u8 = super::CARRY_FLAG;

    /// Initialize a cpu and mmu to be used for testing
    fn init() -> (Cpu, Memory) {
        let mem = Memory::new();
        let mut cpu = Cpu::new();
        cpu.crashed = false;
        // Run the program from the ram instead of rom, so that we can write to it
        cpu.pc = 0xE000;
        (cpu, mem)
    }

    fn instruction_runner(cpu: &mut Cpu, mem: &mut Memory, times: usize) -> usize {
        let mut num_cycles = 0;
        for _ in 0..times {
            // Print out the instructions for debugging
            // println!("{:04X}:\t\t{}", cpu.pc, disasm::disasm(cpu.pc, mem));
            num_cycles += cpu.step(mem) as usize;
        }
        num_cycles
    }

    //
    // Creation test
    //
    #[test]
    fn start_up() {
        let mut emulator = Emulator::new();
        emulator.start();
        assert_eq!(emulator.cpu.af().get(), 0x01B0);
    }

    //
    // GMB 8bit - Loadcommands
    //

    #[test]
    fn ld8_basic1() {
        let (mut c, mut m) = init();
        c.h = 0x8A;
        c.e = 0x10;

        m.sb(c.pc + 0, 0x63); // ld h, e
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.h, c.e);
        assert_eq!(c.h, 0x10);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn ld8_basic2() {
        let (mut c, mut m) = init();
        c.e = 0x10;

        m.sw(c.pc + 0, 0xA51E); // ld e, 0xA5
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.e, 0xA5);
        assert_eq!(c.pc, 0xE002);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn ld_aHL() {
        let (mut c, mut m) = init();
        c.a = 0x10;
        c.hl().set(0xE002);
        m.sb(c.hl().get(), 0xCD);

        m.sb(c.pc + 0, 0x7E); // ld a, (HL)
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0xCD);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn ldd_HLa() {
        let (mut c, mut m) = init();
        c.a = 0x10;
        c.hl().set(0xE002);
        m.sb(c.hl().get(), 0xCD);

        m.sb(c.pc + 0, 0x32); // ldd (HL), a
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(m.lb(0xE002), 0x10);
        assert_eq!(c.hl().get(), 0xE001);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn inc_basic1() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x01;

        m.sb(c.pc + 0, 0x3C); // inc a
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0x02);
        assert_eq!(c.f, 0);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn inc_allflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0xFF;

        m.sb(c.pc + 0, 0x3C); // inc a
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0x00);
        assert_eq!(c.f, Z | H);
        assert_eq!(cycles, 4);
    }

    //
    // GMB 16bit - Loadcommands
    //

    #[test]
    fn ld16_basic1() {
        let (mut c, mut m) = init();
        c.b = 0xFF;
        c.c = 0xFF;

        m.sb(c.pc + 0, 0x01); // ld bc,
        m.sw(c.pc + 1, 0xABCD); // 0xABCD
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.bc().get(), 0xABCD);
        assert_eq!(c.pc, 0xE003);
        assert_eq!(cycles, 12);
    }

    #[test]
    fn load_spHL() {
        let (mut c, mut m) = init();
        c.sp = 0xFF;
        c.hl().set(0xAB);

        m.sb(c.pc + 0, 0xF9); // ld sp,hl
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.sp, 0xAB);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn pushpop_af() {
        let (mut c, mut m) = init();
        c.bc().set(0x1201);
        c.sp = 0xE200;

        m.sb(c.pc + 0, 0xC5); // push bc
        m.sb(c.pc + 1, 0xF1); // pop af
        m.sb(c.pc + 2, 0xF5); // push af
        m.sb(c.pc + 3, 0xD1); // pop de
        m.sb(c.pc + 4, 0x79); // ld a,c
        m.sw(c.pc + 5, 0xF0E6); // and $F0

        let cycles = instruction_runner(&mut c, &mut m, 6);
        assert_eq!(c.a, c.e);
        assert_eq!(c.sp, 0xE200);
        assert_eq!(cycles, 16 + 12 + 16 + 12 + 4 + 8)
    }

    //
    // GMB 8bit-Arithmetic/logical Commands
    //

    #[test]
    fn add_noflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x44;
        c.c = 0x11;

        m.sb(c.pc + 0, 0x81); // add a, c
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0x55);
        assert_eq!(c.f, 0);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn add_zeroflag_set() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x00;
        c.c = 0x00;

        m.sb(c.pc + 0, 0x81); // add a, c
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0x00);
        assert_eq!(c.f, Z);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn add_halfcarry_set() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x08;
        c.c = 0x08;

        m.sb(c.pc + 0, 0x81); // add a, c
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0x10);
        assert_eq!(c.f, H);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn add_fullcarry_set() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x80;
        c.c = 0xFF;

        m.sb(c.pc + 0, 0x81); // add a, c
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0x7F);
        assert_eq!(c.f, C);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn add_allflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0xFF;
        c.c = 0x01;

        m.sb(c.pc + 0, 0x81); // add a, c
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0x00);
        assert_eq!(c.f, C | H | Z);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn add_aHL_allflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0xFF;
        c.hl().set(0xE002);
        m.sb(c.hl().get(), 0x01);

        m.sb(c.pc + 0, 0x86); // add a, (HL)
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.a, 0x00);
        assert_eq!(c.f, C | H | Z);
        assert_eq!(cycles, 8);
    }

    // HERE GOES TESTS FOR SUB !!!

    //
    // GMB 16bit - Arithmetic/logical commands
    //

    #[test]
    fn add_HL16_noflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.hl().set(0x4242);
        c.de().set(0x1111);

        m.sb(c.pc + 0, 0x19); // add hl,de
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.hl().get(), (0x4242_u16).wrapping_add(0x1111_u16));
        assert_eq!(c.f, 0);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn add_HL16_halfcarry() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.hl().set(0x1F11);
        c.de().set(0x1111);

        m.sb(c.pc + 0, 0x19); // add hl,de
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.hl().get(), (0x1F11_u16).wrapping_add(0x1111_u16));
        assert_eq!(c.f, H);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn add_HL16_fullcarry() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.hl().set(0xF111);
        c.de().set(0x1111);

        m.sb(c.pc + 0, 0x19); // add hl,de
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.hl().get(), (0xF111_u16).wrapping_add(0x1111_u16));
        assert_eq!(c.f, C);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn add_HL16_allflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.hl().set(0xFFFF);
        c.de().set(0x0001);

        m.sb(c.pc + 0, 0x19); // add hl,de
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.hl().get(), 0x0000);
        assert_eq!(c.f, H | C);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn inc16_basic1() {
        let (mut c, mut m) = init();
        c.hl().set(0x0000);

        m.sb(c.pc + 0, 0x23); // inc hl
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.hl().get(), 0x0001);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn inc16_basic2() {
        let (mut c, mut m) = init();
        c.hl().set(0xFFFF);

        m.sb(c.pc + 0, 0x23); // inc hl
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.hl().get(), 0x0000);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn dec16_basic1() {
        let (mut c, mut m) = init();
        c.hl().set(0xFFFF);

        m.sb(c.pc + 0, 0x2B); // dec hl
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.hl().get(), 0xFFFE);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn dec16_basic2() {
        let (mut c, mut m) = init();
        c.hl().set(0x0000);

        m.sb(c.pc + 0, 0x2B); // dec hl
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.hl().get(), 0xFFFF);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn add_spn_basic1() {
        let (mut c, mut m) = init();
        c.sp = 100;

        m.sb(c.pc + 0, 0xE8); // add sp,
        m.sb(c.pc + 1, 2); // 2
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.sp, 102);
        assert_eq!(c.pc, 0xE002);
        assert_eq!(cycles, 16);
    }

    #[test]
    fn add_spn_basic2() {
        let (mut c, mut m) = init();
        c.sp = 100;

        m.sb(c.pc + 0, 0xE8); // add sp,
        m.sb(c.pc + 1, (-2i8) as u8); // -2
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.sp, 98);
        assert_eq!(c.pc, 0xE002);
        assert_eq!(cycles, 16);
    }

    #[test]
    fn ld_hlspn_basic1() {
        let (mut c, mut m) = init();
        c.sp = 100;
        c.hl().set(0);

        m.sb(c.pc + 0, 0xF8); // ld hl,sp +
        m.sb(c.pc + 1, 2); // 2
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.sp, 100);
        assert_eq!(c.hl().get(), 102);
        assert_eq!(c.pc, 0xE002);
        assert_eq!(cycles, 12);
    }

    #[test]
    fn ld_hlspn_basic2() {
        let (mut c, mut m) = init();
        c.sp = 100;
        c.hl().set(0);

        m.sb(c.pc + 0, 0xF8); // ld hl,sp +
        m.sb(c.pc + 1, (-2i8) as u8); // -2
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.sp, 100);
        assert_eq!(c.hl().get(), 98);
        assert_eq!(c.pc, 0xE002);
        assert_eq!(cycles, 12);
    }

    //
    // GMB Jumpcommands
    //

    #[test]
    fn jp() {
        let (mut c, mut m) = init();

        m.sb(c.pc + 0, 0xC3); // jp
        m.sw(c.pc + 1, 0x1234); // 0x1234
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0x1234);
        assert_eq!(cycles, 16);
    }

    #[test]
    fn jp_hl() {
        let (mut c, mut m) = init();
        c.hl().set(0x1234);

        m.sb(c.pc + 0, 0xE9); // jp hl
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0x1234);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn jp_cond_zeroflag() {
        let (mut c, mut m) = init();
        c.f = 0;

        m.sb(c.pc + 0, 0xCA); // jp z
        m.sw(c.pc + 1, 0x1234); // 0x1234
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE003);
        assert_eq!(cycles, 12);

        c.f = Z;
        m.sb(c.pc + 0, 0xCA); // jp z
        m.sw(c.pc + 1, 0x1234); // 0x1234
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0x1234);
        assert_eq!(cycles, 16);
    }

    #[test]
    fn jp_cond_notzeroflag() {
        let (mut c, mut m) = init();
        c.f = Z;

        m.sb(c.pc + 0, 0xC2); // jp nz
        m.sw(c.pc + 1, 0x1234); // 0x1234
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE003);
        assert_eq!(cycles, 12);

        c.f = 0;
        m.sb(c.pc + 0, 0xC2); // jp nz
        m.sw(c.pc + 1, 0x1234); // 0x1234
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0x1234);
        assert_eq!(cycles, 16);
    }

    #[test]
    fn jp_cond_carryflag() {
        let (mut c, mut m) = init();
        c.f = 0;

        m.sb(c.pc + 0, 0xDA); // jp c
        m.sw(c.pc + 1, 0x1234); // 0x1234
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE003);
        assert_eq!(cycles, 12);

        c.f = C;
        m.sb(c.pc + 0, 0xDA); // jp c
        m.sw(c.pc + 1, 0x1234); // 0x1234
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0x1234);
        assert_eq!(cycles, 16);
    }

    #[test]
    fn jr_positive() {
        let (mut c, mut m) = init();

        m.sb(c.pc + 0, 0x18); // jr
        m.sb(c.pc + 1, 2); // 2
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE002 + 2);
        assert_eq!(cycles, 12);
    }

    #[test]
    fn jr_negative() {
        let (mut c, mut m) = init();

        m.sb(c.pc + 0, 0x18); // jr
        m.sb(c.pc + 1, (-2i8) as u8); // -2
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE002 - 2);
        assert_eq!(cycles, 12);
    }

    #[test]
    fn jr_cond() {
        let (mut c, mut m) = init();
        c.f = 0;

        m.sb(c.pc + 0, 0x28); // jr z
        m.sb(c.pc + 1, 2); // 2
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE002);
        assert_eq!(cycles, 8);

        c.f = Z;
        m.sb(c.pc + 0, 0x28); // jr z
        m.sb(c.pc + 1, 2); // 2
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE004 + 2);
        assert_eq!(cycles, 12);
    }

    #[test]
    fn call_ret() {
        let (mut c, mut m) = init();
        c.sp = 0xE200;

        m.sb(c.pc + 0, 0xCD); // call
        m.sw(c.pc + 1, 0xE100); // 0xE100
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE100);
        assert_eq!(c.sp, 0xE200 - 2);
        assert_eq!(m.lw(c.sp), 0xE003);
        assert_eq!(cycles, 24);

        m.sb(c.pc + 0, 0xC9); // ret
        let cycles = instruction_runner(&mut c, &mut m, 1);
        assert_eq!(c.pc, 0xE003);
        assert_eq!(c.sp, 0xE200);
        assert_eq!(cycles, 16);
    }
}
