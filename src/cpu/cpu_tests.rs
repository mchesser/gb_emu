#[cfg(test)]
mod cpu_tests {
    use cpu::Cpu;
    use cpu::disasm;
    use mmu::Memory;

    const Z: u8 = super::ZERO_FLAG;
    const N: u8 = super::ADD_SUB_FLAG;
    const H: u8 = super::HALF_CARRY_FLAG;
    const C: u8 = super::CARRY_FLAG;

    /// Initialize a cpu and mmu to be used for testing
    fn init() -> (Box<Cpu>, Box<Memory>) {
        let mem = box Memory::new();
        let mut cpu = box Cpu::new();
        cpu.crashed = false;
        // Run the program from the ram instead of rom, so that we can write to it
        cpu.pc = 0xE000;
        (cpu, mem)
    }

    fn instruction_runner(cpu: &mut Cpu, mem: &mut Memory, times: uint) -> uint {
        let mut num_cycles = 0;
        for _ in range(0, times) {
            println!("{:04X}:\t\t{}", cpu.pc, disasm::disasm(cpu.pc, mem));
            num_cycles += cpu.step(mem) as uint;
            println!("{}", num_cycles);
        }
        num_cycles
    }

    //
    // GMB 8bit - Loadcommands
    //

    #[test]
    fn test_ld8_basic1() {
        let (mut c, mut m) = init();
        c.h = 0x8A;
        c.e = 0x10;

        m.sb(c.pc + 0, 0x63);   // ld h, e
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.h, c.e);
        assert_eq!(c.h, 0x10);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_ld8_basic2() {
        let (mut c, mut m) = init();
        c.e = 0x10;

        m.sw(c.pc + 0, 0xA51E); // ld e, 0xA5
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.e, 0xA5);
        assert_eq!(c.pc, 0xE002);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn test_ld_aHL() {
        let (mut c, mut m) = init();
        c.a = 0x10;
        c.hl().set(0xE002);
        m.sb(c.hl().get(), 0xCD);

        m.sb(c.pc + 0, 0x7E);   // ld a, (HL)
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0xCD);
        assert_eq!(cycles, 8);
    }

    #[test]
    fn test_ldd_HLa() {
        let (mut c, mut m) = init();
        c.a = 0x10;
        c.hl().set(0xE002);
        m.sb(c.hl().get(), 0xCD);

        m.sb(c.pc + 0, 0x32);   // ldd (HL), a
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(m.lb(0xE002), 0x10);
        assert_eq!(c.hl().get(), 0xE001);
        assert_eq!(cycles, 8);
    }

    //
    // GMB 8bit-Arithmetic/logical Commands
    //
    #[test]
    fn test_add_basic1() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x44;
        c.c = 0x11;

        m.sb(c.pc + 0, 0x81);   // add a, c
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0x55);
        assert_eq!(c.f, 0);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_add_zeroflag_set() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x00;
        c.c = 0x00;

        m.sb(c.pc + 0, 0x81);   // add a, c
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0x00);
        assert_eq!(c.f, Z);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_add_halfcarry_set() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x08;
        c.c = 0x08;

        m.sb(c.pc + 0, 0x81);   // add a, c
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0x10);
        assert_eq!(c.f, H);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_add_fullcarry_set() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x80;
        c.c = 0xFF;

        m.sb(c.pc + 0, 0x81);   // add a, c
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0x7F);
        assert_eq!(c.f, C);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_add_allflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0xFF;
        c.c = 0x01;

        m.sb(c.pc + 0, 0x81);   // add a, c
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0x00);
        assert_eq!(c.f, C | H | Z);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_add_aHL_allflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0xFF;
        c.hl().set(0xE002);
        m.sb(c.hl().get(), 0x01);

        m.sb(c.pc + 0, 0x86);   // add a, (HL)
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0x00);
        assert_eq!(c.f, C | H | Z);
        assert_eq!(cycles, 8);
    }

    // HERE GOES TESTS FOR SUB !!!

    #[test]
    fn test_inc_basic1() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0x01;

        m.sb(c.pc + 0, 0x3C);   // inc a
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0x02);
        assert_eq!(c.f, 0);
        assert_eq!(cycles, 4);
    }

    #[test]
    fn test_inc_allflags() {
        let (mut c, mut m) = init();
        c.f = 0x00;

        c.a = 0xFF;

        m.sb(c.pc + 0, 0x3C);   // inc a
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.a, 0x00);
        assert_eq!(c.f, Z | H);
        assert_eq!(cycles, 4);
    }

    //
    // GMB 16bit - Loadcommands
    //

    #[test]
    fn test_ld16_basic1() {
        let (mut c, mut m) = init();
        c.b = 0xFF;
        c.c = 0xFF;

        m.sb(c.pc + 0, 0x01);   // ld bc,
        m.sw(c.pc + 1, 0xABCD); // 0xABCD
        let cycles = instruction_runner(&mut *c, &mut *m, 1);
        assert_eq!(c.bc().get(), 0xABCD);
        assert_eq!(c.pc, 0xE003);
        assert_eq!(cycles, 12);
    }

    #[test]
    fn test_push16_pop16() {
        let (mut c, mut m) = init();
        c.bc().set(0x1200);

        m.sb(c.pc + 0, 0xC5);   // push bc
        m.sb(c.pc + 1, 0xF1);   // pop af
        m.sb(c.pc + 2, 0xF5);   // push af
        m.sb(c.pc + 3, 0xD1);   // pop de
        m.sb(c.pc + 4, 0x79);   // ld a,c
        m.sw(c.pc + 5, 0xF0E6); // and $F0

        let cycles = instruction_runner(&mut *c, &mut *m, 6);
        assert_eq!(c.a, c.e);
        assert_eq!(cycles, 16 + 12 + 16 + 12 + 4 + 8)
    }

}
