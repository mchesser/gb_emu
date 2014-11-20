use mmu::Memory;
use cpu::Interrupt;

pub const HEIGHT: uint = 144;
pub const WIDTH: uint = 160;

const VRAM_SIZE: uint = 0x2000;
const OAM_SIZE: uint = 0xA0;

pub mod timings {
    pub const OAM_READ: u16 = 80;
    pub const VRAM_READ: u16 = 172;
    pub const HBLANK: u16 = 204;
    pub const VBLANK: u16 = 456;

    pub const FULL_FRAME: u16 = (OAM_READ + VRAM_READ + HBLANK) * 144 + VBLANK * 10;
}

type Color = [u8, ..4];
const GB_COLOR_TABLE: &'static [Color] = &[
    [0xFF, 0xFF, 0xFF, 0xFF], // 0% on (white)
    [0xC0, 0xC0, 0xC0, 0xFF], // 33% on (light gray)
    [0x60, 0x60, 0x60, 0xFF], // 66% on (dark gray)
    [0x00, 0x00, 0x00, 0xFF], // 100% on (black)
];

#[repr(u8)]
pub enum Mode {
    HBlank   = 0,
    VBlank   = 1,
    OamRead  = 2,
    VramRead = 3,
}

pub struct Gpu {
    pub mode: Mode,
    pub mode_clock: u16,

    /// The GB display. Colours are defined in the order: (r, g, b, a). And the pixels are ordered
    /// by row ([ row1, row2, row3, ...]).
    framebuffer: [[u8, ..HEIGHT * WIDTH * 4], ..2],
    /// The buffer that is being drawn too.
    back_buffer_id: uint,
    pub ready_flag: bool,

    /// The current line being processed by the GBP (0 ... 142 = Hblank, 143 ... 153 = Vblank)
    pub line: u8,

    /// The GPU vram (mapped to: 0x8000-0x9FFF)
    pub vram: [[u8, ..VRAM_SIZE], ..2],
    /// The active VRAM bank. CGB only (mapped to: 0xFF4F)
    pub vram_bank: u8,

    /// The GB sprite access table (OAM) (mapped to: [0xFE00-0xFF9F])
    pub oam: [u8, ..OAM_SIZE],

    /// LCD Control Register (mapped to: 0xFF40)
    pub lcdc: u8,
    /// LCD Status Register (mapped to: 0xFF42)
    pub stat: u8,

    /// Scroll Y Register (mapped to: 0xFF42)
    pub scy: u8,
    /// Scroll X Register (mapped to: 0xFF43)
    pub scx: u8,
    /// LCDC Y-Coordinate Register (mapped to: 0xFF44)
    pub ly: u8,
    /// LY Compare Register (mapped to: 0xFF45)
    pub lyc: u8,
    /// Window Y Position Register (mapped to: 0xFF4A)
    pub wy: u8,
    /// Window X Position Register (mapped to: 0xFF4B)
    pub wx: u8,

    /// Bg Palette Data Register (mapped to: 0xFF47)
    pub bgp: u8,
    /// Object Palette 0 Data (mapped to: 0xFF48)
    pub obp0: u8,
    /// Object Palette 1 Data (mapped to: 0xFF49)
    pub obp1: u8,

    // TODO: implement CGB colour palettes
    // pub bcps: u8,
    // pub bcpd: u8,
    // pub ocps: u8,
    // pub ocpd: u8,

    /// LCD OAM DMA controller (mapped to: 0xFF46)
    pub dma: u8,

    // TODO: implement CGB DMA controllers
    // pub hdma1: u8,
    // pub hdma2: u8,
    // pub hdma3: u8,
    // pub hdma4: u8,
    // pub hdma5: u8,
}

impl Gpu {
    pub fn new() -> Gpu {
        Gpu {
            mode: Mode::HBlank,
            mode_clock: 0,
            framebuffer: [[0, ..HEIGHT * WIDTH * 4], ..2],
            back_buffer_id: 0,
            ready_flag: false,
            line: 0,
            vram: [[0, ..VRAM_SIZE], ..2],
            vram_bank: 0,
            oam: [0, ..OAM_SIZE],
            lcdc: 0,
            stat: 0,
            scy: 0,
            scx: 0,
            ly: 0,
            lyc: 0,
            wy: 0,
            wx: 0,
            bgp: 0,
            obp0: 0,
            obp1: 0,
            // TODO: implement CGB colour palettes
            // bcps: 0,
            // bcpd: 0,
            // ocps: 0,
            // ocpd: 0,
            dma: 0,
            // TODO: implement CGB DMA controllers
            // hdma1: 0,
            // hdma2: 0,
            // hdma3: 0,
            // hdma4: 0,
            // hdma5: 0,
        }
    }

    /// Returns the currently banked vram
    pub fn vram(&mut self) -> &mut [u8] {
        self.vram[self.vram_bank as uint]
    }

    /// Returns the back buffer
    pub fn back_buffer(&mut self) -> &mut [u8] {
        self.framebuffer[self.back_buffer_id]
    }

    /// Returns the front buffer
    pub fn front_buffer(&mut self) -> &mut [u8] {
        self.framebuffer[1 - self.back_buffer_id]
    }

    /// Flip the front and back buffers
    fn flip_buffers(&mut self) {
        self.back_buffer_id = 1 - self.back_buffer_id;
        self.ready_flag = true;
    }

    /// Set the current GPU mode
    pub fn set_mode(&mut self, mode: Mode) {
        self.mode_clock = 0;
        self.mode = mode;
    }

    /// Render background pixels
    pub fn render_bg_scanline(&mut self) {
        let tile_y = (self.line + self.scy) % 8;
        let mut tile_x = self.scx % 8;

        let line_offset = self.tilemap_offset() + ((self.line + self.scy) / 8) as u16;
        let mut tile_offset = (self.scx / 8) as u16;
        let mut tile_id = self.vram()[(line_offset + tile_offset) as uint];

        let mut draw_offset = self.line as uint * WIDTH * 4;
        for _ in range(0, WIDTH) {
            let color = GB_COLOR_TABLE[self.tile_lookup(tile_id, tile_x, tile_y)];

            // Write pixel to framebuffer
            self.back_buffer()[draw_offset + 0] = color[0];
            self.back_buffer()[draw_offset + 1] = color[1];
            self.back_buffer()[draw_offset + 2] = color[2];
            self.back_buffer()[draw_offset + 3] = color[3];
            draw_offset += 4;

            tile_x += 1;
            if tile_x >= 8 {
                // Reached the end of a tile so read the next one
                tile_x = 0;
                tile_offset += 1;
                tile_id = self.vram()[(line_offset + tile_offset) as uint];
            }
        }
    }

    /// Get the offset of the currently enabled tilemap
    fn tilemap_offset(&self) -> u16 {
        // FIXME(major): support other tilemap
        0x0080
    }

    /// Look up the color value of a pixel in a tile
    fn tile_lookup(&mut self, id: u8, x: u8, y: u8) -> uint {
        let tile_height = 8;
        let index = (id * tile_height * 2 + y * 2) as uint;
        let low = self.vram()[index];
        let high = self.vram()[index + 1];

        // A single pixel is stored over two bytes. The pixels lower bit is stored in the first byte
        // and the high bit is stored in the second byte
        ((((high >> (x as uint)) & 1) << 1) | ((low >> (x as uint)) & 1)) as uint
    }
}

/// Perform a GPU step based on the number of ticks elapsed in the CPU.
/// This serves to synchronize the GPU mode with the CPU.
pub fn step(mem: &mut Memory, ticks: u8) {
    mem.gpu.mode_clock += ticks as u16;

    match mem.gpu.mode {
        Mode::OamRead => {
            if mem.gpu.mode_clock >= timings::OAM_READ {
                mem.gpu.set_mode(Mode::VramRead);
            }
        },

        Mode::VramRead => {
            if mem.gpu.mode_clock >= timings::VRAM_READ {
                mem.gpu.set_mode(Mode::HBlank);
            }
        },

        Mode::HBlank => {
            if mem.gpu.mode_clock >= timings::HBLANK {
                mem.gpu.line += 1;
                if mem.gpu.line > 142 {
                    mem.gpu.set_mode(Mode::VBlank);
                    mem.gpu.flip_buffers();
                    mem.if_reg |= Interrupt::VBlank as u8;
                }
                else {
                    mem.gpu.render_bg_scanline();
                    mem.gpu.set_mode(Mode::OamRead);
                }
            }
        },

        Mode::VBlank => {
            if mem.gpu.mode_clock >= timings::VBLANK {
                mem.gpu.mode_clock = 0;
                mem.gpu.line += 1;
                if mem.gpu.line > 153 {
                    mem.gpu.set_mode(Mode::OamRead);
                    mem.gpu.line = 0;
                }
            }
        },
    }
}
