use mmu::Memory;
use cpu::Interrupt;

pub const HEIGHT: uint = 144;
pub const WIDTH: uint = 160;
const VBLANK_END: u8 = 153;

const VRAM_SIZE: uint = 0x2000;
const OAM_SIZE: uint = 0xA0;

pub mod timings {
    pub const OAM_READ: u32 = 80;
    pub const VRAM_READ: u32 = 172;
    pub const HBLANK: u32 = 204;
    pub const VBLANK: u32 = 456;

    pub const FULL_FRAME: u32 = (OAM_READ + VRAM_READ + HBLANK) * 144 + VBLANK * 10;
}

pub type Color = [u8, ..4];
const GB_COLOR_TABLE: &'static [Color] = &[
    [0xFF, 0xFF, 0xFF, 0xFF], // 0% on (white)
    [0xC0, 0xC0, 0xC0, 0xFF], // 33% on (light gray)
    [0x60, 0x60, 0x60, 0xFF], // 66% on (dark gray)
    [0x00, 0x00, 0x00, 0xFF], // 100% on (black)
];
pub fn palette_lookup(palette: u8, color_id: uint) -> Color {
    GB_COLOR_TABLE[((palette >> (2 * color_id)) & 0x3) as uint]
}

#[deriving(Copy)]
pub enum Mode {
    HBlank   = 0,
    VBlank   = 1,
    OamRead  = 2,
    VramRead = 3,
}

struct LcdStatRegister {
    lyc_interrupt: bool,    // Bit 6
    oam_interrupt: bool,    // Bit 5
    vblank_interrupt: bool, // Bit 4
    hblank_interrupt: bool, // Bit 3
    // lcy_flag: bool          Bit 2 (generated automatically)
    mode: Mode,             // Bit 0-1
}

impl LcdStatRegister {
    pub fn new() -> LcdStatRegister {
        LcdStatRegister {
            lyc_interrupt: false,
            oam_interrupt: false,
            vblank_interrupt: false,
            hblank_interrupt: false,
            mode: Mode::HBlank,
        }
    }
}


pub struct Gpu {
    pub mode_clock: u16,

    /// The GB display. Colours are defined in the order: (r, g, b, a). And the pixels are ordered
    /// by row ([ row1, row2, row3, ...]).
    pub framebuffer: [u8, ..HEIGHT * WIDTH * 4],
    /// A flag that indicates that the framebuffer is ready to be read
    pub ready_flag: bool,

    /// The GPU vram (mapped to: 0x8000-0x9FFF)
    pub vram: [[u8, ..VRAM_SIZE], ..2],
    /// The active VRAM bank. CGB only (mapped to: 0xFF4F)
    pub vram_bank: u8,

    /// The GB sprite access table (OAM) (mapped to: [0xFE00-0xFF9F])
    pub oam: [u8, ..OAM_SIZE],

    /// LCD Control Register (mapped to: 0xFF40)
    pub lcdc: u8,
    /// LCD Status Register (mapped to: 0xFF42)
    stat: LcdStatRegister,

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
            mode_clock: 0,
            framebuffer: [0, ..HEIGHT * WIDTH * 4],
            ready_flag: false,
            vram: [[0, ..VRAM_SIZE], ..2],
            vram_bank: 0,
            oam: [0, ..OAM_SIZE],
            lcdc: 0,
            stat: LcdStatRegister::new(),
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

    fn lcd_on(&self) -> bool {
        self.lcdc & 0b1000_0000 != 0
    }

    fn winmap_base_offset(&self) -> u16 {
        if self.lcdc & 0b0100_0000 == 0 { 0x1800 } else { 0x1C00 }
    }

    fn window_display_on(&self) -> bool {
        self.lcdc & 0b0010_0000 != 0
    }

    /// Adjust a tile id based on the currently selected tile set
    fn adjust_tile_id(&self, tile_id: uint) -> uint {
        if self.lcdc & 0b0001_0000 == 0 {
            if tile_id < 128 { return tile_id + 256 }
        }
        tile_id
    }

    fn bgmap_base_offset(&self) -> u16 {
        if self.lcdc & 0b0000_1000 == 0 { 0x1800 } else { 0x1C00 }
    }

    fn get_sprite_height(&self) -> int {
        if self.lcdc & 0b0000_0100 == 0 { 8 } else { 16 }
    }

    fn obj_display_on(&self) -> bool {
        self.lcdc & 0b0000_0010 != 0
    }

    fn bg_display_on(&self) -> bool {
        self.lcdc & 0b0000_0001 != 0
    }

    /// Returns the currently banked vram
    pub fn vram(&self) -> &[u8] {
        &self.vram[self.vram_bank as uint]
    }

    /// Returns the currently banked vram
    pub fn vram_mut(&mut self) -> &mut [u8] {
        &mut self.vram[self.vram_bank as uint]
    }

    /// Returns the value of the lcd status register
    pub fn get_stat(&self) -> u8 {
        let mut result = self.stat.mode as u8;
        if self.stat.lyc_interrupt { result |= 0b01000000 }
        if self.stat.oam_interrupt { result |= 0b00100000 }
        if self.stat.vblank_interrupt { result |= 0b00010000 }
        if self.stat.hblank_interrupt { result |= 0b00001000 }
        if self.lyc == self.ly { result |= 0b00000100 }
        result
    }

    /// Set the value of the lcd status register
    pub fn set_stat(&mut self, val: u8) {
        self.stat.lyc_interrupt =    (val & 0b01000000) != 0;
        self.stat.oam_interrupt =    (val & 0b00100000) != 0;
        self.stat.vblank_interrupt = (val & 0b00010000) != 0;
        self.stat.hblank_interrupt = (val & 0b00001000) != 0;

        // lyc_flag and mode are readonly
    }

    pub fn set_mode(&mut self, mode: Mode) {
        self.mode_clock = 0;
        self.stat.mode = mode;
    }

    pub fn render_scanline(&mut self) {
        if self.lcd_on() {
            if self.bg_display_on() {
                self.render_bg_scanline();
            }
            if self.window_display_on() {
                self.render_window_scanline();
            }
            if self.obj_display_on() {
                self.render_sprite_scanline();
            }
        }
    }

    fn render_bg_scanline(&mut self) {
        // FIXME(major): The background does not wrap
        let map_base = self.bgmap_base_offset() + ((self.ly + self.scy) / 8) as u16 * 32;
        let mut map_offset = (self.scx / 8) as u16;

        let tile_y = (self.ly + self.scy) % 8;
        let mut tile_x = self.scx % 8;

        let mut tile_id = self.vram()[(map_base + map_offset) as uint] as uint;
        tile_id = self.adjust_tile_id(tile_id);

        let mut draw_offset = self.ly as uint * WIDTH * 4;
        for _ in (0..WIDTH) {
            // Not sure why the tile_x needs to be reversed here.
            let color_id = self.tile_lookup(tile_id, 7 - tile_x, tile_y);
            let color = palette_lookup(self.bgp, color_id);

            write_pixel(&mut self.framebuffer, draw_offset as uint, color);
            draw_offset += 4;

            tile_x += 1;
            // Move to the next tile if we have reached the end of this tile
            if tile_x >= 8 {
                tile_x = 0;
                map_offset += 1;
                tile_id = self.vram()[(map_base + map_offset) as uint] as uint;
                tile_id = self.adjust_tile_id(tile_id);
            }
        }
    }

    fn render_window_scanline(&mut self) {
        // If the current line is less than the window's y offset or the window's x offset if off
        // the screen, then we have nothing to do for this scanline.
        if self.ly < self.wy || self.wx >= WIDTH as u8 + 7 {
            return
        }

        let map_base = self.winmap_base_offset() + ((self.ly - self.wy) / 8) as u16 * 32;
        let mut map_offset = 0;

        let tile_y = (self.ly - self.wy) % 8;
        // FIXME(major): correctly handle window wrapping
        let mut tile_x = self.wx - 7;

        let mut tile_id = self.vram()[(map_base + map_offset) as uint] as uint;
        tile_id = self.adjust_tile_id(tile_id);

        let mut draw_offset = self.ly as uint * WIDTH * 4;
        for _ in (0..WIDTH) {
            // Not sure why the tile_x needs to be reversed here.
            let color_id = self.tile_lookup(tile_id, 7 - tile_x, tile_y);
            let color = palette_lookup(self.bgp, color_id);

            write_pixel(&mut self.framebuffer, draw_offset as uint, color);
            draw_offset += 4;

            tile_x += 1;
            // Move to the next tile if we have reached the end of this tile
            // FIXME(major): correctly handle window wrapping
            if tile_x >= 8 {
                tile_x = 0;
                map_offset += 1;
                tile_id = self.vram()[(map_base + map_offset) as uint] as uint;
                tile_id = self.adjust_tile_id(tile_id);
            }
        }
    }

    fn render_sprite_scanline(&mut self) {
        let sprite_height = self.get_sprite_height();
        let line_num = self.ly as int;

        // TODO(major): Set priorities in GB mode
        // TODO(major): Only allow 10 sprites per scanline in CGB mode
        let mut _num_sprites = 0_i32;

        // FIXME(minor): Can we do this more efficiently?
        for sprite in self.oam.chunks(4) {
            // Read sprite attributes
            let mut dy = sprite[0] as int - 16;
            let dx = sprite[1] as int - 8;
            let mut tile_id = sprite[2];
            let flags = sprite[3];

            // Check if the sprite appears on this scanline
            if dy > line_num || dy + sprite_height <= line_num || dx <= -8 || dx >= WIDTH as int {
                continue
            }
            _num_sprites += 1;

            // 8x16 sprites consist of two adjacent tiles, so adjust the tile id based on where we
            // are in the sprite
            if sprite_height == 16 {
                tile_id &= 0xFE;
                if self.ly as int - dy >= 8 {
                    tile_id |= 1;
                    dy += 8;
                }
            }

            let mut draw_offset = (line_num * WIDTH as int + dx as int) * 4;
            let palette = if flags & 0x10 == 0 { self.obp0 } else { self.obp1 };

            // Get the y coordinate of the sprite. If bit 6 is set, the sprite is flipped so we take
            // the y offset from the bottom of the sprite.
            // CHECKME: how are flipped 8x16 sprites handled
            let tile_y = if flags & 0x40 == 0 { line_num - dy } else { 7 - (line_num - dy) } as u8;
            assert!(tile_y < 8);

            for x in (0..8) {
                if dx + x >= 0 && (dx as int + x as int) < WIDTH as int {
                    // Flip x coordinate if bit 5 is set
                    let tile_x = if flags & 0x20 == 0 { 7 - x } else { x } as u8;
                    assert!(tile_x < 8);

                    let color_id = self.tile_lookup(tile_id as uint, tile_x, tile_y);

                    // Pixels in a sprite with a color id of 0 are transparent
                    if color_id != 0 {
                        let color = palette_lookup(palette, color_id);
                        write_pixel(&mut self.framebuffer, draw_offset as uint, color);
                    }
                }
                draw_offset += 4;
            }

        }
    }

    /// Look up the color value of a pixel in a tile
    fn tile_lookup(&self, id: uint, x: u8, y: u8) -> uint {
        let tile_height = 8;

        let index = id as uint * tile_height * 2 + y as uint * 2;

        // A single pixel is stored over two bytes. The pixels lower bit is stored in the first byte
        // and the high bit is stored in the second byte
        let low = self.vram()[index];
        let high = self.vram()[index + 1];
        ((((high >> (x as uint)) & 1) << 1) | ((low >> (x as uint)) & 1)) as uint
    }
}

/// Perform a GPU step based on the number of ticks elapsed in the CPU.
/// This serves to synchronize the GPU mode with the CPU.
pub fn step(mem: &mut Memory, ticks: u8) {
    mem.gpu.mode_clock += ticks as u16;

    match mem.gpu.stat.mode {
        Mode::OamRead => {
            if mem.gpu.mode_clock >= timings::OAM_READ as u16 {
                mem.gpu.set_mode(Mode::VramRead);
            }
        },

        Mode::VramRead => {
            if mem.gpu.mode_clock >= timings::VRAM_READ as u16 {
                mem.gpu.set_mode(Mode::HBlank);
                mem.gpu.render_scanline();
                if mem.gpu.stat.hblank_interrupt {
                    mem.if_reg |= Interrupt::Stat as u8;
                }
            }
        },

        Mode::HBlank => {
            if mem.gpu.mode_clock >= timings::HBLANK as u16 {
                mem.gpu.ly += 1;
                if mem.gpu.ly >= HEIGHT as u8 {
                    mem.gpu.set_mode(Mode::VBlank);
                    mem.gpu.ready_flag = true;
                    mem.if_reg |= Interrupt::VBlank as u8;
                }
                else {
                    mem.gpu.set_mode(Mode::OamRead);
                    if mem.gpu.stat.oam_interrupt {
                        mem.if_reg |= Interrupt::Stat as u8;
                    }
                }
            }
        },

        Mode::VBlank => {
            if mem.gpu.mode_clock >= timings::VBLANK as u16 {
                mem.gpu.mode_clock = 0;
                mem.gpu.ly += 1;
                if mem.gpu.ly > VBLANK_END {
                    mem.gpu.set_mode(Mode::OamRead);
                    mem.gpu.ly = 0;
                }

                if mem.gpu.lyc == mem.gpu.ly && mem.gpu.stat.lyc_interrupt {
                    mem.if_reg |= Interrupt::Stat as u8;
                }
            }
        },
    }
}

/// Write a pixel to an offset in the frame buffer
fn write_pixel(framebuffer: &mut [u8], offset: uint, color: Color) {
    framebuffer[offset..(offset + 4)] = &color;
}

/// Perform a DMA transfer from memory to the sprite access table
pub fn oam_dma_transfer(mem: &mut Memory) {
    let start_addr = mem.gpu.dma as u16 << 8;

    for i in (0..OAM_SIZE) {
        mem.gpu.oam[i] = mem.lb(start_addr + i as u16);
    }
}
