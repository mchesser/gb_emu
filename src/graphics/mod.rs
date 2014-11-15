pub const HEIGHT: uint = 144;
pub const WIDTH: uint = 160;

pub struct Gpu {
    framebuffer: [u8, ..HEIGHT * WIDTH * 4],
}

impl Gpu {
    pub fn new() -> Gpu {
        unimplemented!()
    }

    pub fn vram(&mut self) -> &mut [u8] {
        unimplemented!()
    }

    pub fn map_oam(&mut self, _addr: u16) -> &mut u8 {
        unimplemented!()
    }
}
