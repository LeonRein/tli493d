use crate::types::{Diagnostics, RawReading};

pub(crate) const REG_CONFIG: usize = 0x10;
pub(crate) const REG_MOD1: usize = 0x11;
pub(crate) const REG_MOD2: usize = 0x13;
pub(crate) const REG_CONFIG2: usize = 0x14;

pub(crate) const REG_CACHE_LEN: usize = 0x18;

pub(crate) fn set_bits(orig: u8, mask: u8, shift: u8, value: u8) -> u8 {
    (orig & !mask) | ((value << shift) & mask)
}

pub(crate) fn set_bit(orig: u8, mask: u8, enabled: bool) -> u8 {
    if enabled {
        orig | mask
    } else {
        orig & !mask
    }
}

pub(crate) fn decode_data_frame(frame: &[u8; 7]) -> (RawReading, Diagnostics) {
    let x = sign_extend(12, ((frame[0] as u16) << 4) | ((frame[4] as u16) >> 4));
    let y = sign_extend(12, ((frame[1] as u16) << 4) | ((frame[4] as u16) & 0x0f));
    let z = sign_extend(12, ((frame[2] as u16) << 4) | ((frame[5] as u16) & 0x0f));
    let temp = sign_extend(10, ((frame[3] as u16) << 2) | (((frame[5] as u16) >> 6) & 0x03));

    (
        RawReading { x, y, z, temp },
        Diagnostics::from_diag_byte(frame[6]),
    )
}

pub(crate) fn calculate_fuse_parity(mod1: u8, mod2: u8) -> bool {
    let parity = (mod1 & !0x80) ^ (mod2 & 0x80);
    parity.count_ones() % 2 == 0
}

pub(crate) fn set_fuse_parity(mod1: u8, mod2: u8) -> u8 {
    set_bit(mod1, 0x80, calculate_fuse_parity(mod1, mod2))
}

pub(crate) fn temperature_to_c(raw_temp: i16) -> f32 {
    (((raw_temp as f32 * 4.0) - 1180.0) * 0.24) + 25.0
}

fn sign_extend(bits: u8, value: u16) -> i16 {
    let shift = 16 - bits;
    ((value << shift) as i16) >> shift
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_frame_sign_extension() {
        let frame = [0xf0, 0x10, 0x80, 0x50, 0xa5, 0x8c, 0x13];
        let (raw, diag) = decode_data_frame(&frame);

        assert_eq!(raw.x, -246);
        assert_eq!(raw.y, 261);
        assert_eq!(raw.z, -2036);
        assert_eq!(raw.temp, 322);
        assert_eq!(diag.frame, 0x03);
    }

    #[test]
    fn fuse_parity_bit_updates_mod1() {
        let mod1 = 0x12;
        let mod2 = 0x80;
        let out = set_fuse_parity(mod1, mod2);

        let parity = out ^ (mod2 & 0x80);
        assert_eq!(parity.count_ones() % 2, 1);
    }
}
