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

pub(crate) fn calculate_fuse_parity(mod1: u8, mod2: u8, mod2_prd_mask: u8) -> bool {
    let parity = (mod1 & !0x80) ^ (mod2 & mod2_prd_mask);
    parity.count_ones().is_multiple_of(2)
}

pub(crate) fn set_fuse_parity(mod1: u8, mod2: u8, mod2_prd_mask: u8) -> u8 {
    set_bit(mod1, 0x80, calculate_fuse_parity(mod1, mod2, mod2_prd_mask))
}

pub(crate) fn set_config_parity(config: u8, wakeup_odd_parity: bool) -> u8 {
    let data_has_odd_ones = (config & !0x01).count_ones() % 2 == 1;
    let parity = if wakeup_odd_parity {
        !data_has_odd_ones
    } else {
        data_has_odd_ones
    };
    set_bit(config, 0x01, parity)
}

pub(crate) fn has_valid_bus_parity(frame: &[u8; 7], diag: Diagnostics) -> bool {
    let payload_xor = frame[..6].iter().copied().reduce(|a, b| a ^ b).unwrap_or(0);
    let expected_p = payload_xor.count_ones() % 2 == 0;
    expected_p == diag.p
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
        let out = set_fuse_parity(mod1, mod2, 0x80);

        let parity = out ^ (mod2 & 0x80);
        assert_eq!(parity.count_ones() % 2, 1);
    }

    // Fuse parity is ODD over MOD1 (all bits incl. FP) and MOD2 bit 7 (PRD)
    // — user manual §1.2.4. Exhaustively check the invariant.
    #[test]
    fn fuse_parity_is_odd_over_mod1_and_mod2_prd() {
        for mod1 in 0u8..=0xFF {
            for &mod2 in &[0x00u8, 0x01, 0x80, 0x81] {
                let out = set_fuse_parity(mod1, mod2, 0x80);
                let total_ones = out.count_ones() + (mod2 & 0x80).count_ones();
                assert_eq!(total_ones % 2, 1, "mod1={mod1:#04x} mod2={mod2:#04x}");
            }
        }
    }

    // The A2B6 address-change to slot A2 sets IICADR=0b10 on a base MOD1 of
    // 0x94 (PR|INT). With factory MOD2=0x01 (PRD=0) the written byte is 0x54,
    // FP=0 — the exact value observed on hardware.
    #[test]
    fn address_change_mod1_matches_hardware() {
        let base_mod1 = 0x94; // PR=1, INT=1, IICADR=0, MODE=0, FP=1
        let mod1_a2 = set_bits(base_mod1, 0x60, 5, 0b10); // IICADR = A2
        assert_eq!(set_fuse_parity(mod1_a2, 0x01, 0x80), 0x54);
    }

    // Config parity is EVEN over register 0x10 (incl. CP) — user manual §1.2.3.
    #[test]
    fn config_parity_is_even() {
        for config in 0u8..=0xFF {
            let out = set_config_parity(config, false);
            assert_eq!(out.count_ones() % 2, 0, "config={config:#04x}");
        }
        // Concrete cases: all-zero CONFIG -> CP=0; X2 set (0x08) -> CP=1.
        assert_eq!(set_config_parity(0x00, false), 0x00);
        assert_eq!(set_config_parity(0x08, false), 0x09);
    }
}
