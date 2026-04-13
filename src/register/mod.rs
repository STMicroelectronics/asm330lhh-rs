pub mod main;

use super::prelude::*;

use derive_more::TryFrom;
use st_mem_bank_macro::mem_bank;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom, Debug)]
#[try_from(repr)]
#[mem_bank(Asm330lhh, generics = 2)]
pub enum MemBank {
    #[default]
    #[main]
    UserBank = 0,
}

#[derive(Default)]
pub struct AllSources {
    pub all_int_src: AllIntSrc,
    pub wake_up_src: WakeUpSrc,
    pub d6d_src: D6dSrc,
    pub status_reg: StatusReg,
}

#[derive(Default)]
pub struct PinInt1Route {
    pub int1_ctrl: Int1Ctrl,
    pub md1_cfg: Md1Cfg,
}

#[derive(Default)]
pub struct PinInt2Route {
    pub int2_ctrl: Int2Ctrl,
    pub md2_cfg: Md2Cfg,
}
