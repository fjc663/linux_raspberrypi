// SPDX-License-Identifier: GPL-2.0

//! This is Pl011 ARM Device Data

use crate::device::Device;

use core::{
    ffi::c_void,
    ptr::NonNull,
};

/// Define constants for UART registers
/// Data read or written from the interface
pub const UART01X_DR: u32 = 0x00;
/// Receive status register (Read)
pub const UART01X_RSR: u32 = 0x04;
/// Error clear register (Write)
pub const UART01X_ECR: u32 = 0x04;
/// Line control register, high byte
pub const UART010_LCRH: u32 = 0x08;
/// DMA watermark configure register
pub const ST_UART011_DMAWM: u32 = 0x08;
/// Line control register, middle byte
pub const UART010_LCRM: u32 = 0x0C;
/// Timeout period register
pub const ST_UART011_TIMEOUT: u32 = 0x0C;
/// Line control register, low byte
pub const UART010_LCRL: u32 = 0x10;
/// Control register
pub const UART010_CR: u32 = 0x14;
/// Flag register (Read only)
pub const UART01X_FR: u32 = 0x18;
/// Interrupt identification register (Read)
pub const UART010_IIR: u32 = 0x1C;
/// Interrupt clear register (Write)
pub const UART010_ICR: u32 = 0x1C;
/// Rx line control register
pub const ST_UART011_LCRH_RX: u32 = 0x1C;
/// IrDA low power counter register
pub const UART01X_ILPR: u32 = 0x20;
/// Integer baud rate divisor register
pub const UART011_IBRD: u32 = 0x24;
/// Fractional baud rate divisor register
pub const UART011_FBRD: u32 = 0x28;
/// Line control register
pub const UART011_LCRH: u32 = 0x2c;
/// Tx Line control register
pub const ST_UART011_LCRH_TX: u32 = 0x2c;
/// Control register
pub const UART011_CR: u32 = 0x30;
/// Interrupt fifo level select
pub const UART011_IFLS: u32 = 0x34;
/// Interrupt mask
pub const UART011_IMSC: u32 = 0x38;
/// Raw interrupt status
pub const UART011_RIS: u32 = 0x3c;
/// Masked interrupt status
pub const UART011_MIS: u32 = 0x40;
/// Interrupt clear register
pub const UART011_ICR: u32 = 0x44;
/// DMA control register
pub const UART011_DMACR: u32 = 0x48;
/// XON/XOFF control register
pub const ST_UART011_XFCR: u32 = 0x50;
/// XON1 register
pub const ST_UART011_XON1: u32 = 0x54;
/// XON2 register
pub const ST_UART011_XON2: u32 = 0x58;
/// XON1 register
pub const ST_UART011_XOFF1: u32 = 0x5C;
/// XON2 register
pub const ST_UART011_XOFF2: u32 = 0x60;
/// Integration test control register
pub const ST_UART011_ITCR: u32 = 0x80;
/// Integration test input register
pub const ST_UART011_ITIP: u32 = 0x84;
/// Autobaud control register
pub const ST_UART011_ABCR: u32 = 0x100;
/// Autobaud interrupt mask/clear register
pub const ST_UART011_ABIMSC: u32 = 0x15C;

// Constants for UART011 IFLS (Interrupt FIFO Level Select) register
/// Interrupt FIFO Level Select values for receive.
/// 1/8 FIFO level for receive
pub const UART011_IFLS_RX1_8: u32 = 0 << 3;
/// 2/8 FIFO level for receive
pub const UART011_IFLS_RX2_8: u32 = 1 << 3;
/// 4/8 FIFO level for receive
pub const UART011_IFLS_RX4_8: u32 = 2 << 3;
/// 6/8 FIFO level for receive
pub const UART011_IFLS_RX6_8: u32 = 3 << 3;
/// 7/8 FIFO level for receive
pub const UART011_IFLS_RX7_8: u32 = 4 << 3;

/// Interrupt FIFO Level Select values for transmit.
/// 1/8 FIFO level for transmit
pub const UART011_IFLS_TX1_8: u32 = 0 << 0;
/// 2/8 FIFO level for transmit
pub const UART011_IFLS_TX2_8: u32 = 1 << 0;
/// 4/8 FIFO level for transmit
pub const UART011_IFLS_TX4_8: u32 = 2 << 0;
/// 6/8 FIFO level for transmit
pub const UART011_IFLS_TX6_8: u32 = 3 << 0;
/// 7/8 FIFO level for transmit
pub const UART011_IFLS_TX7_8: u32 = 4 << 0;

/// Flag register bit definitions for UART.
/// Ring Indicator
pub const UART011_FR_RI: u32 = 0x100;
/// Transmit FIFO Empty
pub const UART011_FR_TXFE: u32 = 0x080;
/// Receive FIFO Full
pub const UART011_FR_RXFF: u32 = 0x040;
/// Transmit FIFO Full
pub const UART01X_FR_TXFF: u32 = 0x020;
/// Receive FIFO Empty
pub const UART01X_FR_RXFE: u32 = 0x010;
/// UART is busy
pub const UART01X_FR_BUSY: u32 = 0x008;
/// Data Carrier Detect
pub const UART01X_FR_DCD: u32 = 0x004;
/// Data Set Ready
pub const UART01X_FR_DSR: u32 = 0x002;
/// Clear To Send
pub const UART01X_FR_CTS: u32 = 0x001;

/// Combines the DCD, DSR, and CTS modem status flags.
pub const UART01X_FR_MODEM_ANY: u32 = UART01X_FR_DCD | UART01X_FR_DSR | UART01X_FR_CTS;

// ../linux_raspberrypi/include/linux/amba/serial.h
/// CTS hardware flow control.
pub const UART011_CR_CTSEN: u32 = 0x8000;
/// RTS hardware flow control.
pub const UART011_CR_RTSEN: u32 = 0x4000;
/// OUT2.
pub const UART011_CR_OUT2: u32 = 0x2000;
/// OUT1.
pub const UART011_CR_OUT1: u32 = 0x1000;
/// RTS.
pub const UART011_CR_RTS: u32 = 0x0800;
/// DTR.
pub const UART011_CR_DTR: u32 = 0x0400;
/// Receive enable.
pub const UART011_CR_RXE: u32 = 0x0200;
/// Transmit enable.
pub const UART011_CR_TXE: u32 = 0x0100;
/// Loopback enable.
pub const UART011_CR_LBE: u32 = 0x0080;
/// RTIE.
pub const UART010_CR_RTIE: u32 = 0x0040;
/// TIE.
pub const UART010_CR_TIE: u32 = 0x0020;
/// RIE.
pub const UART010_CR_RIE: u32 = 0x0010;
/// MSIE.
pub const UART010_CR_MSIE: u32 = 0x0008;
/// Oversampling factor.
pub const ST_UART011_CR_OVSFACT: u32 = 0x0008;
/// SIR low power mode.
pub const UART01X_CR_IIRLP: u32 = 0x0004;
/// SIR enable.
pub const UART01X_CR_SIREN: u32 = 0x0002;
/// UART enable.
pub const UART01X_CR_UARTEN: u32 = 0x0001;

/// TIOCM_LE.
pub const TIOCM_LE: u32 = 0x001;
/// TIOCM_DTR.
pub const TIOCM_DTR: u32 = 0x002;
/// TIOCM_RTS.
pub const TIOCM_RTS: u32 = 0x004;
/// TIOCM_ST.
pub const TIOCM_ST: u32 = 0x008;
/// TIOCM_SR.
pub const TIOCM_SR: u32 = 0x010;
/// TIOCM_CTS.
pub const TIOCM_CTS: u32 = 0x020;
/// TIOCM_CAR.
pub const TIOCM_CAR: u32 = 0x040;
/// TIOCM_RNG.
pub const TIOCM_RNG: u32 = 0x080;
/// TIOCM_DSR.
pub const TIOCM_DSR: u32 = 0x100;
/// TIOCM_CD (alias for TIOCM_CAR).
pub const TIOCM_CD: u32 = TIOCM_CAR;
/// TIOCM_RI (alias for TIOCM_RNG).
pub const TIOCM_RI: u32 = TIOCM_RNG;
/// TIOCM_OUT1.
pub const TIOCM_OUT1: u32 = 0x2000;
/// TIOCM_OUT2.
pub const TIOCM_OUT2: u32 = 0x4000;
/// TIOCM_LOOP.
pub const TIOCM_LOOP: u32 = 0x8000;

/// Overrun error interrupt mask.
pub const UART011_OEIM: u32 = 1 << 10;
/// Break error interrupt mask.
pub const UART011_BEIM: u32 = 1 << 9;
/// Parity error interrupt mask.
pub const UART011_PEIM: u32 = 1 << 8;
/// Framing error interrupt mask.
pub const UART011_FEIM: u32 = 1 << 7;
/// Receive timeout interrupt mask.
pub const UART011_RTIM: u32 = 1 << 6;
/// Transmit interrupt mask.
pub const UART011_TXIM: u32 = 1 << 5;
/// Receive interrupt mask.
pub const UART011_RXIM: u32 = 1 << 4;
/// DSR interrupt mask.
pub const UART011_DSRMIM: u32 = 1 << 3;
/// DCD interrupt mask.
pub const UART011_DCDMIM: u32 = 1 << 2;
/// CTS interrupt mask.
pub const UART011_CTSMIM: u32 = 1 << 1;
/// RI interrupt mask.
pub const UART011_RIMIM: u32 = 1 << 0;

/// Overrun error interrupt status
pub const UART011_OEIS: u32 = 1 << 10;
/// Break error interrupt status
pub const UART011_BEIS: u32 = 1 << 9;
/// Parity error interrupt status
pub const UART011_PEIS: u32 = 1 << 8;
/// Framing error interrupt status
pub const UART011_FEIS: u32 = 1 << 7;
/// Receive timeout interrupt status
pub const UART011_RTIS: u32 = 1 << 6;
/// Transmit interrupt status
pub const UART011_TXIS: u32 = 1 << 5;
/// Receive interrupt status
pub const UART011_RXIS: u32 = 1 << 4;
/// DSR interrupt status
pub const UART011_DSRMIS: u32 = 1 << 3;
/// DCD interrupt status
pub const UART011_DCDMIS: u32 = 1 << 2;
/// CTS interrupt status
pub const UART011_CTSMIS: u32 = 1 << 1;
/// RI interrupt status
pub const UART011_RIMIS: u32 = 1 << 0;

/// Overrun error in data register
pub const UART011_DR_OE: u32 = 1 << 11;
/// Break error in data register
pub const UART011_DR_BE: u32 = 1 << 10;
/// Parity error in data register
pub const UART011_DR_PE: u32 = 1 << 9;
/// Framing error in data register
pub const UART011_DR_FE: u32 = 1 << 8;

/// Stick parity select
pub const UART011_LCRH_SPS: u32 = 0x80;
/// Word length: 8 bits
pub const UART01X_LCRH_WLEN_8: u32 = 0x60;
/// Word length: 7 bits
pub const UART01X_LCRH_WLEN_7: u32 = 0x40;
/// Word length: 6 bits
pub const UART01X_LCRH_WLEN_6: u32 = 0x20;
/// Word length: 5 bits
pub const UART01X_LCRH_WLEN_5: u32 = 0x00;
/// Enable FIFOs
pub const UART01X_LCRH_FEN: u32 = 0x10;
/// Two stop bits select
pub const UART01X_LCRH_STP2: u32 = 0x08;
/// Even parity select
pub const UART01X_LCRH_EPS: u32 = 0x04;
/// Parity enable
pub const UART01X_LCRH_PEN: u32 = 0x02;
/// Send break
pub const UART01X_LCRH_BRK: u32 = 0x01;

/// UART registers for PL011.
#[repr(usize)]
pub enum Register {
    /// Data Register.
    RegDr,
    /// DMA Watermark Status Register.
    RegStDmawm,
    /// Timeout Status Register.
    RegStTimeout,
    /// Flag Register.
    RegFr,
    /// Receive Line Control Register.
    RegLcrhRx,
    /// Transmit Line Control Register.
    RegLcrhTx,
    /// Integer Baud Rate Register.
    RegIbrd,
    /// Fractional Baud Rate Register.
    RegFbrd,
    /// Control Register.
    RegCr,
    /// Interrupt FIFO Level Select Register.
    RegIfls,
    /// Interrupt Mask Set/Clear Register.
    RegImsc,
    /// Raw Interrupt Status Register.
    RegRis,
    /// Masked Interrupt Status Register.
    RegMis,
    /// Interrupt Clear Register.
    RegIcr,
    /// DMA Control Register.
    RegDmacr,
    /// Transmit FIFO Control Register.
    RegStXfcr,
    /// XON1 Character Register.
    RegStXon1,
    /// XON2 Character Register.
    RegStXon2,
    /// XOFF1 Character Register.
    RegStXoff1,
    /// XOFF2 Character Register.
    RegStXoff2,
    /// Integration Test Control Register.
    RegStItcr,
    /// Integration Test Input Register.
    RegStItip,
    /// Auto-Baud Control Register.
    RegStAbcr,
    /// Auto-Baud Interrupt Mask Set/Clear Register.
    RegStAbimsc,
    /// The size of the array, must be the last entry.
    RegArraySize,
}


/// The array size is determined by the number of registers defined in the `Register` enum.
pub const REG_ARRAY_SIZE: usize = Register::RegArraySize as usize;
/// Initializes and returns an array of register offsets for the UART.
pub const fn initialize_offsets() -> [u32; REG_ARRAY_SIZE] {
    let mut offsets = [0; REG_ARRAY_SIZE];
    offsets[Register::RegDr as usize] = UART01X_DR;
    offsets[Register::RegFr as usize] = UART01X_FR;
    offsets[Register::RegLcrhRx as usize] = UART011_LCRH;
    offsets[Register::RegLcrhTx as usize] = UART011_LCRH;
    offsets[Register::RegIbrd as usize] = UART011_IBRD;
    offsets[Register::RegFbrd as usize] = UART011_FBRD;
    offsets[Register::RegCr as usize] = UART011_CR;
    offsets[Register::RegIfls as usize] = UART011_IFLS;
    offsets[Register::RegImsc as usize] = UART011_IMSC;
    offsets[Register::RegRis as usize] = UART011_RIS;
    offsets[Register::RegMis as usize] = UART011_MIS;
    offsets[Register::RegIcr as usize] = UART011_ICR;
    offsets[Register::RegDmacr as usize] = UART011_DMACR;

    offsets
}

#[derive(Default, Copy, Clone)]
/// Represents the configuration data for the vendor.
pub struct VendorData {
    /// Interrupt FIFO Level Select
    pub ifls: u32,

    /// FIFO status indicating if it is busy
    pub fr_busy: u32,

    /// Data Set Ready status
    pub fr_dsr: u32,

    /// Clear To Send status
    pub fr_cts: u32,

    /// Ring Indicator status
    pub fr_ri: u32,

    /// Inverse FIFO control
    pub inv_fr: u32,

    /// Indicates if 32-bit access is allowed
    pub access_32b: bool,

    /// Indicates if oversampling is enabled
    pub oversampling: bool,

    /// DMA threshold configuration
    pub dma_threshold: bool,

    /// Workaround for Clear To Send event
    pub cts_event_workaround: bool,

    /// Indicates if this feature is always enabled
    pub always_enabled: bool,

    /// Options for fixing fixed functionality
    pub fixed_options: bool,
}

/// 定义一个代表 dma_chan 的结构体
pub struct DmaChan;

// 定义 dma_filter 类型
type DmaFilter = Option<unsafe extern "C" fn(chan: *mut DmaChan, filter_param: *mut c_void) -> bool>;

// 定义 init 和 exit 函数类型
type InitFunc = Option<extern "C" fn()>;
type ExitFunc = Option<extern "C" fn()>;

/// Structure representing PL011-specific data for the AMBA device.
pub struct AmbaPl011Data {
    /// Function pointer for DMA filter.
    pub dma_filter: DmaFilter,
    /// Parameter for DMA receive.
    pub dma_rx_param: Option<NonNull<c_void>>,
    /// Parameter for DMA transmit.
    pub dma_tx_param: Option<NonNull<c_void>>,
    /// Flag indicating if DMA receive polling is enabled.
    pub dma_rx_poll_enable: bool,
    /// Rate of DMA receive polling.
    pub dma_rx_poll_rate: u32,
    /// Timeout for DMA receive polling.
    pub dma_rx_poll_timeout: u32,
    /// Initialization function for the device.
    pub init: InitFunc,
    /// Exit function for the device.
    pub exit: ExitFunc,
}


/// Selects the default pin control state.
#[cfg(feature = "pm")]
extern "C" {
    pub fn pinctrl_pm_select_default_state(dev: &Device) -> i32;
}

/// Selects the sleep pin control state.
#[cfg(feature = "pm")]
extern "C" {
    pub fn pinctrl_pm_select_sleep_state(dev: &Device) -> i32;
}

/// Selects the idle pin control state.
#[cfg(feature = "pm")]
extern "C" {
    pub fn pinctrl_pm_select_idle_state(dev: &Device) -> i32;
}

/// No-op for default pin control state.
#[cfg(not(feature = "pm"))]
pub fn pinctrl_pm_select_default_state(_dev: &Device) -> i32 {
    0
}

/// No-op for sleep pin control state.
#[cfg(not(feature = "pm"))]
pub fn pinctrl_pm_select_sleep_state(_dev: &Device) -> i32 {
    0
}

/// No-op for idle pin control state.
#[cfg(not(feature = "pm"))]
pub fn pinctrl_pm_select_idle_state(_dev: &Device) -> i32 {
    0
}



