// SPDX-License-Identifier: GPL-2.0

//! AMBA串口驱动程序（PL011）。
//!
//! 基于由ARM Ltd/Deep Blue Solutions Ltd编写的C驱动程序。

use kernel::{
    bindings,  // 内核绑定
    prelude::*,  // 导入内核预导入模块
    amba,  // AMBA（高级微控制器总线架构）模块
    clk::Clk,  // 时钟模块
    device,  // 设备模块
    device::RawDevice,
    driver,
    new_device_data,
    error::{code::*, Result},  // 错误处理模块，包括错误代码和结果类型
    module_amba_driver,  // AMBA驱动模块
    c_str,  // C风格字符串模块
    str::CString,
    irq,
    io_mem::IoMem,  // IO内存模块
    types::ForeignOwnable,
    delay::coarse_sleep,
    sync::UniqueArc,
    serial::{  // 串口模块
               uart_console::{Console, ConsoleOps, flags},  // 串口控制台相关模块，包括控制台、控制台操作和标志
               uart_driver::UartDriver,  // 串口驱动模块
               uart_port::{UartPort, PortRegistration, UartPortOps},  // 串口端口模块，包括串口端口和端口注册
               pl011_config::*,  // PL011配置模块
    },
};
use core::{
    time::Duration,
    fmt::Write,
    pin::{pin, Pin},
    ptr,
    ffi::{c_void, c_uint, c_uchar},
};
use alloc::boxed::Box;

// UART寄存器的大小
const UART_SIZE: usize = 0x200;
// 内存映射IO类型
const UPIO_MEM: u32 = 2;
const UPIO_MEM32: u32 = 3;

// 启动自动配置标志
pub const UPF_BOOT_AUTOCONF: u64 = 1_u64 << 28;

// UART端口数量
pub(crate) const UART_NR: usize = 14;
// AMBA主设备号
const AMBA_MAJOR: i32 = 204;
// AMBA次设备号
const AMBA_MINOR: i32 = 64;


const TIOCSER_TEMT: usize = 0x01;
const UPSTAT_AUTORTS: bindings::upstat_t = 1 << 2;
const UPSTAT_AUTOCTS: bindings::upstat_t = 1 << 3;
const SER_RS485_ENABLED: u32 = 1 << 0;
const SER_RS485_RX_DURING_TX: u32 = 1 << 4;
const SER_RS485_RTS_ON_SEND: u32 = 1 << 1;
const MAX_UDELAY_MS: u64 = 5;
const AMBA_ISR_PASS_LIMIT: u32 = 256;
const UART_DR_ERROR: u32 = UART011_DR_OE | UART011_DR_BE | UART011_DR_PE | UART011_DR_FE;
const TASK_INTERRUPTIBLE: u32 = 0x00000001;
const PAGE_SIZE: i32 = 1 << PAGE_SHIFT;
const PAGE_SHIFT: i32 = 12;
const UART_XMIT_SIZE: i32 = PAGE_SIZE;
const WAKEUP_CHARS: i32 = 256;
const SER_RS485_RTS_AFTER_SEND: u32 = 1 << 2;
const UART_DUMMY_DR_RX: u32 = 1 << 16;
const UPF_HARDPPS_CD: bindings::upf_t = bindings::ASYNC_HARDPPS_CD as bindings::upf_t;
const PORT_AMBA: c_uint = 32;
const UART_CONFIG_TYPE: i32 = 1 << 0;
const CONFIG_OF: i32 = 1;

// 设备名称
const DEV_NAME: &CStr = c_str!("ttyAMA");
// 驱动程序名称
const DRIVER_NAME: &CStr = c_str!("ttyAMA");

#[cfg(target_arch = "aarch64")]
#[inline(always)]
fn cpu_relax() {
    unsafe { core::arch::asm!("yield") }
} // 未知!!!!!

static PL011_STD_OFFSETS: [u32; REG_ARRAY_SIZE] = initialize_offsets();


// 静态结构体，包含所有的UART端口
pub(crate) static mut PORTS: [Option<UartPort>; UART_NR] = [None; UART_NR];

// AMBA UART控制台的静态结构体
static AMBA_CONSOLE: Console = {
    let name: [i8; 16usize] = [
        't' as _, 't' as _, 'y' as _, 'A' as _, 'M' as _, 'A' as _,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];
    Console::new::<Pl011Console>(name, UART_DRIVER.as_ptr())
        .with_config(
            (flags::CON_PRINTBUFFER | flags::CON_ANYTIME) as _,
            -1, 0, 0, 0, 0, 0,
        )
};

// UART驱动程序的静态结构体
pub(crate) static UART_DRIVER: UartDriver = UartDriver::new(
    &THIS_MODULE,
    DRIVER_NAME,
    DEV_NAME,
    &AMBA_CONSOLE,
).with_config(
    AMBA_MAJOR,
    AMBA_MINOR,
    UART_NR as _,
);

// pl011控制台的结构体
struct Pl011Console;

impl Pl011Console {
    // 向 UART 发送一个字符
    extern "C" fn pl011_console_putchar(port_ptr: *mut bindings::uart_port, ch: c_uchar) {
        let uap = unsafe { &UartPort::from_raw(port_ptr) };
        while AmbaPl011Pops::pl011_read(uap, Register::RegFr as usize) & UART01X_FR_TXFF != 0 {
            cpu_relax(); // CPU 进入低功耗状态  // 未知!!!!!  不确定
        }
        AmbaPl011Pops::pl011_write(ch as u32, uap, Register::RegDr as usize); // 将字符写入数据寄存器
    }

    fn pl011_console_get_options(uap: &UartPort, baud: &mut i32, parity: &mut i32, bits: &mut i32) {
        if AmbaPl011Pops::pl011_read(uap, Register::RegCr as usize) & UART01X_CR_UARTEN != 0 {  // 检查 UART 是否启用
            let lcr_h = AmbaPl011Pops::pl011_read(uap, Register::RegLcrhTx as usize); // 读取行控制寄存器

            *parity = 'n' as i32;
            if lcr_h & UART01X_LCRH_PEN != 0 { // 检查是否启用奇偶校验
                if lcr_h & UART01X_LCRH_EPS != 0 {
                    *parity = 'e' as i32; // 启用偶校验
                } else {
                    *parity = 'o' as i32; // 启用奇校验
                }

                if (lcr_h & 0x60) == UART01X_LCRH_WLEN_7 {
                    *bits = 7; // 数据位为7
                } else {
                    *bits = 8; // 数据位为8
                }

                let ibrd = AmbaPl011Pops::pl011_read(uap, Register::RegIbrd as usize); // 读取整数波特率除数
                let fbrd = AmbaPl011Pops::pl011_read(uap, Register::RegFbrd as usize); // 读取小数波特率除数

                let uap_ptr = unsafe { &*uap.as_ptr() };
                let mut pl011_data: Box<PL011Data> = unsafe {
                    <Box<PL011Data> as ForeignOwnable>::from_foreign(uap_ptr.private_data)
                };
                *baud = (uap_ptr.uartclk * 4 / (64 * ibrd + fbrd)) as i32; // 计算波特率

                if pl011_data.vendor.oversampling { // 如果启用过采样
                    if AmbaPl011Pops::pl011_read(uap, Register::RegCr as usize) & ST_UART011_CR_OVSFACT != 0 {
                        *baud *= 2; // 波特率翻倍
                    }
                }
            }
        }
    }

    fn pl011_console_setup(co: &Console, options: *mut i8) -> Result<i32> {
        let mut baud: i32 = 38400;  // 默认波特率
        let mut bits: i32 = 8; // 默认数据位
        let mut parity: i32 = 'n' as i32; // 默认无奇偶校验
        let mut flow: i32 = 'n' as i32; // 默认无流控

        let mut co_ptr = unsafe { &mut *co.as_ptr() };

        /*
         * 检查是否指定了无效的 UART 号，
         * 如果是，则搜索第一个可用的具有控制台支持的端口。
         */
        if co_ptr.index >= UART_NR as i16 {
            co_ptr.index = 0;
        }

        if unsafe { &PORTS[co_ptr.index as usize] }.is_none() {
            return Err(ENODEV);  // 如果端口不存在，返回 -ENODEV
        }
        let uap = &unsafe { &PORTS[co_ptr.index as usize] }.unwrap(); // 获取指定索引的端口
        let mut uap_ptr = unsafe { &mut *uap.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(uap_ptr.private_data)
        };

        /* 允许引脚复用并配置 */
        // pinctrl_pm_select_default_state(uap->port.dev);  未知!!!!!  ../linux_raspberrypi/include/linux/pinctrl/consumer.h

        let dev = unsafe { device::Device::new(uap_ptr.dev) };
        let clk = dev.clk_get().unwrap();
        clk.prepare_enable()?;  //准备时钟

        // 未知!!!!! 怎么实现
        // if (dev_get_platdata(uap->port.dev)) {
        //     struct amba_pl011_data *plat;
        //
        //     plat = dev_get_platdata(uap->port.dev);
        //     if (plat->init)
        //     plat->init(); // 调用平台数据中的初始化函数
        // }

        uap_ptr.uartclk = clk.get_rate() as u32; // 获取 UART 时钟频率

        if pl011_data.vendor.fixed_options {
            baud = pl011_data.fixed_baud as i32; // 如果固定波特率选项启用，使用固定波特率
        } else {
            if !options.is_null() {
                unsafe {
                    bindings::uart_parse_options(options,
                                                 &mut baud as *mut _,
                                                 &mut parity as *mut _,
                                                 &mut bits as *mut _,
                                                 &mut flow as *mut _); // 解析命令行选项
                }
            } else {
                Self::pl011_console_get_options(uap, &mut baud, &mut parity, &mut bits); // 获取控制台选项
            }
        }

        let mut co_ptr = unsafe { &mut *co.as_ptr() };
        let ret = unsafe { bindings::uart_set_options(uap_ptr, co_ptr, baud, parity, bits, flow) }; // 设置 UART 选项

        if ret == 0 {
            Ok(0)
        } else {
            Err(ENODEV)
        }
    }
}

// 实现`Pl011Console`的操作
#[vtable]
impl ConsoleOps for Pl011Console {
    type Data = ();

    fn console_write(co: &Console, s: *const i8, count: u32) {
        let mut co_ptr = unsafe { &mut *co.as_ptr() };
        // 获取 UART 端口
        let uap = &unsafe { &PORTS[co_ptr.index as usize] }.unwrap();
        let mut uap_ptr = unsafe { &mut *uap.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(uap_ptr.private_data)
        };

        let mut locked: i32 = 1;
        let mut old_cr = 0;
        let mut new_cr;
        // let flags: usize;

        let dev = unsafe { device::Device::new(uap_ptr.dev) };
        // 启用时钟
        let _ = dev.clk_get().unwrap().prepare_enable();

        // 保存中断状态
        // local_irq_save(flags);  // 未知!!!!!   ../linux_raspberrypi/include/linux/irqflags.h  +220

        let union_ptr = &uap_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问

        if uap_ptr.sysrq != 0 {
            locked = 0;
        } else if unsafe { bindings::oops_in_progress != 0 } {
            // 如果有故障发生，尝试加锁
            locked = unsafe { bindings::_raw_spin_trylock(&(*union_ptr).rlock as *const _ as *mut _) };
        } else {
            unsafe { bindings::_raw_spin_lock(&(*union_ptr).rlock as *const _ as *mut _) };
        }

        /*
         * 先保存控制寄存器的值，然后禁用中断
         */
        if !pl011_data.vendor.always_enabled { // 如果设备不总是启用
            old_cr = AmbaPl011Pops::pl011_read(uap, Register::RegCr as usize); // 读取当前控制寄存器的值
            new_cr = old_cr & !UART011_CR_CTSEN; // 禁用 CTS
            new_cr |= UART01X_CR_UARTEN | UART011_CR_TXE; // 启用 UART 和发送
            AmbaPl011Pops::pl011_write(new_cr, uap, Register::RegCr as usize); // 写入新的控制寄存器值
        }

        unsafe { bindings::uart_console_write(uap.as_ptr(), s, count, Some(Self::pl011_console_putchar)) }; // 调用通用控制台写入函数

        /*
         * 最后，等待发送器变为空，然后恢复控制寄存器的值。
         * 允许特征寄存器位被反转以解决错误。
         */
        while (AmbaPl011Pops::pl011_read(uap, Register::RegFr as usize) ^ pl011_data.vendor.inv_fr) & pl011_data.vendor.fr_busy != 0 {
            cpu_relax(); // 等待发送完成  // 未知!!!!!  不确定
        }

        if !pl011_data.vendor.always_enabled {
            AmbaPl011Pops::pl011_write(old_cr, uap, Register::RegCr as usize); // 恢复控制寄存器的值
        }

        // 恢复中断状态
        // local_irq_restore(flags); // 未知!!!!!   ../linux_raspberrypi/include/linux/irqflags.h  +227

        // clk_disable(uap->clk); // 禁用时钟  // 未知!!!!! 好像drop自动实现
        pr_info!("console_write ok");
    }

    fn console_read(_co: &Console, _s: *mut i8, _count: u32) -> Result<i32> {
        pr_info!("console_read ok");
        Ok(0)
    }

    fn console_match(
        co: &Console,
        name: *mut i8,
        idx: i32,
        options: *mut i8,
    ) -> Result<i32>
    {
        let mut iotype: c_uchar = 0;
        let mut addr: bindings::resource_size_t = 0;

        /*
         * 受 Qualcomm Technologies QDF2400 E44 勘误影响的系统有一个不同的控制台名称，
         * 所以需要检查该名称。实际的勘误实现发生在探测函数中。
         */

        let name_cstr = unsafe { CStr::from_char_ptr(name) };
        let name_str = name_cstr.to_str()?;
        if name_str != "qdf2400_e44" && name_str != "pl011" {  //未知!!!!!  不确定
            return Err(ENODEV);  // 如果名称不匹配，返回 -ENODEV
        }

        // 如果解析早期控制台参数失败，返回 -ENODEV
        if unsafe {
            bindings::uart_parse_earlycon(
                options,
                &mut iotype as *mut _,
                &mut addr as *mut _,
                options as *mut *mut _,
            ) != 0
        } {
            return Err(ENODEV); // 如果解析早期控制台参数失败，返回 -ENODEV
        }

        if u32::from(iotype) != UPIO_MEM && u32::from(iotype) != UPIO_MEM32 {
            return Err(ENODEV); // 如果 IO 类型不匹配，返回 -ENODEV
        }

        /* 尝试匹配命令行中指定的端口 */
        for i in 0..UART_NR {
            if unsafe { PORTS[i] }.is_none() {
                continue; // 如果端口为空，继续下一次循环
            }

            let port = &unsafe { &PORTS[i] }.unwrap(); // 获取 UART 端口
            let mut port_ptr = unsafe { &mut *port.as_ptr() };

            if port_ptr.mapbase != addr {
                continue; // 如果地址不匹配，继续下一次循环
            }

            let mut co_ptr = unsafe { &mut *co.as_ptr() };
            co_ptr.index = i as i16; // 设置控制台索引
            port_ptr.cons = co_ptr; // 将控制台关联到端口
            return Self::pl011_console_setup(co, options); // 调用控制台设置函数
        }

        pr_info!("console_match ok");
        Err(ENODEV)  // 如果没有匹配成功，返回 -ENODEV
    }

    fn console_device(_co: &Console, _index: *mut i8) -> *mut bindings::tty_driver {
        pr_info!("console_device ok");
        todo!()
    }
}

// VENDOR数据的静态结构体
pub(crate) static VENDOR_DATA: VendorData = VendorData {
    reg_offset: PL011_STD_OFFSETS,
    ifls: UART011_IFLS_RX4_8 | UART011_IFLS_TX4_8,
    fr_busy: UART01X_FR_BUSY,
    fr_dsr: UART01X_FR_DSR,
    fr_cts: UART01X_FR_CTS,
    fr_ri: UART011_FR_RI,
    inv_fr: 0,
    access_32b: false,
    oversampling: false,
    dma_threshold: false,
    cts_event_workaround: false,
    always_enabled: false,
    fixed_options: false,
};

// PL011数据的结构体
#[derive(Default, Copy, Clone)]
struct PL011Data {
    reg_offset: [u32; REG_ARRAY_SIZE],
    im: u32,
    old_status: u32,
    fifosize: u32,
    fixed_baud: u32,
    type_: [u8; 12],
    vendor: VendorData,
    rs485_tx_started: bool,
    rs485_tx_drain_interval: u64,
}

// PL011资源的结构体
struct PL011Resources {
    base: IoMem<UART_SIZE>,
    parent_irq: u32,
}

struct Pl011Handler {}

impl irq::Handler for Pl011Handler {
    type Data = Box<UartPort>;

    fn handle_irq(data: &UartPort) -> irq::Return {
        let port_ptr = unsafe { &*data.as_ptr() };
        let mut flags: u64 = 0;
        let mut handled = false;
        let mut pass_counter = AMBA_ISR_PASS_LIMIT;
        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问

        flags = unsafe { bindings::_raw_spin_lock_irqsave(&(*union_ptr).rlock as *const _ as *mut _) };

        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };
        // 读取当前的中断状态并掩码掉未启用的中断
        let mut status = AmbaPl011Pops::pl011_read(data, Register::RegRis as usize) & pl011_data.im;
        if status != 0 {
            loop {
                // 检查并应用 CTS 事件的临时解决方案
                AmbaPl011Pops::check_apply_cts_event_workaround(data);
                // 清除处理过的中断
                AmbaPl011Pops::pl011_write(status & !(UART011_TXIS | UART011_RTIS |
                    UART011_RXIS), data, Register::RegIcr as usize);

                // 处理接收字符的中断
                if status & (UART011_RTIS | UART011_RXIS) != 0 {
                    AmbaPl011Pops::pl011_rx_chars(data);
                }

                // 处理调制解调器状态变化的中断
                if status & (UART011_DSRMIS | UART011_DCDMIS |
                    UART011_CTSMIS | UART011_RIMIS) != 0 {
                    AmbaPl011Pops::pl011_modem_status(data);
                }

                // 处理发送字符的中断
                if status & UART011_TXIS != 0 {
                    AmbaPl011Pops::pl011_tx_chars(data, true);
                }

                // 如果处理循环次数达到限制，则退出循环
                if pass_counter == 0 {
                    break;
                }

                pass_counter -= 1;

                // 重新读取中断状态以检查是否有新的中断发生
                status = AmbaPl011Pops::pl011_read(data, Register::RegRis as usize) & pl011_data.im;

                if status == 0 {
                    break;
                }
            }
            handled = true;
        }

        unsafe {
            // 恢复之前保存的中断状态，并退出临界区
            bindings::_raw_spin_unlock_irqrestore(&(*union_ptr).rlock as *const _ as *mut _, flags);
        }

        if handled {
            return irq::Return::Handled;
        }
        irq::Return::None
    }
}

struct AmbaPl011Pops;

impl AmbaPl011Pops {
    fn pl011_reg_to_offset(reg: usize) -> usize {
        PL011_STD_OFFSETS[reg] as usize
    }

    fn pl011_read(port: &UartPort, reg: usize) -> u32 {
        let port_ptr = unsafe { &*port.as_ptr() };
        let addr = port_ptr.membase.wrapping_add(Self::pl011_reg_to_offset(reg));
        if u32::from(port_ptr.iotype) == UPIO_MEM32 {
            unsafe { bindings::readl_relaxed(addr as _) }
        } else {
            unsafe { bindings::readw_relaxed(addr as _) as u32 }
        }
    }

    /// pl011寄存器写入函数
    fn pl011_write(val: u32, port: &UartPort, reg: usize) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let addr = port_ptr.membase.wrapping_add(Self::pl011_reg_to_offset(reg));

        if u32::from(port_ptr.iotype) == UPIO_MEM32 {
            unsafe { bindings::writel_relaxed(val as _, addr as _) };
        } else {
            unsafe { bindings::writew_relaxed(val as _, addr as _) };
        }
    }

    fn pl011_rs485_tx_start(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut cr: u32 = 0;

        cr = Self::pl011_read(port, Register::RegCr as usize);
        cr |= UART011_CR_TXE;

        if !(port_ptr.rs485.flags & SER_RS485_RX_DURING_TX) != 0 {
            cr &= !UART011_CR_RXE;
        }

        if port_ptr.rs485.flags & SER_RS485_RTS_ON_SEND != 0 {
            cr &= !UART011_CR_RTS;
        } else {
            cr |= UART011_CR_RTS;
        }

        Self::pl011_write(cr, port, Register::RegCr as usize);

        let mut de: u64 = port_ptr.rs485.delay_rts_before_send as u64;
        if de > 0 {
            if de <= MAX_UDELAY_MS {
                coarse_sleep(Duration::from_millis(de));
            } else {
                while de > 0 {
                    coarse_sleep(Duration::from_millis(1));
                    de -= 1;
                }
            }
        }

        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };
        pl011_data.rs485_tx_started = true;
    }

    fn pl011_start_tx_pio(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        if Self::pl011_tx_chars(port, false) {
            pl011_data.im |= UART011_TXIM;
            Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);
        }
    }

    fn pl011_stop_rx(port: &UartPort, pl011_data: &mut PL011Data) {
        // 清除中断掩码寄存器 (IMSC) 中的接收相关中断位
        pl011_data.im &= !(UART011_RXIM | UART011_RTIM | UART011_FEIM |
            UART011_PEIM | UART011_BEIM | UART011_OEIM);

        // 将修改后的中断掩码寄存器值写回到硬件寄存器中，以禁用接收中断
        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);
    }

    fn pl011_hwinit(port: &UartPort, pl011_data: &mut PL011Data) -> Result {
        // 可选地启用引脚复用并配置
        /* pinctrl_pm_select_default_state(port->dev); */  // 未知!!!!! 怎么实现？ ../linux_raspberrypi/include/linux/pinctrl/consumer.h

        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        let dev = unsafe { device::Device::new(port_ptr.dev) };

        let clk = dev.clk_get().unwrap();

        //尝试启用时钟
        clk.prepare_enable()?;

        // 获取时钟频率并保存到 uartclk 中
        port_ptr.uartclk = clk.get_rate() as u32;
        pr_info!("====Serial: uartclk is {}", port_ptr.uartclk);

        // 清除未决的错误和接收中断
        Self::pl011_write(UART011_OEIS | UART011_BEIS | UART011_PEIS |
                              UART011_FEIS | UART011_RTIS | UART011_RXIS,
                          port, Register::RegIcr as usize);

        // 保存中断使能掩码，并启用 RX 中断以防万一用于 NMI 入口
        pl011_data.im = Self::pl011_read(port, Register::RegImsc as usize);
        Self::pl011_write(UART011_RTIM | UART011_RXIM, port, Register::RegImsc as usize);

        // 未知!!!!! 怎么实现
        // 如果有平台特定的数据，执行其初始化
        // if (dev_get_platdata(uap->port.dev)) {
        //     struct amba_pl011_data *plat;
        //
        //     plat = dev_get_platdata(uap->port.dev);
        //     if (plat->init)
        //     plat->init();
        // }

        Ok(())
    }

    fn pl011_fifo_to_tty(port: &UartPort) -> i32 {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        let mut fifotaken = 0;
        while fifotaken != 255 {
            let status = Self::pl011_read(port, Register::RegFr as usize);
            if status & UART01X_FR_RXFE != 0 {
                break;
            }

            let mut ch = Self::pl011_read(port, Register::RegDr as usize);
            let mut flag = bindings::TTY_NORMAL;
            port_ptr.icount.rx += 1;

            if ch & UART_DR_ERROR != 0 {
                if ch & UART011_DR_BE != 0 {
                    ch &= !(UART011_DR_FE | UART011_DR_PE);
                    port_ptr.icount.brk += 1;
                    if port.uart_handle_break() {
                        continue;
                    }
                } else if ch & UART011_DR_PE != 0 {
                    port_ptr.icount.parity += 1;
                } else if ch & UART011_DR_FE != 0 {
                    port_ptr.icount.frame += 1;
                }

                if ch & UART011_DR_OE != 0 {
                    port_ptr.icount.overrun += 1;
                }

                ch &= port_ptr.read_status_mask;

                if ch & UART011_DR_BE != 0 {
                    flag = bindings::TTY_BREAK;
                } else if ch & UART011_DR_PE != 0 {
                    flag = bindings::TTY_PARITY;
                } else if ch & UART011_DR_FE != 0 {
                    flag = bindings::TTY_FRAME;
                }
            }

            let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!! 转化问题疑问
            unsafe { bindings::_raw_spin_unlock(&(*union_ptr).rlock as *const _ as *mut _); }

            let sysrq = port.uart_handle_sysrq_char(ch as u8 & 255);


            unsafe {
                bindings::_raw_spin_lock(&(*union_ptr).rlock as *const _ as *mut _);
            }

            if !sysrq {
                unsafe {
                    bindings::uart_insert_char(
                        port.as_ptr(),
                        ch,
                        UART011_DR_OE,
                        ch as u8,
                        flag as u8,
                    );
                }
            }
        }
        return fifotaken;
    }

    fn check_apply_cts_event_workaround(port: &UartPort) {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };
        if !pl011_data.vendor.cts_event_workaround {
            return;
        }

        // 确保所有位都被解锁
        Self::pl011_write(0x00, port, Register::RegIcr as usize);

        // 解决方法：在写 1 清除（W1C）之前引入 26 纳秒（1 个 UART 时钟）的延迟；
        // 单次 APB 访问会引入 2 个 PCLK（133.12MHz）的延迟，
        // 因此添加 2 次虚拟读取。
        Self::pl011_read(port, Register::RegIcr as usize);
        Self::pl011_read(port, Register::RegIcr as usize);
    }

    fn pl011_rx_chars(port: &UartPort) {
        Self::pl011_fifo_to_tty(port);

        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        unsafe { bindings::_raw_spin_unlock(&(*union_ptr).rlock as *const _ as *mut _); }

        // unsafe { bindings::tty_flip_buffer_push((*port_ptr.state).port) };  // 未知!!!!!  怎么实现

        unsafe {
            bindings::_raw_spin_lock(&(*union_ptr).rlock as *const _ as *mut _);
        }
    }

    fn pl011_modem_status(port: &UartPort) {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        // 读取调制解调器状态寄存器的值，并与调制解调器状态掩码进行按位与操作
        let status = Self::pl011_read(port, Register::RegFr as usize) & UART01X_FR_MODEM_ANY;

        // 计算当前状态与之前保存的状态之间的变化
        let delta = status ^ pl011_data.old_status;

        // 更新保存的状态为当前状态
        pl011_data.old_status = status;

        // 如果没有状态变化，直接返回
        if !delta != 0 {
            return;
        }

        // 如果数据载体检测(DCD)信号有变化，处理DCD信号变化
        if delta & UART01X_FR_DCD != 0 {
            unsafe { bindings::uart_handle_dcd_change(port.as_ptr(), status & UART01X_FR_DCD != 0) };
        }

        // 如果数据集就绪(DSR)信号有变化，更新DSR信号计数
        if delta & pl011_data.vendor.fr_dsr != 0 {
            port_ptr.icount.dsr += 1;
        }

        // 如果清除发送(CTS)信号有变化，处理CTS信号变化
        if delta & pl011_data.vendor.fr_cts != 0 {
            unsafe { bindings::uart_handle_cts_change(port.as_ptr(), status & pl011_data.vendor.fr_cts != 0) };
        }
        unsafe {
            let wait_queue: *mut bindings::wait_queue_head = &mut (*port_ptr.state).port.delta_msr_wait as *mut _;
            let key: *mut c_void = ptr::null_mut();
            // 唤醒等待调制解调器状态变化的进程

            bindings::__wake_up(wait_queue, TASK_INTERRUPTIBLE, 1, key);
        }
    }

    fn pl011_tx_char(port: &UartPort, c: u8, from_irq: bool) -> bool {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        // 检查如果不是从中断上下文调用，并且 UART 的发送 FIFO 已满，则无法发送字符
        if !from_irq && (Self::pl011_read(port, Register::RegFr as usize) & UART01X_FR_TXFF != 0) {
            return false; // 无法发送字符
        }

        // 将字符写入 UART 数据寄存器
        Self::pl011_write(c as _, port, Register::RegDr as usize);

        // 内存屏障，确保写操作按顺序执行
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);  // 未知!!!!!  需理解

        // 更新发送计数
        port_ptr.icount.tx += 1;

        // 返回 true 表示字符已成功发送
        return true;
    }

    fn pl011_stop_tx(port: &UartPort) {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        // 清除发送中断使能位
        pl011_data.im &= !UART011_TXIM;

        // 将更新后的中断屏蔽寄存器值写回到硬件寄存器
        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);
    }

    fn pl011_tx_chars(port: &UartPort, from_irq: bool) -> bool {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };
        // 获取指向 UART 发送缓冲区的指针
        let mut xmit = unsafe { (*port_ptr.state).xmit };

        // 计算一次传输的最大字符数，通常为 FIFO 大小的一半
        let mut count = pl011_data.fifosize >> 1;

        // 如果有要立即发送的字符（x_char），优先发送它
        if port_ptr.x_char != 0 {
            if !Self::pl011_tx_char(port, port_ptr.x_char as _, from_irq) {
                return true;  // 发送成功，返回 true
            }

            port_ptr.x_char = 0;  // 清空 x_char
            count -= 1;  // 减少发送计数
        }

        fn uart_circ_empty(circ: &bindings::circ_buf) -> bool {
            circ.head == circ.tail
        }

        fn uart_circ_chars_pending(circ: &bindings::circ_buf) -> i32 {
            (circ.head - circ.tail) & (UART_XMIT_SIZE - 1)
        }

        // 如果缓冲区为空或 UART 停止传输，停止发送
        if uart_circ_empty(&xmit) || port.uart_tx_stopped() {
            Self::pl011_stop_tx(port);
            return false;
        }

        // 循环发送字符，直到缓冲区为空或达到发送计数限制
        loop {
            // 如果来自中断且发送计数为 0，退出循环
            if from_irq && count == 0 {
                break;
            }
            count -= 1;

            // 如果来自中断且发送计数为 0 且 FIFO 已满，退出循环
            if from_irq && count == 0 &&
                Self::pl011_read(port, Register::RegFr as usize) & UART01X_FR_TXFF != 0 {
                break;
            }

            let c = unsafe { *xmit.buf.wrapping_add(xmit.tail as usize) }; // 获取字符
            // 发送一个字符，如果发送失败，退出循环
            if !Self::pl011_tx_char(port, c.try_into().unwrap(), from_irq) {
                break;
            }

            // 更新环形缓冲区的尾部指针
            xmit.tail = (xmit.tail + 1) & (UART_XMIT_SIZE - 1);

            // 缓冲区为空退出循环
            if uart_circ_empty(&xmit) {
                break;
            }
        }

        // 如果缓冲区中待发送的字符数少于 WAKEUP_CHARS，唤醒写进程
        if uart_circ_chars_pending(&xmit) < WAKEUP_CHARS {
            unsafe { bindings::uart_write_wakeup(port.as_ptr()) };
        }

        // 如果缓冲区为空，停止发送
        if uart_circ_empty(&xmit) {
            Self::pl011_stop_tx(port);
            return false;
        }

        // 返回 true 表示仍有字符待发送
        return true;
    }

    fn request_irq(irq: u32, data: Box<UartPort>) -> Result<irq::Registration<Pl011Handler>> {
        irq::Registration::try_new(irq, data, irq::flags::SHARED, fmt!("uart-pl011"))
    }

    fn pl011_allocate_irq(port: &UartPort, pl011_data: &mut PL011Data) -> Result<irq::Registration<Pl011Handler>> {
        let port_ptr = unsafe { &*port.as_ptr() };

        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);
        Self::request_irq(port_ptr.irq, Box::try_new(*port)?)
    }

    fn pl011_enable_interrupts(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };
        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        // 进入临界区，保存当前中断状态并禁止中断，以确保线程安全
        let flags = unsafe { bindings::_raw_spin_lock_irqsave(&(*union_ptr).rlock as *const _ as *mut _) };

        Self::pl011_write(UART011_RTIS | UART011_RXIS, port, Register::RegIcr as usize);

        for i in 0..pl011_data.fifosize * 2 {
            // 读取状态寄存器并检查接收 FIFO 是否为空
            if Self::pl011_read(port, Register::RegFr as usize) & UART01X_FR_RXFE != 0 {
                break;
            }

            // 读取数据寄存器
            Self::pl011_read(port, Register::RegDr as usize);
        }

        pl011_data.im = UART011_RTIM | UART011_RXIM;
        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);

        // 恢复之前保存的中断状态，并退出临界区
        unsafe { bindings::_raw_spin_unlock_irqrestore(&(*union_ptr).rlock as *const _ as *mut _, flags) };
    }

    fn pl011_disable_interrupts(port: &UartPort, pl011_data: &mut PL011Data) {
        let port_ptr = unsafe { &*port.as_ptr() };

        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        // 加锁以保护对 CR 寄存器的访问
        unsafe { bindings::_raw_spin_lock(&(*union_ptr).rlock as *const _ as *mut _); }

        pl011_data.im = 0;
        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);
        Self::pl011_write(0xffff, port, Register::RegIcr as usize);

        // 解锁
        unsafe { bindings::_raw_spin_unlock(&(*union_ptr).rlock as *const _ as *mut _); }
    }

    fn pl011_rs485_tx_stop(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };
        let max_tx_drain_iters: i32 = (port_ptr.fifosize * 2).try_into().unwrap();
        let mut i = 0;

        while !Self::tx_empty(port) != 0 {
            if i > max_tx_drain_iters {
                unsafe { dev_warn!(device::Device::new(port_ptr.dev), "timeout while draining hardware tx queue\n"); }
                break;
            }

            coarse_sleep(Duration::from_nanos(pl011_data.rs485_tx_drain_interval * 1000));
            i += 1;
        }

        if port_ptr.rs485.delay_rts_after_send != 0 {
            coarse_sleep(Duration::from_millis(port_ptr.rs485.delay_rts_after_send as _));
        }

        let mut cr = Self::pl011_read(port, Register::RegCr as usize);

        if port_ptr.rs485.flags & SER_RS485_RTS_AFTER_SEND != 0 {
            cr &= !UART011_CR_RTS;
        } else {
            cr |= UART011_CR_RTS;
        }

        /* Disable the transmitter and reenable the transceiver */
        cr &= !UART011_CR_TXE;
        cr |= UART011_CR_RXE;
        Self::pl011_write(cr, port, Register::RegCr as usize);

        pl011_data.rs485_tx_started = false;
    }

    fn pl011_shutdown_channel(port: &UartPort, lcrh: usize) {
        let mut val = Self::pl011_read(port, lcrh);
        val &= !(UART01X_LCRH_BRK | UART01X_LCRH_FEN);
        Self::pl011_write(val, port, lcrh);
    }

    fn pl011_split_lcrh(port: &UartPort) -> bool {
        Self::pl011_reg_to_offset(Register::RegLcrhRx as usize) !=
            Self::pl011_reg_to_offset(Register::RegLcrhTx as usize)
    }

    fn pl011_disable_uart(port: &UartPort) {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };

        port_ptr.status &= !(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);

        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        // 加锁以保护对 CR 寄存器的访问
        unsafe { bindings::_raw_spin_lock_irq(&(*union_ptr).rlock as *const _ as *mut _); }

        let mut cr = Self::pl011_read(port, Register::RegCr as usize);
        cr &= UART011_CR_RTS | UART011_CR_DTR;
        cr |= UART01X_CR_UARTEN | UART011_CR_TXE;
        Self::pl011_write(cr, port, Register::RegCr as usize);

        // 解锁
        unsafe { bindings::_raw_spin_unlock_irq(&(*union_ptr).rlock as *const _ as *mut _); }

        /*
	    * disable break condition and fifos
	    */
        Self::pl011_shutdown_channel(port, Register::RegLcrhRx as usize);
        if Self::pl011_split_lcrh(port) {
            Self::pl011_shutdown_channel(port, Register::RegLcrhTx as usize);
        }
    }

    fn pl011_setup_status_masks(port: &UartPort, new: *mut bindings::ktermios) {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };

        port_ptr.read_status_mask = UART011_DR_OE | 255;

        if unsafe { (*new).c_iflag & bindings::INPCK != 0 } {
            port_ptr.read_status_mask |= UART011_DR_FE | UART011_DR_PE;
        }
        if unsafe { (*new).c_iflag & (bindings::IGNBRK | bindings::BRKINT | bindings::PARMRK) != 0 } {
            port_ptr.read_status_mask |= UART011_DR_BE;
        }

        /*
         * Characters to ignore
         */
        port_ptr.ignore_status_mask = 0;
        if unsafe { (*new).c_iflag & bindings::IGNPAR != 0 } {
            port_ptr.ignore_status_mask |= UART011_DR_FE | UART011_DR_PE;
        }

        if unsafe { (*new).c_iflag & bindings::IGNBRK != 0 } {
            port_ptr.ignore_status_mask |= UART011_DR_BE;
            /*
             * If we're ignoring parity and break indicators,
             * ignore overruns too (for real raw support).
             */
            if unsafe { (*new).c_iflag & bindings::IGNPAR != 0 } {
                port_ptr.ignore_status_mask |= UART011_DR_OE;
            }
        }

        /*
         * Ignore all characters if CREAD is not set.
         */
        if unsafe { (*new).c_cflag & bindings::CREAD == 0 } {
            port_ptr.ignore_status_mask |= UART_DUMMY_DR_RX;
        }
    }

    fn pl011_enable_ms(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        // 启用调制解调器状态中断，包括环路指示器 (UART011_RIMIM)、
        // 清除发送 (UART011_CTSMIM)、数据载波检测 (UART011_DCDMIM) 和数据终端就绪 (UART011_DSRMIM)
        pl011_data.im |= UART011_RIMIM | UART011_CTSMIM | UART011_DCDMIM | UART011_DSRMIM;

        // 将中断掩码写入 IMSC 寄存器
        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);
    }

    fn pl011_write_lcr_h(port: &UartPort, lcr_h: u32) {
        Self::pl011_write(lcr_h, port, Register::RegLcrhRx as usize);
        if Self::pl011_split_lcrh(port) {
            /*
             * Wait 10 PCLKs before writing LCRH_TX register,
             * to get this delay write read only register 10 times
             */

            for i in 0..10 {
                Self::pl011_write(0xff, port, Register::RegMis as usize);
            }
            Self::pl011_write(lcr_h, port, Register::RegLcrhTx as usize);
        }
    }

    #[cfg(CONFIG_CONSOLE_POLL)]
    fn pl011_quiesce_irqs(port: &UartPort) {
        Self::pl011_write(Self::pl011_read(port, Register::RegMis as usize), port, Register::RegIcr as usize);
        /*
         * 无法清除 TXIM，因为这是“准备传输中断”，所以我们只需屏蔽它。start_tx() 将会取消屏蔽它。
         *
         * 请注意，我们可能会与 start_tx() 发生竞争，如果竞争发生，轮询用户可能会在我们清除中断后立即收到另一个中断。
         * 但这应该是可以的，因为即使没有竞争，也可能发生这种情况，例如控制器立即获得一些新数据并触发了中断。
         *
         * 任何使用轮询例程的人都假设它管理设备（包括传输队列），因此我们也可以处理 start_tx() 调用方的情况。
         */

        Self::pl011_write(Self::pl011_read(port, Register::RegImsc as usize) & !UART011_TXIM, port, Register::RegImsc as usize);
    }
}

#[vtable]
impl UartPortOps for AmbaPl011Pops {
    type Data = Box<PL011Data>;

    /// * @tx_empty:      check if the UART TX FIFO is empty
    fn tx_empty(port: &UartPort) -> u32 {
        let status = Self::pl011_read(port, Register::RegFr as usize) ^ VENDOR_DATA.inv_fr;
        if status & (VENDOR_DATA.fr_busy | UART01X_FR_TXFF) == 1 {
            0
        } else {
            TIOCSER_TEMT as u32
        }
    }

    /// * @set_mctrl:    set the modem control register
    fn set_mctrl(port: &UartPort, mctrl: u32) {
        // 读取当前的控制寄存器值
        let mut cr = Self::pl011_read(port, Register::RegCr as usize);

        // 辅助函数用于设置或清除寄存器位
        let set_or_clear_bit = |current_cr: u32, mctrl_bit: u32, reg_bit: u32| {
            if mctrl & mctrl_bit != 0 {
                current_cr | reg_bit // 设置位
            } else {
                current_cr & !reg_bit // 清除位
            }
        };

        // 更新控制寄存器的各个位
        cr = set_or_clear_bit(cr, TIOCM_RTS, UART011_CR_RTS);
        cr = set_or_clear_bit(cr, TIOCM_DTR, UART011_CR_DTR);
        cr = set_or_clear_bit(cr, TIOCM_OUT1, UART011_CR_OUT1);
        cr = set_or_clear_bit(cr, TIOCM_OUT2, UART011_CR_OUT2);
        cr = set_or_clear_bit(cr, TIOCM_LOOP, UART011_CR_LBE);

        let port_ptr = unsafe { &*port.as_ptr() };
        // 如果启用了自动 RTS 控制，检查是否需要禁用自动 RTS
        if port_ptr.status & UPSTAT_AUTORTS != 0 {
            cr &= !UART011_CR_RTSEN; // 清除 RTSEN 位
        }

        // 将更新后的控制寄存器值写回到硬件
        Self::pl011_write(cr, port, Register::RegCr as usize);
    }

    /// * @get_mctrl:    get the modem control register
    fn get_mctrl(port: &UartPort) -> u32 {
        let mut result: u32 = 0;
        let status = Self::pl011_read(port, Register::RegFr as usize);

        let get_or_clear_bit = |current_result: u32, mctrl_bit: u32, reg_bit: u32| {
            if status & mctrl_bit != 0 {
                current_result | reg_bit // 设置位
            } else {
                current_result
            }
        };

        result = get_or_clear_bit(result, UART01X_FR_DCD, TIOCM_CAR);
        result = get_or_clear_bit(result, VENDOR_DATA.fr_dsr, TIOCM_DSR);
        result = get_or_clear_bit(result, VENDOR_DATA.fr_cts, TIOCM_CTS);
        result = get_or_clear_bit(result, VENDOR_DATA.fr_ri, TIOCM_RNG);

        result
    }

    /// * @stop_tx:      stop transmitting
    fn stop_tx(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };

        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };
        pl011_data.im &= !UART011_TXIM;
        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);
    }

    /// * @start_tx:    start transmitting
    fn start_tx(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        // 如果启用了 RS485 模式且传输尚未开始，调用 RS485 传输启动函数
        if ((port_ptr.rs485.flags & SER_RS485_ENABLED) != 0) && !pl011_data.rs485_tx_started {
            Self::pl011_rs485_tx_start(port);
        }

        // 启动 PIO 模式下的传输
        Self::pl011_start_tx_pio(port);
    }

    /// * @throttle:     stop receiving
    fn throttle(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };

        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        // 进入临界区，保存当前中断状态并禁止中断，以确保线程安全
        let flags = unsafe { bindings::_raw_spin_lock_irqsave(&(*union_ptr).rlock as *const _ as *mut _) };

        // 停止 UART 的接收操作
        Self::pl011_stop_rx(port, pl011_data.as_mut());

        unsafe {
            // 恢复之前保存的中断状态，并退出临界区
            bindings::_raw_spin_unlock_irqrestore(&(*union_ptr).rlock as *const _ as *mut _, flags);
        }
    }

    /// * @unthrottle:   start receiving
    fn unthrottle(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };

        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        // 进入临界区，保存当前中断状态并禁止中断，以确保线程安全
        let flags = unsafe { bindings::_raw_spin_lock_irqsave(&(*union_ptr).rlock as *const _ as *mut _) };

        // 设置中断掩码，启用接收中断 (UART011_RXIM) 和接收定时中断 (UART011_RTIM)
        pl011_data.im = UART011_RTIM | UART011_RXIM;
        // 将中断掩码写入 IMSC 寄存器
        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);

        unsafe {
            // 恢复之前保存的中断状态，并退出临界区
            bindings::_raw_spin_unlock_irqrestore(&(*union_ptr).rlock as *const _ as *mut _, flags);
        }
    }

    /// * @send_xchar:  send a break character
    fn send_xchar(_port: &UartPort, ch: i8) {}

    /// * @stop_rx:      stop receiving
    fn stop_rx(_port: &UartPort) {}

    /// * @start_rx:    start receiving
    fn start_rx(_port: &UartPort) {}

    /// * @enable_ms:    enable modem status interrupts
    fn enable_ms(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };

        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        // 启用调制解调器状态中断，包括环路指示器 (UART011_RIMIM)、
        // 清除发送 (UART011_CTSMIM)、数据载波检测 (UART011_DCDMIM) 和数据终端就绪 (UART011_DSRMIM)
        pl011_data.im |= UART011_RIMIM | UART011_CTSMIM | UART011_DCDMIM | UART011_DSRMIM;
        // 将中断掩码写入 IMSC 寄存器
        Self::pl011_write(pl011_data.im, port, Register::RegImsc as usize);
    }

    /// * @break_ctl:   set the break control
    fn break_ctl(port: &UartPort, ctl: i32) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        // 进入临界区，保存当前中断状态并禁止中断，以确保线程安全
        let flags = unsafe { bindings::_raw_spin_lock_irqsave(&(*union_ptr).rlock as *const _ as *mut _) };

        // 读取当前 LCRH_TX 寄存器的值
        let mut lcr_h = Self::pl011_read(port, Register::RegLcrhTx as usize);
        // 根据 ctl 的值设置或清除 LCRH 寄存器中的 Break 位
        if ctl == -1 {
            lcr_h |= UART01X_LCRH_BRK;
        } else {
            lcr_h &= !UART01X_LCRH_BRK;
        }

        // 将修改后的 LCRH 值写回寄存器
        Self::pl011_write(lcr_h, port, Register::RegLcrhTx as usize);

        // 恢复之前保存的中断状态，并退出临界区
        unsafe { bindings::_raw_spin_unlock_irqrestore(&(*union_ptr).rlock as *const _ as *mut _, flags) };
    }

    /// * @startup:      start the UART
    fn startup(port: &UartPort) -> i32 {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        // 初始化硬件
        if let Err(_) = Self::pl011_hwinit(port, pl011_data.as_mut()) {
            // clk_disable_unprepare(uap->clk); // 未知!!!!! 好像drop自动实现
            return 1;
        }

        // 分配中断
        if let Err(_) = Self::pl011_allocate_irq(port, pl011_data.as_mut()) {
            // clk_disable_unprepare(uap->clk); // 未知!!!!! 好像drop自动实现
            return 1;
        }

        // 配置 FIFO 水平
        Self::pl011_write(pl011_data.vendor.ifls, port, Register::RegIfls as usize);

        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        // 加锁以保护对 CR 寄存器的访问
        unsafe { bindings::_raw_spin_lock_irq(&(*union_ptr).rlock as *const _ as *mut _); }

        // 读取当前 CR 寄存器的值
        let mut cr = Self::pl011_read(port, Register::RegCr as usize);
        cr &= UART011_CR_RTS | UART011_CR_DTR;
        cr |= UART01X_CR_UARTEN | UART011_CR_RXE;

        // 如果 RS485 模式未启用，启用 TX
        if port_ptr.rs485.flags & SER_RS485_ENABLED == 0 {
            cr |= UART011_CR_TXE;
        }

        // 写回 CR 寄存器
        Self::pl011_write(cr, port, Register::RegCr as usize);

        // 解锁
        unsafe { bindings::_raw_spin_unlock_irq(&(*union_ptr).rlock as *const _ as *mut _); }

        // 初始化调制解调器信号的旧状态
        pl011_data.old_status = Self::pl011_read(port, Register::RegFr as usize) & UART01X_FR_MODEM_ANY;

        // 启用中断
        Self::pl011_enable_interrupts(port);

        0
    }

    /// * @shutdown:     shutdown the UART
    fn shutdown(port: &UartPort) {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        Self::pl011_disable_interrupts(port, pl011_data.as_mut());

        if port_ptr.rs485.flags & SER_RS485_ENABLED != 0 && pl011_data.rs485_tx_started {
            Self::pl011_rs485_tx_stop(port);
        }

        // free_irq(uap->port.irq, uap); // 未知!!!!! 好像drop自动实现

        Self::pl011_disable_uart(port);


        /*
         * Shut down the clock producer
         */
        // clk_disable_unprepare(uap->clk);  // 未知!!!!! 好像drop自动实现

        /* Optionally let pins go into sleep states */
        // pinctrl_pm_select_sleep_state(port->dev);  // 未知!!!!! 怎么实现？

        //  未知!!!!! 怎么实现？
        // if (dev_get_platdata(uap->port.dev)) {
        //     struct amba_pl011_data *plat;
        //
        //     plat = dev_get_platdata(uap->port.dev);
        //     if (plat->exit)
        //     plat->exit();
        // }

        unsafe {
            if let Some(flush_buffer_fn) = (*port_ptr.ops).flush_buffer {
                flush_buffer_fn(port.as_ptr());
            }
        }
    }

    /// * @flush_buffer: flush the UART buffer
    fn flush_buffer(_port: &UartPort) {}

    /// * @set_termios: set the termios structure
    fn set_termios(
        port: &UartPort,
        new: *mut bindings::ktermios,
        old: *const bindings::ktermios,
    )
    {
        let mut port_ptr = unsafe { &mut *port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        let mut clkdiv: u32 = 0;
        // 根据是否支持过采样来设置时钟分频器
        if pl011_data.vendor.oversampling {
            clkdiv = 8;
        } else {
            clkdiv = 16;
        }

        /*
         * 请求核心函数为我们计算分频器
         */
        let baud = unsafe {
            bindings::uart_get_baud_rate(port.as_ptr(), new, old, 0,
                                         port_ptr.uartclk / clkdiv)
        };

        fn div_round_closest(x: u32, divisor: u32) -> u32 {
            (x + (divisor / 2)) / divisor     // 未知!!!!!  可能有错
        }

        let mut quot: u32 = 0;
        // 根据波特率计算分频器
        if baud > port_ptr.uartclk / 16 {
            quot = div_round_closest(port_ptr.uartclk * 8, baud);
        } else {
            quot = div_round_closest(port_ptr.uartclk * 4, baud);
        }

        // 根据 c_cflag 设置数据位
        let mut lcr_h: u32 = unsafe {
            match (*new).c_cflag & bindings::CSIZE {
                bindings::CS5 => UART01X_LCRH_WLEN_5,
                bindings::CS6 => UART01X_LCRH_WLEN_6,
                bindings::CS7 => UART01X_LCRH_WLEN_7,
                _ => UART01X_LCRH_WLEN_8,
            }
        };

        // 设置停止位
        if unsafe { (*new).c_cflag & bindings::CSTOPB != 0 } {
            lcr_h |= UART01X_LCRH_STP2;
        }

        // 设置校验位
        if unsafe { (*new).c_cflag & bindings::PARENB != 0 } {
            lcr_h |= UART01X_LCRH_PEN;

            if unsafe { !((*new).c_cflag & bindings::PARODD) != 0 } {
                lcr_h |= UART01X_LCRH_EPS;
            }

            if unsafe { (*new).c_cflag & bindings::CMSPAR != 0 } {
                lcr_h |= UART011_LCRH_SPS;
            }
        }

        // 如果 FIFO 大小大于 1，启用 FIFO
        if pl011_data.fifosize > 1 {
            lcr_h |= UART01X_LCRH_FEN;
        }

        // 获取帧大小
        let bits = unsafe { bindings::tty_get_frame_size((*new).c_cflag) };

        let union_ptr = &port_ptr.lock as *const _ as *const bindings::spinlock__bindgen_ty_1;   // 未知!!!!!  转化问题疑问
        // 保护代码块，禁止中断
        let flags = unsafe { bindings::_raw_spin_lock_irqsave(&(*union_ptr).rlock as *const _ as *mut _) };

        /*
         * 更新每个端口的超时
         */
        unsafe { bindings::uart_update_timeout(port.as_ptr(), (*new).c_cflag, baud) };

        fn div_round_up(n: u32, d: u32) -> u32 {
            assert!(d > 0, "divisor must be greater than zero");
            (n + d - 1) / d
        }

        /*
        * 计算发送一个字符所需的时间。我们在等待传输队列为空时使用此轮询间隔
        */
        pl011_data.rs485_tx_drain_interval = div_round_up((bits as u32) * 1000 * 1000,
                                                          baud as u32) as u64;

        // 设置状态掩码
        Self::pl011_setup_status_masks(port, new);

        // 如果启用调制解调器状态信号
        if unsafe { port_ptr.flags & UPF_HARDPPS_CD != 0 || (*new).c_cflag & bindings::CRTSCTS != 0 || !((*new).c_cflag & bindings::CLOCAL) != 0 } {
            Self::pl011_enable_ms(port);
        }

        // 如果启用 RS485，禁用 CRTSCTS
        if port_ptr.rs485.flags & SER_RS485_ENABLED != 0 {
            unsafe { (*new).c_cflag &= !bindings::CRTSCTS }
        }

        // 读取当前控制寄存器的值
        let mut old_cr = Self::pl011_read(port, Register::RegCr as usize);

        // 根据 c_cflag 设置硬件流控制
        if unsafe { (*new).c_cflag & bindings::CRTSCTS != 0 } {
            if old_cr & UART011_CR_RTS != 0 {
                old_cr |= UART011_CR_RTSEN;
            }

            old_cr |= UART011_CR_CTSEN;
            port_ptr.status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
        } else {
            old_cr &= !(UART011_CR_CTSEN | UART011_CR_RTSEN);
            port_ptr.status &= !(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
        }

        // 如果vendor支持过采样，调整控制寄存器的过采样位
        if pl011_data.vendor.oversampling {
            if baud > port_ptr.uartclk / 16 {
                old_cr |= ST_UART011_CR_OVSFACT;
            } else {
                old_cr &= !ST_UART011_CR_OVSFACT;
            }
        }

        /*
         * 对于 ST Micro 过采样变体，通过降低分频器略微增加比特率，以避免高速度下采样延迟导致的数据损坏。
         */
        if pl011_data.vendor.oversampling {
            if (baud >= 3000000) && (baud < 3250000) && (quot > 1) {
                quot -= 1;
            } else if (baud > 3250000) && (quot > 2) {
                quot -= 2;
            }
        }

        // 设置波特率
        Self::pl011_write(quot & 0x3f, port, Register::RegFbrd as usize);
        Self::pl011_write(quot >> 6, port, Register::RegIbrd as usize);

        /*
         * NOTE: 必须在 REG_FBRD 和 REG_IBRD 之后写入 REG_LCRH_TX 和 REG_LCRH_RX。
         */
        Self::pl011_write_lcr_h(port, lcr_h);

        /*
         * 接收已被 pl011_disable_uart 在关闭期间禁用。如果你需要使用 tty_driver 在 tty_find_polling_driver() 后返回，则需要重新启用接收。
         */
        old_cr |= UART011_CR_RXE;
        Self::pl011_write(old_cr, port, Register::RegCr as usize);

        // 释放代码块，允许中断
        unsafe { bindings::_raw_spin_unlock_irqrestore(&(*union_ptr).rlock as *const _ as *mut _, flags) };
    }

    /// * @set_ldisc:    set the line discipline
    fn set_ldisc(_port: &UartPort, _arg2: *mut bindings::ktermios) {}

    /// * @pm:            power management
    fn pm(_port: &UartPort, _state: u32, _oldstate: u32) {}

    /// * @type:          get the type of the UART
    fn port_type(port: &UartPort) -> *const i8 {
        let port_ptr = unsafe { &*port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };

        if port_ptr.type_ == PORT_AMBA {
            // 将 type_ 字段转换为 C 字符串指针
            let c_str = CStr::from_bytes_with_nul(&pl011_data.type_).unwrap();
            c_str.as_ptr() as *const i8
        } else {
            ptr::null()
        }
    }

    /// * @release_port: release the UART port
    fn release_port(uart_port: &UartPort) {}

    /// * @request_port: request the UART port
    fn request_port(uart_port: &UartPort) -> i32 {
        todo!();
    }

    /// * @config_port:  configure the UART port
    fn config_port(uart_port: &UartPort, flags: i32) {
        if flags & UART_CONFIG_TYPE != 0 {
            let mut port_ptr = unsafe { &mut *uart_port.as_ptr() };
            port_ptr.type_ = PORT_AMBA;
        }
    }

    /// * @verify_port:  verify the UART port
    fn verify_port(uart_port: &UartPort, ser: *mut bindings::serial_struct) -> i32 {
        let port_ptr = unsafe { &*uart_port.as_ptr() };
        let mut ret: i32 = 0;
        if unsafe { (*ser).type_ != bindings::PORT_UNKNOWN as i32 && (*ser).type_ != bindings::PORT_AMBA as i32 } {
            ret = -(bindings::EINVAL as i32);
        }
        if unsafe { (*ser).irq < 0 || (*ser).irq >= bindings::NR_IRQS as i32 } {
            ret = -(bindings::EINVAL as i32);
        }
        if unsafe { (*ser).baud_base < 9600 } {
            ret = -(bindings::EINVAL as i32);
        }
        if unsafe { port_ptr.mapbase != (*(*ser).iomem_base) as u64 } {
            ret = -(bindings::EINVAL as i32);
        }

        ret
    }

    /// * @ioctl:        ioctl handler
    fn ioctl(uart_port: &UartPort, arg2: u32, arg3: u64) -> i32 {
        todo!();
    }

    #[cfg(CONFIG_CONSOLE_POLL)]
    fn poll_init(uart_port: &UartPort) -> i32 {
        let port_ptr = unsafe { &*uart_port.as_ptr() };
        let mut pl011_data: Box<PL011Data> = unsafe {
            <Self::Data as ForeignOwnable>::from_foreign(port_ptr.private_data)
        };
        // 初始化硬件
        if let Err(_) = Self::pl011_hwinit(uart_port, pl011_data.as_mut()) {
            return 1;
        }
        0
    }

    #[cfg(CONFIG_CONSOLE_POLL)]
    fn poll_put_char(uart_port: &UartPort, arg2: u8) {
        // 等待传输队列不满
        while Self::pl011_read(uart_port, Register::RegFr as usize) & UART01X_FR_TXFF != 0 {
            // 放松 CPU，以防止忙等待导致 CPU 过度消耗
            cpu_relax();  // 未知!!!!!  不确定
        }

        // 将字符写入传输数据寄存器
        Self::pl011_write(arg2 as u32, uart_port, Register::RegDr as usize);
    }

    #[cfg(CONFIG_CONSOLE_POLL)]
    fn poll_get_char(uart_port: &UartPort) -> i32 {
        /*
         * The caller might need IRQs lowered, e.g. if used with KDB NMI
         * debugger.
         */
        Self::pl011_quiesce_irqs(uart_port);

        let status = Self::pl011_read(uart_port, Register::RegFr as usize);
        if status & UART01X_FR_RXFE != 0 {
            return bindings::NO_POLL_CHAR as i32;
        }

        Self::pl011_read(uart_port, Register::RegDr as usize) as i32
    }
}

// PL011注册类型
type PL011Registrations = PortRegistration<AmbaPl011Pops>;
// PL011设备数据类型
type PL011DeviceData = device::Data<PL011Registrations, PL011Resources, PL011Data>;

struct AmbaDeviceData {
    inner: Pin<UniqueArc<PL011DeviceData>>,
}

impl driver::DeviceRemoval for AmbaDeviceData {
    fn device_remove(&self) {
        dbg!("********* 释放资源 *********\n");
    }
}

// 定义AMBA ID表
kernel::define_amba_id_table! {MY_AMDA_ID_TABLE, (), [
    ({id: 0x00041011, mask: 0x000fffff}, None),
]}
// 定义模块的AMBA ID表
kernel::module_amba_id_table!(UART_MOD_TABLE, MY_AMDA_ID_TABLE);

// PL011设备的结构体
struct PL011Device;
// 实现AMBA驱动程序接口
impl amba::Driver for PL011Device {
    // type Data = Box<AmbaDeviceData>;
    type Data = ();

    // AMBA ID表
    kernel::driver_amba_id_table!(MY_AMDA_ID_TABLE);
    // 探测函数
    fn probe(adev: &mut amba::Device, _data: Option<&Self::IdInfo>) -> Result {
        dev_info!(adev,"{} PL061 GPIO chip (probe)\n",adev.name());
        dbg!("********** PL061 GPIO chip (probe) *********\n");

        // 查找可用的端口号
        let portnr = pl011_find_free_port()?;
        dev_info!(adev,"portnr is {}\n",portnr);

        // 确定FIFO大小
        let fifosize = if adev.revision_get().unwrap() < 3 { 16 } else { 32 };
        // 设置IO类型
        let iotype = UPIO_MEM as u8;
        // 获取寄存器基地址
        let reg_base = adev.take_resource().ok_or(ENXIO)?;
        // 初始化IO内存
        let reg_mem: IoMem<UART_SIZE> = unsafe { IoMem::try_new(&reg_base)? };
        // 获取基地址的偏移量
        let mapbase = reg_base.get_offset();
        // 获取内存基地址
        let membase = reg_mem.get();
        // 获取中断号
        let irq = adev.irq(0).ok_or(ENXIO)?;

        dev_info!(adev,"fifosize is {}\n",fifosize);
        dev_info!(adev,"mapbase is 0x{:x}\n", mapbase);
        dev_info!(adev,"membase is 0x{:p}\n",membase);
        dev_info!(adev,"irq is {}\n",irq);

        // 确定系统请求支持
        let has_sysrq = 1;
        // 设置标志
        let flags = UPF_BOOT_AUTOCONF;
        // 创建并设置串口端口
        let mut uap = UartPort::new();

        let dev = device::Device::from_dev(adev);
        let index = pl011_probe_dt_alias(portnr as _, dev.clone());
        uap.setup(
            membase,
            mapbase,
            irq,
            iotype,
            flags,
            has_sysrq,
            fifosize,
            index as _,
        );

        // 未知!!!!! 麻烦、复杂
        let mut type_: [u8; 12] = [0; 12];
        // 设置端口类型
        let rev = adev.revision_get().unwrap();
        let type_str = CString::try_from_fmt(fmt!("PL011 rev{}", rev)).unwrap();
        // 将格式化的字符串复制到 uap.type 中
        let bytes = type_str.as_bytes_with_nul(); // 包含 NUL 终止符
        let len = bytes.len().min(type_.len());
        type_[..len].copy_from_slice(&bytes[..len]);
        // 手动填充剩余部分为 0
        for byte in &mut type_[len..] {
            *byte = 0;
        }

        let pl011_data = PL011Data {
            reg_offset: VENDOR_DATA.reg_offset,
            im: 0,
            old_status: 0,
            fifosize,
            fixed_baud: 0,
            type_: type_,
            vendor: VENDOR_DATA,
            rs485_tx_started: false,
            rs485_tx_drain_interval: 0,
        };

        let pl011_resources = PL011Resources {
            base: reg_mem,
            parent_irq: irq,
        };

        // 未知!!!!!  所有权问题，可能要改
        let mut pl011_registrations: PortRegistration<AmbaPl011Pops> = PortRegistration::new(uap);
        unsafe { PORTS[portnr as usize] = Some(uap) };

        // let pl011_device_data = new_device_data!(pl011_registrations, pl011_resources, pl011_data, "ttyAMA")?;  // 未知!!!!!  可能有问题

        /* 确保该 UART 的中断被屏蔽和清除 */
        AmbaPl011Pops::pl011_write(0, &uap, Register::RegImsc as usize); // 屏蔽中断
        AmbaPl011Pops::pl011_write(0xffff, &uap, Register::RegIcr as usize); // 清除中断

        amba_set_drvdata(dev.clone(), &mut uap);

        let pl011_registrations_pin = pin!(pl011_registrations);
        pl011_registrations_pin.register(
            adev,
            &UART_DRIVER,
            portnr as _,
            Box::try_new(pl011_data)?,
        )?;

        dbg!("********* PL061 GPIO芯片已注册 *********\n");
        // Ok(Box::try_new(
        //     AmbaDeviceData {
        //         inner: pl011_device_data,
        // })?)

        Ok(())
    }
}


// 注册AMBA驱动程序
module_amba_driver! {
    type: PL011Device,
    name: "pl011_uart",
    author: "Tang Hanwen",
    license: "GPL",
    initcall: "arch",
}

/// 查找可用的驱动端口。
fn pl011_find_free_port() -> Result<usize> {
    for (index, port) in unsafe { PORTS.iter().enumerate() } {
        if let None = port {
            return Ok(index);
        }
    }
    Err(EBUSY)
}

// 查找设备树别名函数
fn pl011_probe_dt_alias(index: i32, dev: device::Device) -> i32 {
    let mut seen_dev_with_alias: bool = false; // 标记是否存在别名设备
    let mut seen_dev_without_alias: bool = false; // 标记是否存在无别名设备
    let mut ret = index;

    if CONFIG_OF == 0 {
        return ret;
    }

    let dev_ptr = dev.raw_device();
    let np = unsafe { (*dev_ptr).of_node }; // 获取设备节点
    if np.is_null() {
        return ret;
    }

    let alias_name = c_str!("serial");
    ret = unsafe { bindings::of_alias_get_id(np, alias_name.as_ptr() as *const _) }; // 获取设备树中的别名 ID

    if ret < 0 {
        seen_dev_without_alias = true; // 标记存在无别名设备
        ret = index; // 使用传入的索引
    } else {
        seen_dev_with_alias = true; // 标记存在别名设备
        if ret >= UART_NR as _ || unsafe { !PORTS[ret as usize].is_none() } { // 检查别名 ID 是否有效
            dev_warn!(dev, "requested serial port {}  not available.\n", ret); // 打印警告信息
            ret = index; // 使用传入的索引
        }
    }

    if seen_dev_with_alias && seen_dev_without_alias {  // 如果同时存在别名和无别名设备
        dev_warn!(dev, "aliased and non-aliased serial devices found in device tree. Serial port enumeration may be unpredictable.\n"); // 打印警告信息
    }

    ret // 返回索引
}

fn amba_set_drvdata(dev: device::Device, uap: &mut UartPort){
    let dev_ptr = dev.raw_device();
    unsafe { (*dev_ptr).driver_data = uap.as_ptr() as *mut _ };
}

