// SPDX-License-Identifier: GPL-2.0

//! 支持UART控制台。
//!
//! C头文件: ['include/linux/console.h']

use crate::{
    bindings,  // 内核绑定
    error::{Result, from_result},  // 错误处理模块，包括结果类型和错误转换函数
    types::ForeignOwnable,  // 类型模块，包含外部所有权类型
};

use core::mem::MaybeUninit;  // 导入可能未初始化的内存模块

use macros::vtable;  // 导入虚表宏

/// 与[`Console`]关联的一般控制台标志。
/// 访问模式标志的位掩码。
///
/// # 示例
///
/// ```
/// use kernel::serial::uart_console;
/// # fn do_something() {}
/// # let flags = 0;
/// if (flags & uart_console::flags::CON_CONSDEV) == uart_console::flags::CON_BOOT {
///     do_something();
/// }
/// ```
pub mod flags {
    /// 文件以追加模式打开。
    pub const CON_PRINTBUFFER: u32 = bindings::cons_flags_CON_PRINTBUFFER;

    /// 表示控制台驱动程序正在备份。
    pub const CON_CONSDEV: u32 = bindings::cons_flags_CON_CONSDEV;

    /// 表示是否允许控制台打印记录。
    pub const CON_ENABLED: u32 = bindings::cons_flags_CON_ENABLED;

    /// 标记控制台驱动程序为早期控制台驱动程序，
    /// 在真正的驱动程序可用之前用于引导期间。
    pub const CON_BOOT: u32 = bindings::cons_flags_CON_BOOT;

    /// 一个错误命名的历史标志，告诉核心代码
    /// 可以在标记为离线的CPU上调用旧的['console::write']回调。
    pub const CON_ANYTIME: u32 = bindings::cons_flags_CON_ANYTIME;

    /// 表示盲文设备，显然这些设备不会接收printk垃圾信息。
    pub const CON_BRL: u32 = bindings::cons_flags_CON_BRL;

    /// 控制台支持/dev/kmesg的扩展输出格式，
    /// 这需要更大的输出缓冲区。
    pub const CON_EXTENDED: u32 = bindings::cons_flags_CON_EXTENDED;

    /// 表示控制台是否暂停。如果为真，
    /// 则不应调用打印回调。
    pub const CON_SUSPENDED: u32 = bindings::cons_flags_CON_SUSPENDED;
}

/// 控制台的操作
#[vtable]
pub trait ConsoleOps {
    /// 用户数据，将在所有操作中访问
    type Data: ForeignOwnable + Send + Sync = ();

    /// 输出消息的写回调（可选）
    fn console_write(_co: &Console, _s: *const i8, _count: u32);

    /// 控制台输入的读取回调（可选）
    fn console_read(_co: &Console, _s: *mut i8, _count: u32) -> Result<i32>;

    /// 匹配控制台的回调（可选）
    fn console_match(
        _co: &Console,
        _name: *mut i8,
        _idx: i32,
        _options: *mut i8,
    ) -> Result<i32>;

    /// 底层TTY设备驱动程序（可选）
    fn console_device(_co: &Console, _index: *mut i8) -> *mut bindings::tty_driver;
}

/// [`UartDriver`]的控制台
///
/// # 不变条件
///
/// `self.0`始终包含有效数据。
pub struct Console(bindings::console);
impl Console {
    /// 创建一个新的[`UartDriver`]控制台
    pub const fn new<T: ConsoleOps>(name:[i8; 16usize], reg: *mut bindings::uart_driver) -> Self {
        // SAFETY: `console`是一个C结构，持有已用0初始化的数据，
        // 因此可以安全地使用。
        let mut console = unsafe { MaybeUninit::<bindings::console>::zeroed().assume_init() };
        console.name   = name;
        console.write  = Some(console_write_callback::<T>);
        console.read   = Some(console_read_callback::<T>);
        console.match_ = Some(console_match_callback::<T>);
        console.device = Some(console_device_callback::<T>);
        console.data   = reg as _ ;
        Self(console)
    }

    /// 设置控制台的其他配置
    pub const fn with_config(
        mut self,
        flags: i16,
        index: i16,
        cflag: i32,
        ispeed: u32,
        ospeed: u32,
        seq: u64,
        dropped: u64
    ) -> Self {
        self.0.flags = flags;
        self.0.index = index;
        self.0.cflag = cflag;
        self.0.ispeed = ispeed;
        self.0.ospeed = ospeed;
        self.0.seq    = seq;
        self.0.dropped = dropped;
        self
    }

    /// 从有效指针创建对[`Console`]的引用。
    ///
    /// # 安全
    ///
    /// 调用者必须确保`ptr`有效，非空，并且在返回引用存在期间引用计数不为零。
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::console) -> &'a Self {
        // SAFETY: 由函数的安全要求保证。
        unsafe { &*ptr.cast() }
    }

    /// 返回内部C结构的原始指针。
    #[inline]
    pub const fn as_ptr(&self) -> *mut bindings::console {
        &self.0 as *const _ as _
    }
}

unsafe impl Send for Console {}
unsafe impl Sync for Console {}

unsafe extern "C" fn console_write_callback<T: ConsoleOps> (
    co: *mut bindings::console,
    s: *const core::ffi::c_char,
    count: core::ffi::c_uint,
) {
    let co = unsafe { Console::from_raw(co) };
    T::console_write(co, s, count);
}

unsafe extern "C" fn console_read_callback<T: ConsoleOps> (
    co: *mut bindings::console,
    s: *mut core::ffi::c_char,
    count: core::ffi::c_uint,
) -> core::ffi::c_int {
    from_result(|| {
        // SAFETY: 存储为芯片数据的值在注册期间由`into_foreign`返回。
        let co = unsafe { Console::from_raw(co) };
        T::console_read(co, s, count)
    })
}

unsafe extern "C" fn console_match_callback<T: ConsoleOps> (
    co: *mut bindings::console,
    name: *mut core::ffi::c_char,
    idx: core::ffi::c_int,
    options: *mut core::ffi::c_char,
) -> core::ffi::c_int {
    from_result(|| {
        let co = unsafe { Console::from_raw(co) };
        T::console_match(co, name, idx, options)
    })
}

unsafe extern "C" fn console_device_callback<T: ConsoleOps> (
    co: *mut bindings::console,
    index: *mut core::ffi::c_int,
) -> *mut bindings::tty_driver {
    let co = unsafe { Console::from_raw(co) };
    T::console_device(co, index as *mut i8)
}

// pub mod uart_driver {
//     use crate::console::RawConsole;
//     use crate::console::{self, ConsoleOps};
//     use crate::error::{self, to_result};
//     use crate::prelude::EINVAL;
//     use crate::serial_core::uart_port::{self, RawUartPort};
//     use crate::str::CString;
//     use core::fmt;
//     use core::pin::Pin;
//     use kernel::error::Result;

//     pub struct Options {
//         minor: i32,
//         major: i32,
//         nr: i32,
//     }
//     // uart driver reg
//     pub struct Registration<A: ConsoleOps> {
//         registered: bool,
//         uart_driver: bindings::uart_driver,
//         console: console::Registration<bindings::uart_driver, A>,
//     }

//     impl<A: ConsoleOps> Registration<A> {
//         pub fn register_with_options(
//             self: Pin<&mut Self>,
//             name: fmt::Arguments<'_>,
//             opts: &Options,
//         ) -> Result {
//             let this = unsafe { self.get_unchecked_mut() };
//             if this.registered {
//                 return Err(EINVAL);
//             }
//             // 17 is CON_PRINTBUFFER | CON_ANYTIME
//             // -1 is for the index
//             let console = console::Registration::<bindings::uart_driver, A>::new(
//                 name,
//                 17,
//                 -1,
//                 &this.uart_driver as *const _,
//             )?;

//             let name = CString::try_from_fmt(name)?;

//             this.uart_driver.cons = console.raw_console();
//             this.uart_driver.minor = opts.minor;
//             this.uart_driver.major = opts.major;
//             this.uart_driver.nr = opts.nr;

//             unsafe {
//                 let ret = bindings::uart_register_driver(
//                     &this.uart_driver as *const _ as *mut bindings::uart_driver,
//                 );
//                 if ret < 0 {
//                     return Err(error::Error::from_errno(ret));
//                 }
//             };

//             this.registered = true;

//             Ok(())
//         }

//         pub fn unregister(self: Pin<&mut Self>) {
//             let this = unsafe { self.get_unchecked_mut() };
//             if this.registered {
//                 unsafe {
//                     bindings::uart_unregister_driver(
//                         &this.uart_driver as *const _ as *mut bindings::uart_driver,
//                     );
//                 }
//             }
//         }

//         pub fn add_port(self: Pin<&mut Self>, port: &uart_port::UartPort) -> Result {
//             unsafe {
//                 let this = self.get_unchecked_mut();
//                 to_result(bindings::uart_add_one_port(
//                     &this.uart_driver as *const _ as *mut bindings::uart_driver,
//                     port.raw_uart_port(),
//                 ))
//             }
//         }

//         pub fn remove_port(self: Pin<&mut Self>, port: &uart_port::UartPort) {
//             unsafe {
//                 let this = self.get_unchecked_mut();
//                 bindings::uart_remove_one_port(
//                     &this.uart_driver as *const _ as *mut bindings::uart_driver,
//                     port.raw_uart_port(),
//                 )
//             }
//         }
//     }
// }
