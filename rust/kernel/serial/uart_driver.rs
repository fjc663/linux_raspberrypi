// SPDX-License-Identifier: GPL-2.0

//! 支持UART驱动程序。
//!
//! C头文件: ['include/linux/serial_core.h']

use super::uart_console::Console;  // 引入UART控制台模块

use core::mem::MaybeUninit;  // 导入可能未初始化的内存模块

use crate::{
    bindings,  // 内核绑定
    error::{to_result, Result},  // 错误处理模块，包括结果类型和错误转换函数
    pr_info,  // 内核信息打印宏
    str::CStr,  // 字符串处理模块
    ThisModule,  // 当前模块
};

/// 封装内核的`struct uart_driver`。
///
/// # 不变条件
///
/// 指针是非空且有效的，并且在整个引用期间引用计数不为零。
#[repr(transparent)]
pub struct UartDriver(bindings::uart_driver);  // 定义一个透明包装类型，封装内核的`struct uart_driver`

impl UartDriver {
    /// 创建一个新的[`UartDriver`]
    ///
    /// # 参数
    /// - `owner`: 指向当前模块的指针
    /// - `driver_name`: 驱动程序名称
    /// - `dev_name`: 设备名称
    /// - `reg`: 关联的控制台对象
    pub const fn new(owner: &'static ThisModule, driver_name: &'static CStr, dev_name: &'static CStr, reg: &Console) -> Self {
        // SAFETY: `uart_driver`是一个C结构，持有已用0初始化的数据，
        // 因此可以安全地使用。
        let mut uart_driver = unsafe { MaybeUninit::<bindings::uart_driver>::zeroed().assume_init() };
        uart_driver.owner = owner.as_ptr();  // 设置驱动程序的所有者
        uart_driver.driver_name = driver_name.as_char_ptr();  // 设置驱动程序名称
        uart_driver.dev_name = dev_name.as_char_ptr();  // 设置设备名称
        uart_driver.cons = reg.as_ptr();  // 设置控制台指针
        // uart_driver.state = null_mut();
        // uart_driver.tty_driver = null_mut();
        Self(uart_driver)  // 返回封装的`UartDriver`实例
    }

    /// 设置控制台的其他配置
    ///
    /// # 参数
    /// - `major`: 主要设备号
    /// - `minor`: 次要设备号
    /// - `nr`: 设备数量
    pub const fn with_config(
        mut self,
        major: i32,
        minor: i32,
        nr: i32
    ) -> Self {
        self.0.major = major;  // 设置主要设备号
        self.0.minor = minor;  // 设置次要设备号
        self.0.nr = nr;  // 设置设备数量
        self  // 返回配置后的`UartDriver`实例
    }

    /// 从有效指针创建对[`UartDriver`]的引用。
    ///
    /// # 安全
    ///
    /// 调用者必须确保`ptr`有效，非空，并且在返回引用存在期间引用计数不为零。
    ///
    /// # 参数
    /// - `ptr`: 指向`uart_driver`的指针
    ///
    /// # 返回
    /// 对`UartDriver`的引用
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::uart_driver) -> &'a Self {
        // SAFETY: 由函数的安全要求保证。
        unsafe { &*ptr.cast() }
    }

    /// 返回内部C结构的原始指针。
    ///
    /// # 返回
    /// 指向内部C结构的原始指针
    #[inline]
    pub const fn as_ptr(&self) -> *mut bindings::uart_driver {
        &self.0 as *const _ as *mut _  // 返回指向内部C结构的原始指针
    }

    /// 使用`uart_register_driver`注册驱动程序到UART核心层。
    ///
    /// 将UART驱动程序注册到核心驱动程序。我们依次注册到TTY层，
    /// 并初始化每个端口的核心驱动程序状态。
    ///
    /// # 返回
    /// 注册结果
    pub fn register(&self) -> Result {
        unsafe {
            to_result(bindings::uart_register_driver(self.as_ptr()))?;  // 调用内核函数注册UART驱动程序
            Ok(())
        }
    }

}

impl Drop for UartDriver {
    /// 在对象销毁时调用，用于取消注册驱动程序
    fn drop(&mut self) {
        pr_info!("UartDriver Drop!");  // 打印信息，表示驱动程序正在销毁
        unsafe { bindings::uart_unregister_driver(&mut self.0) };  // 调用内核函数取消注册UART驱动程序
    }
}

unsafe impl Send for UartDriver {}  // 实现`Send`特性，表示`UartDriver`可以在线程间传递
unsafe impl Sync for UartDriver {}  // 实现`Sync`特性，表示`UartDriver`可以在线程间共享

/// UartDriver的引用类型
pub struct UartDriverRef(*mut bindings::uart_driver);

impl UartDriverRef {
    /// 返回指向`uart_driver`的原始指针
    ///
    /// # 返回
    /// 指向`uart_driver`的原始指针
    #[warn(dead_code)]
    fn as_ptr(&self) -> *mut bindings::uart_driver {
        self.0
    }
}

impl From<bindings::uart_driver> for UartDriver {
    /// 从`bindings::uart_driver`转换为`UartDriver`
    ///
    /// # 参数
    /// - `value`: `bindings::uart_driver`实例
    ///
    /// # 返回
    /// `UartDriver`实例
    fn from(value: bindings::uart_driver) -> Self {
        Self(value)
    }
}

impl From<*mut bindings::uart_driver> for UartDriverRef {
    /// 从指向`bindings::uart_driver`的指针转换为`UartDriverRef`
    ///
    /// # 参数
    /// - `value`: 指向`bindings::uart_driver`的指针
    ///
    /// # 返回
    /// `UartDriverRef`实例
    fn from(value: *mut bindings::uart_driver) -> Self {
        Self(value)
    }
}
