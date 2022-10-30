use core::ffi::c_void;
use core::marker::PhantomData;
use core::ptr;

use embedded_hal::sai::{ErrorKind, SaiCommMode};

use esp_idf_sys::*;

use crate::gpio::*;
use crate::peripheral::{Peripheral, PeripheralRef};
// use crate::units::*;

// pub use embedded_hal::I2S::Operation;

crate::embedded_hal_error!(
    SaiError,
    embedded_hal::sai::Error,
    embedded_hal::sai::ErrorKind
);


// pub type I2sMasterConfig = config::MasterConfig;
// pub type I2sSlaveConfig = config::SlaveConfig;

/// Synchronous Audio Interface configuration
pub mod config {
    // use crate::units::*;

    #[repr(u8)]
    pub enum Mode {
        /// Master mode
        Master = (0x1 << 0),
        /// Slave mode
        Slave = (0x1 << 1),
        /// TX mode
        Tx = (0x1 << 2),
        /// RX mode
        Rx = (0x1 << 3),
        /// I2S PDM mode
        Pdm = (0x1 << 6),
    }

    #[repr(u8)]
    pub enum BitsPerSample {
        /// 8 bits per sample
        Bits8 = 8,
        /// 16 bits per sample
        Bits16 = 16,
        /// 24 bits per sample
        Bits24 = 24,
        /// 32 bits per sample
        Bits32 = 32,
    }

    impl From<usize> for BitsPerSample {
        fn from(size: usize) -> Self {
        match size {
            1 => BitsPerSample::Bits8,
            2 => BitsPerSample::Bits16,
            3 => BitsPerSample::Bits24,
            4 => BitsPerSample::Bits32,
            _ => unreachable!(),
        }
    }
    }

    #[repr(u8)]
    pub enum ChannelFormat {
        /// Separated left and right channel
        RightLeft = 0,
        /// Load right channel data in both two channels
        AllRight = 1,
        /// Load left channel data in both two channels
        AllLeft = 2,
        /// Only load data in right channel (mono mode)
        OnlyRight = 3,
        /// Only load data in left channel (mono mode)
        OnlyLeft = 4,
    }

    #[repr(u8)]
    pub enum CommFormat {
        /// I2S communication I2S Philips standard, data launch at second BCK
        I2s = 0x01,
        /// I2S communication MSB alignment standard, data launch at first BCK
        Msb = 0x02,
        /// PCM Short standard, also known as DSP mode. The period of synchronization signal (WS) is 1 bck cycle.
        PcmShort = 0x04,
        /// PCM Long standard. The period of synchronization signal (WS) is channel_bit*bck cycles.
        PcmLong = 0x0C,
    }
}

pub trait I2s: Send {
    fn port() -> i2s_port_t;
}

pub trait I2sCommFormat<M: SaiCommMode> {
    fn get_comm_format() -> config::CommFormat;
}


pub trait I2sConfigure<'d, I2S, M, W>
where
    I2S: I2s + I2sCommFormat<M>,
    M: SaiCommMode,
    W: Sized,
{
    fn configure(
        bck: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        data_in: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        data_out: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        mck: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        sample_rate: u32,
    ) -> Result<(), EspError> {
        let tranceiver_mode = match (&data_in, &data_out) {
            (Some(_), Some(_)) => config::Mode::Rx as u32 | config::Mode::Tx as u32,
            (Some(_), None) => config::Mode::Rx as u32,
            (None, Some(_)) => config::Mode::Tx as u32,
            (None, None) => unreachable!(),
        };
        crate::into_ref!(bck, ws);
        let mck_pin = mck.map_or(-1, |pin| pin.into_ref().pin());
        let data_in_pin = data_in.map_or(-1, |pin| pin.into_ref().pin());
        let data_out_pin = data_out.map_or(-1, |pin| pin.into_ref().pin());
    
        let pin_config = i2s_pin_config_t {
            bck_io_num: bck.pin(),
            ws_io_num: ws.pin(),
            data_in_num: data_in_pin,
            data_out_num: data_out_pin,
            mck_io_num: mck_pin,
        };
    
        let i2s_config = i2s_driver_config_t {
            mode: config::Mode::Master as u32 | tranceiver_mode,
            sample_rate: sample_rate,
            bits_per_sample: config::BitsPerSample::from(core::mem::size_of::<W>()) as u32,
            channel_format: config::ChannelFormat::RightLeft as u32,
            communication_format: I2S::get_comm_format() as u32,
            intr_alloc_flags: ESP_INTR_FLAG_LEVEL1 as i32,
            dma_buf_count: 8,
            dma_buf_len: 64,
            use_apll: false,
            ..Default::default()
        };
    
        esp!(unsafe { i2s_driver_install(I2S::port(), &i2s_config, 0, ptr::null_mut()) })?;
        esp!(unsafe { i2s_set_pin(I2S::port(), &pin_config) })
    }
}

pub trait I2sRx<'d, I2S, M=I2sMode, W=i16>: I2sConfigure<'d, I2S, M, W>
where
    I2S: I2s + I2sCommFormat<M>,
    M: SaiCommMode,
    W: Sized,
{
    fn new_rx<TPin: Peripheral<P = TPinMode> + 'd, TPinMode: InputPin + OutputPin>(
        i2s: impl Peripheral<P = I2S> + 'd,
        bck: TPin,
        ws: TPin,
        data_in: TPin,
        mck: Option<TPin>,
        sample_rate: u32,
    ) -> Result<I2sDriver<'d, I2S, M, W>, EspError>
    where
        TPin: Peripheral<P = TPinMode>,
        TPinMode: InputPin + OutputPin;

    fn read<'w>(&mut self, samples: &'w mut [W]) -> Result<(), EspError>;
}

pub trait I2sTx<'d, I2S, M=I2sMode, W=i16>: I2sConfigure<'d, I2S, M, W>
where
    I2S: I2s + I2sCommFormat<M>,
    M: SaiCommMode,
    W: Sized,
{
    fn new_tx<TPin: Peripheral<P = TPinMode> + 'd, TPinMode: InputPin + OutputPin>(
        i2s: impl Peripheral<P = I2S> + 'd,
        bck: TPin,
        ws: TPin,
        data_out: TPin,
        mck: Option<TPin>,
        sample_rate: u32,
    ) -> Result<I2sDriver<'d, I2S, M, W>, EspError>
    where
        TPin: Peripheral<P = TPinMode>,
        TPinMode: InputPin + OutputPin;

    fn write<'w>( &mut self, samples: &'w [W]) -> Result<(), EspError>;
}

pub trait I2sRxTx<'d, I2S, M=I2sMode, W=i16>:
    I2sConfigure<'d, I2S, M, W> +
    I2sRx<'d, I2S, M, W> +
    I2sTx<'d, I2S, M, W> 
where
    I2S: I2s + I2sCommFormat<M>,
    M: SaiCommMode,
{
    fn new<TPin: Peripheral<P = TPinMode> + 'd, TPinMode: InputPin + OutputPin>(
        i2s: impl Peripheral<P = I2S> + 'd,
        bck: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        data_in: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        data_out: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        mck: Option<TPin>,
        sample_rate: u32,
    ) -> Result<I2sDriver<'d, I2S, M, W>, EspError>
    where
        TPin: Peripheral<P = TPinMode>,
        TPinMode: InputPin + OutputPin;
}


// pub trait SaiTranceiverMode<I2S: I2s> {}
// impl<'d, I2S, M> SaiTranceiverMode<I2S> for I2sRx<'d, I2S, M>
// where
//     I2S: I2s,
//     M: SaiCommMode,
// {}
// impl<'d, I2S, M> SaiTranceiverMode for I2sTx<'d, I2s, M>
// where

// {}
// impl<'d, I2S, M> SaiTranceiverMode for I2sRxTx<'d, I2s, M>
// where

// {}


pub struct I2sDriver<'d, I2S, M, W>
where
    I2S: I2s,
    M: SaiCommMode,
    W: Sized,
{
    _i2s: PeripheralRef<'d, I2S>,
    _comm_mode: PhantomData<M>,
    _sample_size: PhantomData<W>,
}

// impl<'d, I2S, M, W> I2sDriver<'d, I2S, M, W>
// where
//     I2S: I2s + I2sCommFormat<M>,
//     M: SaiCommMode,
//     W: Sized,
// {
//     fn read<'w>(&mut self, samples: &'w mut [i16]) -> Result<(), EspError> {
//         Ok(())
//     }

//     fn write<'w>( &mut self, samples: &'w [i16]) -> Result<(), EspError> {
//         let mut _bytes_written = 0;
//         esp!(unsafe {  i2s_write(I2S::port(), samples.as_ptr() as *const c_void, 512, &mut _bytes_written, 100) })
//     }
// }


impl<'d, I2S, M, W> I2sRx<'d, I2S, M, W> for I2sDriver<'d, I2S, M, W>
where
    I2S: I2s + I2sCommFormat<M>,
    M: SaiCommMode,
    W: Sized,
{
    fn new_rx<TPin: Peripheral<P = TPinMode> + 'd, TPinMode: InputPin + OutputPin>(
        i2s: impl Peripheral<P = I2S> + 'd,
        bck: TPin,
        ws: TPin,
        data_in: TPin,
        mck: Option<TPin>,
        sample_rate: u32,
    ) -> Result<I2sDriver<'d, I2S, M, W>, EspError>
    where
        TPin: Peripheral<P = TPinMode>,
        TPinMode: InputPin + OutputPin,
    {
        let i2s_ref = i2s.into_ref();
        Self::configure(bck, ws, Some(data_in), None::<TPin>, mck, sample_rate)?;

        // let mck_pin = mck.map_or(-1, |pin| pin.into_ref().pin());
    
        // let pin_config = i2s_pin_config_t {
        //     bck_io_num: bck.pin(),
        //     ws_io_num: ws.pin(),
        //     data_in_num: data_in.pin(),
        //     data_out_num: -1,
        //     mck_io_num: mck_pin,
        // };
    
        // let i2s_config = i2s_driver_config_t {
        //     mode: config::Mode::Master as u32 | config::Mode::Rx as u32,
        //     sample_rate: sample_rate,
        //     bits_per_sample: config::BitsPerSample::from(core::mem::size_of::<W>()) as u32,
        //     channel_format: config::ChannelFormat::RightLeft as u32,
        //     communication_format: I2S::get_comm_format() as u32,
        //     intr_alloc_flags: ESP_INTR_FLAG_LEVEL1 as i32,
        //     dma_buf_count: 8,
        //     dma_buf_len: 64,
        //     use_apll: false,
        //     ..Default::default()
        // };
    
        // esp!(unsafe { i2s_driver_install(I2S::port(), &i2s_config, 0, ptr::null_mut()) })?;
        // esp!(unsafe { i2s_set_pin(I2S::port(), &pin_config) })?;
    
        Ok(I2sDriver { _i2s: i2s_ref, _sample_size: PhantomData, _comm_mode: PhantomData })
    }

    fn read<'w>(&mut self, _samples: &'w mut [W]) -> Result<(), EspError> {
        Ok(())
    }
}

impl<'d, I2S, M, W> I2sTx<'d, I2S, M, W> for I2sDriver<'d, I2S, M, W>
where
    I2S: I2s + I2sCommFormat<M>,
    M: SaiCommMode,
    W: Sized,
{
    fn new_tx<TPin, TPinMode>(
        i2s: impl Peripheral<P = I2S> + 'd,
        bck: TPin,
        ws: TPin,
        data_out: TPin,
        mck: Option<TPin>,
        sample_rate: u32,
    ) -> Result<I2sDriver<'d, I2S, M, W>, EspError>
    where
        TPin: Peripheral<P = TPinMode> + 'd,
        TPinMode: InputPin + OutputPin,
    {
        // crate::into_ref!(i2s, bck, ws, data_out);
        let i2s_ref = i2s.into_ref();
        Self::configure(bck, ws, None::<TPin>, Some(data_out), mck, sample_rate)?;
        
        // let mck_pin = mck.map_or(-1, |pin| pin.into_ref().pin());
    
        // let pin_config = i2s_pin_config_t {
        //     bck_io_num: bck.pin(),
        //     ws_io_num: ws.pin(),
        //     data_in_num: -1,
        //     data_out_num: data_out.pin(),
        //     mck_io_num: mck_pin,
        // };
    
    
        // let i2s_config = i2s_driver_config_t {
        //     mode: config::Mode::Master as u32 | config::Mode::Rx as u32,
        //     sample_rate: sample_rate,
        //     bits_per_sample: config::BitsPerSample::from(core::mem::size_of::<W>()) as u32,
        //     channel_format: config::ChannelFormat::RightLeft as u32,
        //     communication_format: I2S::get_comm_format() as u32,
        //     intr_alloc_flags: ESP_INTR_FLAG_LEVEL1 as i32,
        //     dma_buf_count: 8,
        //     dma_buf_len: 64,
        //     use_apll: false,
        //     ..Default::default()
        // };
    
        // esp!(unsafe { i2s_driver_install(I2S::port(), &i2s_config, 0, ptr::null_mut()) })?;
        // esp!(unsafe { i2s_set_pin(I2S::port(), &pin_config) })?;
    
        Ok(I2sDriver { _i2s: i2s_ref, _sample_size: PhantomData, _comm_mode: PhantomData })
    }

    fn write<'w>( &mut self, samples: &'w [W]) -> Result<(), EspError> {
        let mut _bytes_written = 0;
        esp!(unsafe {  i2s_write(I2S::port(), samples.as_ptr() as *const c_void, samples.len() as u32, &mut _bytes_written, 100) })
    }
}

impl<'d, I2S, M, W> I2sRxTx<'d, I2S, M, W> for I2sDriver<'d, I2S, M, W>
where
    I2S: I2s + I2sCommFormat<M>,
    M: SaiCommMode,
    W: Sized,
{
    fn new<TPin, TPinMode>(
        i2s: impl Peripheral<P = I2S> + 'd,
        bck: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        data_in: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        data_out: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        mck: Option<TPin>,
        sample_rate: u32,
    ) -> Result<I2sDriver<'d, I2S, M, W>, EspError>
    where
        TPin: Peripheral<P = TPinMode> + 'd,
        TPinMode: InputPin + OutputPin,
    {
        // crate::into_ref!(i2s, bck, ws, data_in, data_out);
        let i2s_ref = i2s.into_ref();
        Self::configure(bck, ws, Some(data_in), Some(data_out), mck, sample_rate)?;

        // let mck_pin = mck.map_or(-1, |pin| pin.into_ref().pin());
    
        // let pin_config = i2s_pin_config_t {
        //     bck_io_num: bck.pin(),
        //     ws_io_num: ws.pin(),
        //     data_in_num: data_in.pin(),
        //     data_out_num: data_out.pin(),
        //     mck_io_num: mck_pin,
        // };
    
    
        // let i2s_config = i2s_driver_config_t {
        //     mode: config::Mode::Master as u32 | config::Mode::Rx as u32 | config::Mode::Tx as u32,
        //     sample_rate: sample_rate,
        //     bits_per_sample: config::BitsPerSample::from(core::mem::size_of::<W>()) as u32,
        //     channel_format: config::ChannelFormat::RightLeft as u32,
        //     communication_format: I2S::get_comm_format() as u32,
        //     intr_alloc_flags: ESP_INTR_FLAG_LEVEL1 as i32,
        //     dma_buf_count: 8,
        //     dma_buf_len: 64,
        //     use_apll: false,
        //     ..Default::default()
        // };
    
        // esp!(unsafe { i2s_driver_install(I2S::port(), &i2s_config, 0, ptr::null_mut()) })?;
        // esp!(unsafe { i2s_set_pin(I2S::port(), &pin_config) })?;
    
        Ok(I2sDriver { _i2s: i2s_ref, _sample_size: PhantomData, _comm_mode: PhantomData })
    }
}


impl<'d, I2S, M, W> I2sConfigure<'d, I2S, M, W> for I2sDriver<'d, I2S, M, W>
where
    I2S: I2s + I2sCommFormat<M>,
    M: SaiCommMode,
    W: Sized,
{
//     fn configure(
//         i2s: impl Peripheral<P = I2S>,
//         bck: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
//         ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
//         data_in: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
//         data_out: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
//         mck: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
//         sample_rate: u32,
//     ) -> Result<(), EspError> {
//         crate::into_ref!(bck, ws);
//         let mck_pin = mck.map_or(-1, |pin| pin.into_ref().pin());
//         // let tranceiver_mode = match (data_in, data_out) {
//         //     (Some(_), Some(_)) => config::Mode::Rx as u32 | config::Mode::Tx as u32,
//         //     (Some(_), None) => config::Mode::Rx as u32,
//         //     (None, Some(_)) => config::Mode::Tx as u32,
//         //     (None, None) => unreachable!(),
//         // };

//         let pin_config = i2s_pin_config_t {
//             bck_io_num: bck.pin(),
//             ws_io_num: ws.pin(),
//             data_in_num: data_in.pin(),
//             data_out_num: data_out.pin(),
//             mck_io_num: mck_pin,
//         };
    
//         let i2s_config = i2s_driver_config_t {
//             mode: config::Mode::Master as u32 | config::Mode::Rx as u32 | config::Mode::Tx as u32,
//             sample_rate: sample_rate,
//             bits_per_sample: config::BitsPerSample::from(core::mem::size_of::<W>()) as u32,
//             channel_format: config::ChannelFormat::RightLeft as u32,
//             communication_format: I2S::get_comm_format() as u32,
//             intr_alloc_flags: ESP_INTR_FLAG_LEVEL1 as i32,
//             dma_buf_count: 8,
//             dma_buf_len: 64,
//             use_apll: false,
//             ..Default::default()
//         };
    
//         esp!(unsafe { i2s_driver_install(I2S::port(), &i2s_config, 0, ptr::null_mut()) })?;
//         esp!(unsafe { i2s_set_pin(I2S::port(), &pin_config) })?;

//         Ok(())
//     }
}





impl<'d, I2S: I2s, M, W> Drop for I2sDriver<'d, I2S, M, W>
where
    M: SaiCommMode,
{
    fn drop(&mut self) {
        esp!(unsafe { i2s_driver_uninstall(I2S::port()) }).unwrap();
    }
}
unsafe impl<'d, I2S: I2s, M, W> Send for I2sDriver<'d, I2S, M, W>
where
    M: SaiCommMode,
{}

// impl<'d, I2S, M, W> embedded_hal::sai::I2sTx<W>
// for I2sDriver<'d, I2S, M, W>
// where 
//     I2S: I2s,
//     M: SaiMode,
// {}


// impl<'d, I2S, W, M> embedded_hal::sai::SaiTx<M, 2, W>
// for I2sDriver<'d, I2S, M, W>
// where
//     I2S: I2s,
//     M: SaiMode,
// {
//     type Error = SaiError;
//     type Sample = W;

//     fn write<'w>(&mut self, _: [&'w [W]; 2]) -> Result<(), Self::Error> { todo!() }
//     fn write_iter<WI>(&mut self, _: [WI; 2]) -> Result<(), Self::Error>
//     where
//         WI: IntoIterator<Item = W>
//     {
//         todo!()
//     }
// }

// impl<'d, I2S, W> embedded_hal::sai::SaiTxInterlaced<embedded_hal::sai::I2sMode, 2, W>
// for I2sDriver<'d, I2S, embedded_hal::sai::I2sMode, W>
// where 
//     I2S: I2s
// {
//     type Error = SaiError;
//     type Sample = W;

//     fn write_interlaced<'w>(&mut self, _: &'w [W]) -> Result<(), Self::Error> { todo!() }
//     fn write_interlaced_iter<WI>(&mut self, _: WI) -> Result<(), Self::Error>
//     where
//         WI: IntoIterator<Item = W> 
//     {
//         todo!()
//     }
// }



// type Error = SaiError;

// fn read<'w>(&mut self, samples: [&'w mut [i16]; CHANNELS]) -> Result<(), Self::Error> {
//     I2sMasterDriver::read(self, samples).map_err(to_i2s_err)
// }

// fn write<'w>(&mut self, samples: [&'w [i16]; CHANNELS]) -> Result<(), Self::Error> {
//     I2sMasterDriver::write(self, samples).map_err(to_i2s_err)
// }

// fn write_iter<'w, WI>(&mut self, _: [WI; CHANNELS]) -> Result<(), Self::Error>
// where WI: IntoIterator<Item = i16> {
//     todo!()
// }





// impl<'d, I2S> embedded_hal_0_2::blocking::I2S::Read for I2sMasterDriver<'d, I2S>
// where
//     I2S: I2s,
// {
//     type Error = I2sError;

//     fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
//         I2sMasterDriver::read(self, addr, buffer, BLOCK).map_err(to_I2S_err)
//     }
// }

// impl<'d, I2S> embedded_hal_0_2::blocking::I2S::Write for I2sMasterDriver<'d, I2S>
// where
//     I2S: I2s,
// {
//     type Error = I2sError;

//     fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
//         I2sMasterDriver::write(self, addr, bytes, BLOCK).map_err(to_I2S_err)
//     }
// }

// impl<'d, I2S> embedded_hal_0_2::blocking::I2S::WriteRead for I2sMasterDriver<'d, I2S>
// where
//     I2S: I2s,
// {
//     type Error = I2sError;

//     fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
//         I2sMasterDriver::write_read(self, addr, bytes, buffer, BLOCK).map_err(to_I2S_err)
//     }
// }

// impl<'d, I2S> embedded_hal::I2S::ErrorType for I2sMasterDriver<'d, I2S>
// where
//     I2S: I2s,
// {
//     type Error = I2sError;
// }

// impl<'d, I2S, const CHANNELS: usize> embedded_hal::sai::Sai<i16, CHANNELS>
//     for I2sMasterDriver<'d, I2S, CHANNELS>
//     where I2S: I2s
// {
//     type Error = SaiError;

//     fn read<'w>(&mut self, samples: [&'w mut [i16]; CHANNELS]) -> Result<(), Self::Error> {
//         I2sMasterDriver::read(self, samples).map_err(to_i2s_err)
//     }

//     fn write<'w>(&mut self, samples: [&'w [i16]; CHANNELS]) -> Result<(), Self::Error> {
//         I2sMasterDriver::write(self, samples).map_err(to_i2s_err)
//     }

//     fn write_iter<'w, WI>(&mut self, _: [WI; CHANNELS]) -> Result<(), Self::Error>
//     where WI: IntoIterator<Item = i16> {
//         todo!()
//     }
// }

fn to_i2s_err(err: EspError) -> SaiError {
    SaiError::other(err)
}

// pub struct I2SSlaveDriver<'d, I2S>
// where
//     I2S: I2s,
// {
//     _I2S: PeripheralRef<'d, I2S>,
// }

// unsafe impl<'d, I2S: I2s> Send for I2SSlaveDriver<'d, I2S> {}

// impl<'d, I2S> I2SSlaveDriver<'d, I2S>
// where
//     I2S: I2s,
// {
//     pub fn new(
//         I2S: impl Peripheral<P = I2S> + 'd,
//         sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
//         scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
//         slave_addr: u8,
//         config: &config::SlaveConfig,
//     ) -> Result<Self, EspError> {
//         crate::into_ref!(I2S, sda, scl);

//         #[cfg(not(esp_idf_version = "4.3"))]
//         let sys_config = I2S_config_t {
//             mode: I2S_mode_t_I2S_MODE_SLAVE,
//             sda_io_num: sda.pin(),
//             sda_pullup_en: config.sda_pullup_enabled,
//             scl_io_num: scl.pin(),
//             scl_pullup_en: config.scl_pullup_enabled,
//             __bindgen_anon_1: I2S_config_t__bindgen_ty_1 {
//                 slave: I2S_config_t__bindgen_ty_1__bindgen_ty_2 {
//                     slave_addr: slave_addr as i16,
//                     addr_10bit_en: 0, // For now; to become configurable with embedded-hal V1.0
//                     maximum_speed: 0,
//                 },
//             },
//             ..Default::default()
//         };

//         #[cfg(esp_idf_version = "4.3")]
//         let sys_config = I2S_config_t {
//             mode: I2S_mode_t_I2S_MODE_SLAVE,
//             sda_io_num: pins.sda.pin(),
//             sda_pullup_en: config.sda_pullup_enabled,
//             scl_io_num: pins.scl.pin(),
//             scl_pullup_en: config.scl_pullup_enabled,
//             __bindgen_anon_1: I2S_config_t__bindgen_ty_1 {
//                 slave: I2S_config_t__bindgen_ty_1__bindgen_ty_2 {
//                     slave_addr: slave_addr as i16,
//                     addr_10bit_en: 0, // For now; to become configurable with embedded-hal V1.0
//                 },
//             },
//             ..Default::default()
//         };

//         esp!(unsafe { I2S_param_config(I2S::port(), &sys_config) })?;

//         esp!(unsafe {
//             I2S_driver_install(
//                 I2S::port(),
//                 I2S_mode_t_I2S_MODE_SLAVE,
//                 config.rx_buf_len as u32,
//                 config.tx_buf_len as u32,
//                 0, // TODO: set flags
//             )
//         })?;

//         Ok(Self { _I2S: I2S })
//     }

//     pub fn read(&mut self, buffer: &mut [u8], timeout: TickType_t) -> Result<usize, EspError> {
//         let n = unsafe {
//             I2S_slave_read_buffer(
//                 I2S::port(),
//                 buffer.as_mut_ptr(),
//                 buffer.len() as u32,
//                 timeout,
//             )
//         };

//         if n > 0 {
//             Ok(n as usize)
//         } else {
//             Err(EspError::from(ESP_ERR_TIMEOUT).unwrap())
//         }
//     }

//     pub fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<usize, EspError> {
//         let n = unsafe {
//             I2S_slave_write_buffer(
//                 I2S::port(),
//                 bytes.as_ptr() as *const u8 as *mut u8,
//                 bytes.len() as i32,
//                 timeout,
//             )
//         };

//         if n > 0 {
//             Ok(n as usize)
//         } else {
//             Err(EspError::from(ESP_ERR_TIMEOUT).unwrap())
//         }
//     }
// }

// #[repr(u32)]
// enum AckType {
//     Ack = I2S_ack_type_t_I2S_MASTER_ACK,
//     #[allow(dead_code)]
//     Nack = I2S_ack_type_t_I2S_MASTER_NACK,
//     LastNack = I2S_ack_type_t_I2S_MASTER_LAST_NACK,
// }

// struct CommandLink<'buffers>(I2S_cmd_handle_t, PhantomData<&'buffers u8>);

// impl<'buffers> CommandLink<'buffers> {
//     fn new() -> Result<Self, EspError> {
//         let handle = unsafe { I2S_cmd_link_create() };

//         if handle.is_null() {
//             return Err(EspError::from(ESP_ERR_NO_MEM).unwrap());
//         }

//         Ok(CommandLink(handle, PhantomData))
//     }

//     fn master_start(&mut self) -> Result<(), EspError> {
//         esp!(unsafe { I2S_master_start(self.0) })
//     }

//     fn master_stop(&mut self) -> Result<(), EspError> {
//         esp!(unsafe { I2S_master_stop(self.0) })
//     }

//     fn master_write_byte(&mut self, data: u8, ack_en: bool) -> Result<(), EspError> {
//         esp!(unsafe { I2S_master_write_byte(self.0, data, ack_en) })
//     }

//     fn master_write(&mut self, buf: &'buffers [u8], ack_en: bool) -> Result<(), EspError> {
//         esp!(unsafe {
//             I2S_master_write(
//                 self.0,
//                 buf.as_ptr() as *const u8 as *mut u8,
//                 buf.len() as u32,
//                 ack_en,
//             )
//         })
//     }

//     fn master_read(&mut self, buf: &'buffers mut [u8], ack: AckType) -> Result<(), EspError> {
//         esp!(unsafe {
//             I2S_master_read(
//                 self.0,
//                 buf.as_ptr() as *const u8 as *mut u8,
//                 buf.len() as u32,
//                 ack as u32,
//             )
//         })
//     }
// }

// impl<'buffers> Drop for CommandLink<'buffers> {
//     fn drop(&mut self) {
//         unsafe {
//             I2S_cmd_link_delete(self.0);
//         }
//     }
// }

macro_rules! impl_I2S {
    ($I2S:ident: $port:expr, [$(($mode:ty => $format:expr)),+$(,)?]) => {
        crate::impl_peripheral!($I2S);

        impl I2s for $I2S {
            #[inline(always)]
            fn port() -> i2s_port_t {
                $port
            }
        }

        $(
            impl I2sCommFormat<$mode> for $I2S {
                #[inline(always)]
                fn get_comm_format() -> config::CommFormat {
                    $format
                }
            }
        )+
    };
}

use embedded_hal::sai::{I2sMode, I2sLeftMode, TdmMode};
use config::CommFormat::*;
impl_I2S!(I2S0: 0, [
    (I2sMode => I2s),
    (I2sLeftMode => Msb),
    (TdmMode => PcmShort),
]);
impl_I2S!(I2S1: 1, [
    (I2sMode => I2s),
    (I2sLeftMode => Msb),
    (TdmMode => PcmShort),
]);
