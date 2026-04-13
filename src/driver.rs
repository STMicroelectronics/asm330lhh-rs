use super::{
    BusOperation, DelayNs, I2c, RegisterOperation, SensorOperation, SevenBitAddress, SpiDevice,
    bisync, i2c, prelude::*, register::BankState, spi,
};

use core::fmt::Debug;
use core::marker::PhantomData;

/// The Asm330lhh generic driver struct.
#[bisync]
pub struct Asm330lhh<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: BankState,
{
    pub bus: B,
    pub tim: T,
    _state: PhantomData<S>,
}

/// Driver errors.
#[derive(Debug)]
#[bisync]
pub enum Error<B> {
    Bus(B), // Error at the bus level
    InvalidFsmNumber,
    BufferTooSmall,
    FailedToReadMemBank,
    FailedToSetMemBank(MemBank),
}

#[bisync]
impl<P, T> Asm330lhh<i2c::I2cBus<P>, T, MainBank>
where
    P: I2c,
    T: DelayNs,
{
    /// Constructor method for using the I2C bus.
    ///
    /// # Arguments
    ///
    /// * `i2c`: The I2C peripheral.
    /// * `address`: The I2C address of the Asm330lhh sensor.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Self`: Returns an instance of `Asm330lhh`.
    ///     * `Err`: Returns an error if the initialization fails.
    pub fn new_i2c(i2c: P, address: I2CAddress, tim: T) -> Self {
        // Initialize the I2C bus with the Asm330lhh address
        let bus = i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<P, T> Asm330lhh<spi::SpiBus<P>, T, MainBank>
where
    P: SpiDevice,
    T: DelayNs,
{
    /// Constructor method for using the SPI bus.
    ///
    /// # Arguments
    ///
    /// * `spi`: The SPI peripheral.
    ///
    /// # Returns
    ///
    /// * `Self`: Returns an instance of `Asm330lhh`.
    pub fn new_spi(spi: P, tim: T) -> Self {
        // Initialize the SPI bus
        let bus = spi::SpiBus::new(spi);
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}
#[bisync]
impl<B, T, S> Asm330lhh<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: BankState,
{
    /// Constructor method from a generic bus that implements
    /// the BusOperation trait.
    ///
    /// # Arguments
    ///
    /// * `spi`: The SPI peripheral.
    ///
    /// # Returns
    ///
    /// * `Self`: Returns an instance of `Asm330lhh`.
    #[inline]
    pub fn from_bus(bus: B, tim: T) -> Self {
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<B: BusOperation, T: DelayNs, S: BankState> SensorOperation for Asm330lhh<B, T, S> {
    type Error = Error<B::Error>;

    #[inline]
    async fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .read_from_register(reg, buf)
            .await
            .map_err(Error::Bus)
    }

    #[inline]
    async fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .write_to_register(reg, buf)
            .await
            .map_err(Error::Bus)
    }
}

#[bisync]
impl<B: BusOperation, T: DelayNs> Asm330lhh<B, T, MainBank> {
    /// Accelerometer full-scale selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fs_xl in reg CTRL1_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_full_scale_set(&mut self, val: FsXl) -> Result<(), Error<B::Error>> {
        let mut ctrl1_xl = Ctrl1Xl::read(self).await?;
        ctrl1_xl.set_fs_xl(val as u8);
        ctrl1_xl.write(self).await
    }

    /// Accelerometer full-scale selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Asm330lhhFsXl`: Get the values of fs_xl in reg CTRL1_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_full_scale_get(&mut self) -> Result<FsXl, Error<B::Error>> {
        let ctrl1_xl = Ctrl1Xl::read(self).await?;
        Ok(FsXl::try_from(ctrl1_xl.fs_xl()).unwrap_or_default())
    }

    /// Accelerometer UI data rate selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odr_xl in reg CTRL1_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_data_rate_set(&mut self, val: OdrXl) -> Result<(), Error<B::Error>> {
        let mut ctrl1xl = Ctrl1Xl::read(self).await?;
        ctrl1xl.set_odr_xl(val as u8);
        ctrl1xl.write(self).await
    }

    /// Accelerometer UI data rate selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OdrXl`: Get the values of odr_xl in reg CTRL1_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_data_rate_get(&mut self) -> Result<OdrXl, Error<B::Error>> {
        let ctrl1xl = Ctrl1Xl::read(self).await?;
        Ok(OdrXl::try_from(ctrl1xl.odr_xl()).unwrap_or_default())
    }

    /// Gyroscope UI chain full-scale selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fs_g in reg CTRL2_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_full_scale_set(&mut self, val: FsGy) -> Result<(), Error<B::Error>> {
        let mut ctrl2_g = Ctrl2G::read(self).await?;
        ctrl2_g.set_fs_g(val as u8);
        ctrl2_g.write(self).await
    }

    /// Gyroscope UI chain full-scale selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `FsG`: Get the values of fs_g in reg CTRL2_G.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_full_scale_get(&mut self) -> Result<FsGy, Error<B::Error>> {
        let ctrl2_g = Ctrl2G::read(self).await?;
        Ok(FsGy::try_from(ctrl2_g.fs_g()).unwrap_or_default())
    }

    /// Gyroscope data rate.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odr_g in reg CTRL2_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_data_rate_set(&mut self, val: OdrGy) -> Result<(), Error<B::Error>> {
        let mut ctrl2g = Ctrl2G::read(self).await?;
        ctrl2g.set_odr_g(val as u8);
        ctrl2g.write(self).await
    }

    /// Gyroscope data rate.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OdrG`: Get the values of odr_g in reg CTRL2_G.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_data_rate_get(&mut self) -> Result<OdrGy, Error<B::Error>> {
        let ctrl2_g = Ctrl2G::read(self).await?;
        Ok(OdrGy::try_from(ctrl2_g.odr_g()).unwrap_or_default())
    }

    /// Block data update.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of bdu in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    pub async fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self).await?;
        ctrl3_c.set_bdu(val);
        ctrl3_c.write(self).await
    }

    /// Block data update.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of bdu in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self).await?;
        Ok(ctrl3_c.bdu())
    }

    /// Weight of XL user offset bits of registers X_OFS_USR (73h),
    /// Y_OFS_USR (74h), Z_OFS_USR (75h).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of usr_off_w in reg CTRL6_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_offset_weight_set(&mut self, val: UsrOffW) -> Result<(), Error<B::Error>> {
        let mut ctrl6_c = Ctrl6C::read(self).await?;
        ctrl6_c.set_usr_off_w(val as u8);
        ctrl6_c.write(self).await
    }

    /// Weight of XL user offset bits of registers X_OFS_USR (73h),
    /// Y_OFS_USR (74h), Z_OFS_USR (75h).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `UsrOffW`: Get the values of usr_off_w in reg CTRL6_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_offset_weight_get(&mut self) -> Result<UsrOffW, Error<B::Error>> {
        let ctrl6_c = Ctrl6C::read(self).await?;
        Ok(UsrOffW::try_from(ctrl6_c.usr_off_w()).unwrap_or_default())
    }

    /// Read all the interrupt flag of the device.
    ///
    /// # Arguments
    ///
    /// * `val`: Get registers ALL_INT_SRC; WAKE_UP_SRC; D6D_SRC; STATUS_REG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn all_sources_get(&mut self) -> Result<AllSources, Error<B::Error>> {
        let sources = AllSources {
            all_int_src: AllIntSrc::read(self).await?,
            wake_up_src: WakeUpSrc::read(self).await?,
            d6d_src: D6dSrc::read(self).await?,
            status_reg: StatusReg::read(self).await?,
        };

        Ok(sources)
    }

    /// The STATUS_REG register is read by the primary interface.
    pub async fn status_reg_get(&mut self) -> Result<StatusReg, Error<B::Error>> {
        StatusReg::read(self).await
    }

    /// Accelerometer new data available.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of xlda in reg STATUS_REG.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.status_reg_get().await?.xlda())
    }

    /// Gyroscope new data available.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of gda in reg STATUS_REG.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.status_reg_get().await?.gda())
    }

    /// Temperature new data available.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tda in reg STATUS_REG.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn temp_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.status_reg_get().await?.tda())
    }

    /// Accelerometer X-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_x_set(&mut self, val: i8) -> Result<(), Error<B::Error>> {
        XOfsUsr::new().with_x_ofs_usr(val).write(self).await
    }

    /// Accelerometer X-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_x_get(&mut self) -> Result<i8, Error<B::Error>> {
        XOfsUsr::read(self).await.map(|reg| reg.x_ofs_usr())
    }

    /// Accelerometer Y-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_y_set(&mut self, val: i8) -> Result<(), Error<B::Error>> {
        YOfsUsr::new().with_y_ofs_usr(val).write(self).await
    }

    /// Accelerometer Y-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_y_get(&mut self) -> Result<i8, Error<B::Error>> {
        YOfsUsr::read(self).await.map(|reg| reg.y_ofs_usr())
    }

    /// Accelerometer Z-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_z_set(&mut self, val: i8) -> Result<(), Error<B::Error>> {
        ZOfsUsr::new().with_z_ofs_usr(val).write(self).await
    }

    /// Accelerometer X-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_z_get(&mut self) -> Result<i8, Error<B::Error>> {
        ZOfsUsr::read(self).await.map(|reg| reg.z_ofs_usr())
    }

    /// Enables user offset on out.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of usr_off_on_out in reg CTRL7_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl7_g = Ctrl7G::read(self).await?;
        ctrl7_g.set_usr_off_on_out(val);
        ctrl7_g.write(self).await
    }

    /// Get user offset on out flag.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Get values of usr_off_on_out in reg CTRL7_G.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl7_g = Ctrl7G::read(self).await?;
        Ok(ctrl7_g.usr_off_on_out())
    }

    /// Reset timestamp counter.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn timestamp_rst(&mut self) -> Result<(), Error<B::Error>> {
        self.write_to_register(Reg::Timestamp2 as u8, &[0xAA])
            .await?;
        self.tim.delay_us(150).await; // AN5398 Section 6.4
        Ok(())
    }

    /// Enables timestamp counter.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of timestamp_en in reg CTRL10_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn timestamp_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl10_c = Ctrl10C::read(self).await?;
        ctrl10_c.set_timestamp_en(val);
        ctrl10_c.write(self).await
    }

    /// Enables timestamp counter.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of timestamp_en in reg CTRL10_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn timestamp_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl10_c = Ctrl10C::read(self).await?;
        Ok(ctrl10_c.timestamp_en())
    }

    /// Timestamp first data output register (r).
    /// The value is expressed as a 32-bit word and the bit resolution
    /// is 25 us.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u32`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn timestamp_raw_get(&mut self) -> Result<u32, Error<B::Error>> {
        Ok(TimestampReg::read(self).await?.timestamp())
    }

    /// Circular burst-mode (rounding) read of the output registers.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of rounding in reg CTRL5_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn rounding_mode_set(&mut self, val: Rounding) -> Result<(), Error<B::Error>> {
        let mut ctrl5_c = Ctrl5C::read(self).await?;
        ctrl5_c.set_rounding(val as u8);
        ctrl5_c.write(self).await
    }

    /// Gyroscope UI chain full-scale selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Rounding`: Get the values of rounding in reg CTRL5_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn rounding_mode_get(&mut self) -> Result<Rounding, Error<B::Error>> {
        let ctrl5_c = Ctrl5C::read(self).await?;
        Ok(Rounding::try_from(ctrl5_c.rounding()).unwrap_or_default())
    }

    /// Temperature data output register (r).
    /// L and H registers together express a 16-bit word in two's
    /// complement.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i16`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(OutTempReg::read(self).await?.temp())
    }

    /// Angular rate sensor. The value is expressed as a 16-bit
    /// word in two's complement.
    ///
    /// # Arguments
    ///
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `[i16;3]`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn angular_rate_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZG::read(self).await?;

        Ok([val.x, val.y, val.z])
    }

    /// Linear acceleration output register. The value is expressed as a
    /// 16-bit word in two's complement.
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn acceleration_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZA::read(self).await?;

        Ok([val.x, val.y, val.z])
    }

    /// FIFO data output.
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub async fn fifo_out_raw_get(&mut self) -> Result<[u8; 6], Error<B::Error>> {
        let mut buff = [0_u8; 6];
        self.read_from_register(Reg::FifoDataOutXL as u8, &mut buff)
            .await?;
        Ok(buff)
    }

    /// DEVICE_CONF bit configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of device_conf in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn device_conf_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self).await?;
        ctrl9_xl.set_device_conf(val);
        ctrl9_xl.write(self).await
    }

    /// DEVICE_CONF bit configuration
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of device_conf in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub async fn device_conf_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self).await?;
        Ok(ctrl9_xl.device_conf())
    }

    /// Difference in percentage of the effective ODR (and timestamp rate)
    /// with respect to the typical.
    /// Step:  0.15%. 8-bit format, 2's complement.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i8`: Change the values of freq_fine in reg INTERNAL_FREQ_FINE.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn odr_cal_reg_get(&mut self) -> Result<i8, Error<B::Error>> {
        Ok(InternalFreqFine::read(self).await?.freq_fine())
    }

    /// Data-ready pulsed / latched mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of dataready_pulsed in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn data_ready_mode_set(
        &mut self,
        val: DatareadyPulsed,
    ) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg1 = CounterBdrReg::read(self).await?;
        counter_bdr_reg1.set_dataready_pulsed(val as u8);
        counter_bdr_reg1.write(self).await
    }

    /// Data-ready pulsed / latched mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of dataready_pulsed in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn data_ready_mode_get(&mut self) -> Result<DatareadyPulsed, Error<B::Error>> {
        let counter_bdr_reg1 = CounterBdrReg::read(self).await?;
        Ok(DatareadyPulsed::try_from(counter_bdr_reg1.dataready_pulsed()).unwrap_or_default())
    }

    /// Device Who am I.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn device_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).await.map(|reg| reg.id())
    }

    /// Software reset. Restore the default values in user registers.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sw_reset in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub async fn reset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self).await?;
        ctrl3_c.set_sw_reset(val);
        ctrl3_c.write(self).await
    }

    /// Software reset. Restore the default values in user registers.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of sw_reset in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self).await?;
        Ok(ctrl3_c.sw_reset())
    }

    /// Register address automatically incremented during a multiple byte
    /// access with a serial interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of if_inc in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn auto_increment_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self).await?;
        ctrl3_c.set_if_inc(val);
        ctrl3_c.write(self).await
    }

    /// Register address automatically incremented during a multiple byte
    /// access with a serial interface.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of if_inc in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn auto_increment_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self).await?;
        Ok(ctrl3_c.if_inc())
    }

    /// Reboot memory content. Reload the calibration parameters.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of boot in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub async fn boot_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self).await?;
        ctrl3_c.set_boot(val);
        ctrl3_c.write(self).await
    }

    /// Reboot memory content. Reload the calibration parameters.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of boot in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn boot_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self).await?;
        Ok(ctrl3_c.boot())
    }

    /// Linear acceleration sensor self-test enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of st_xl in reg CTRL5_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_self_test_set(&mut self, val: StXl) -> Result<(), Error<B::Error>> {
        let mut ctrl5_c = Ctrl5C::read(self).await?;
        ctrl5_c.set_st_xl(val as u8);
        ctrl5_c.write(self).await
    }

    /// Linear acceleration sensor self-test enable.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `StXl`: Get the values of st_xl in reg CTRL5_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_self_test_get(&mut self) -> Result<StXl, Error<B::Error>> {
        let ctrl5_c = Ctrl5C::read(self).await?;
        Ok(StXl::try_from(ctrl5_c.st_xl()).unwrap_or_default())
    }

    /// Angular rate sensor self-test enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of st_g in reg CTRL5_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_self_test_set(&mut self, val: StGy) -> Result<(), Error<B::Error>> {
        let mut ctrl5_c = Ctrl5C::read(self).await?;
        ctrl5_c.set_st_g(val as u8);
        ctrl5_c.write(self).await
    }

    /// Angular rate sensor self-test enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of st_g in reg CTRL5_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub async fn gy_self_test_get(&mut self) -> Result<StGy, Error<B::Error>> {
        let ctrl5_c = Ctrl5C::read(self).await?;
        Ok(StGy::try_from(ctrl5_c.st_g()).unwrap_or_default())
    }

    /// Accelerometer output from LPF2 filtering stage selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of lpf2_xl_en in reg CTRL1_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_filter_lp2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl1_xl = Ctrl1Xl::read(self).await?;
        ctrl1_xl.set_lpf2_xl_en(val);
        ctrl1_xl.write(self).await
    }

    /// Accelerometer output from LPF2 filtering stage selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of lpf2_xl_en in reg CTRL1_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_filter_lp2_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl1_xl = Ctrl1Xl::read(self).await?;
        Ok(ctrl1_xl.lpf2_xl_en())
    }

    /// Enables gyroscope digital LPF1 if auxiliary SPI is disabled;
    /// the bandwidth can be selected through FTYPE \[2:0\] in CTRL6_C.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of lpf1_sel_g in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_filter_lp1_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self).await?;
        ctrl4_c.set_lpf1_sel_g(val);
        ctrl4_c.write(self).await
    }

    /// Enables gyroscope digital LPF1 if auxiliary SPI is disabled;
    /// the bandwidth can be selected through FTYPE \[2:0\] in CTRL6_C.
    pub async fn gy_filter_lp1_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self).await?;
        Ok(ctrl4_c.lpf1_sel_g())
    }

    /// Mask DRDY on pin (both XL & Gyro) until filter settling ends
    /// (XL and Gyro independently masked).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of drdy_mask in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn drdy_mask_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self).await?;
        ctrl4_c.set_drdy_mask(val);
        ctrl4_c.write(self).await
    }

    /// Mask DRDY on pin (both XL & Gyro) until filter settling ends
    /// (XL and Gyro independently masked).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of drdy_mask in reg CTRL4_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn drdy_mask_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self).await?;
        Ok(ctrl4_c.drdy_mask())
    }

    /// Gyroscope low pass filter 1 bandwidth.
    /// See Table 58 on Datasheet for more information
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ftype in reg CTRL6_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_lp1_bandwidth_set(&mut self, val: Ftype) -> Result<(), Error<B::Error>> {
        let mut ctrl6_c = Ctrl6C::read(self).await?;
        ctrl6_c.set_ftype(val as u8);
        ctrl6_c.write(self).await
    }

    /// Gyroscope low pass filter 1 bandwidth.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Ftype`: Get the values of ftype in reg CTRL6_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_lp1_bandwidth_get(&mut self) -> Result<Ftype, Error<B::Error>> {
        let ctrl6_c = Ctrl6C::read(self).await?;
        Ok(Ftype::try_from(ctrl6_c.ftype()).unwrap_or_default())
    }

    /// Low pass filter 2 on 6D function selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of low_pass_on_6d in reg CTRL8_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_lp2_on_6d_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl8_xl = Ctrl8Xl::read(self).await?;
        ctrl8_xl.set_low_pass_on_6d(val);
        ctrl8_xl.write(self).await
    }

    /// Low pass filter 2 on 6D function selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of low_pass_on_6d in reg CTRL8_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_lp2_on_6d_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl8_xl = Ctrl8Xl::read(self).await?;
        Ok(ctrl8_xl.low_pass_on_6d())
    }

    /// Accelerometer slope filter / high-pass filter selection on output.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of hp_slope_xl_en in reg CTRL8_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_hp_path_on_out_set(&mut self, val: HpSlopeXlEn) -> Result<(), Error<B::Error>> {
        let mut ctrl8_xl = Ctrl8Xl::read(self).await?;

        ctrl8_xl.set_hp_slope_xl_en(((val as u8) & 0x10) >> 4);
        ctrl8_xl.set_hp_ref_mode_xl(((val as u8) & 0x20) >> 5);
        ctrl8_xl.set_hpcf_xl((val as u8) & 0x07);

        ctrl8_xl.write(self).await
    }

    /// Accelerometer slope filter / high-pass filter selection on output.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `HpSlopeXlEn`: Get the values of hp_slope_xl_en in reg CTRL8_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_hp_path_on_out_get(&mut self) -> Result<HpSlopeXlEn, Error<B::Error>> {
        let ctrl8_xl = Ctrl8Xl::read(self).await?;

        let value = (ctrl8_xl.hp_ref_mode_xl() << 5)
            + (ctrl8_xl.hp_slope_xl_en() << 4)
            + ctrl8_xl.hpcf_xl();
        Ok(HpSlopeXlEn::try_from(value).unwrap_or_default())
    }

    /// Enables accelerometer LPF2 and HPF fast-settling mode.
    /// The filter sets the second samples after writing this bit.
    /// Active only during device exit from powerdown mode.
    pub async fn xl_fast_settling_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl8_xl = Ctrl8Xl::read(self).await?;
        ctrl8_xl.set_fastsettl_mode_xl(val);
        ctrl8_xl.write(self).await
    }

    /// Enables accelerometer LPF2 and HPF fast-settling mode.
    /// The filter sets the second samples after writing
    /// this bit. Active only during device exit from powerdown mode.
    pub async fn xl_fast_settling_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl8_xl = Ctrl8Xl::read(self).await?;
        Ok(ctrl8_xl.fastsettl_mode_xl())
    }

    /// Enables gyroscope digital high-pass filter. The filter is enabled
    /// only if the gyro is in HP mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of hp_en_g and hpm_g in reg CTRL7_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_hp_filter_set(&mut self, val: HpmGy) -> Result<(), Error<B::Error>> {
        let mut ctrl7_g = Ctrl7G::read(self).await?;
        ctrl7_g.set_hp_en_g(((val as u8) & 0x80) >> 7);
        ctrl7_g.set_hpm_g((val as u8) & 0x03);
        ctrl7_g.write(self).await
    }

    /// Enables gyroscope digital high-pass filter. The filter is
    /// enabled only if the gyro is in HP mode.
    pub async fn gy_hp_filter_get(&mut self) -> Result<HpmGy, Error<B::Error>> {
        let ctrl7_g = Ctrl7G::read(self).await?;
        let value = (ctrl7_g.hp_en_g() << 7) + ctrl7_g.hpm_g();
        Ok(HpmGy::try_from(value).unwrap_or_default())
    }

    /// Connect/Disconnect SDO/SA0 internal pull-up.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sdo_pu_en in reg PIN_CTRL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn sdo_sa0_mode_set(&mut self, val: SdoPuEn) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self).await?;
        pin_ctrl.set_sdo_pu_en(val as u8);
        pin_ctrl.write(self).await
    }

    /// Connect/Disconnect SDO/SA0 internal pull-up.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of sdo_pu_en in reg PIN_CTRL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SdoPuEn`: The value of sdo_pu_en.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn sdo_sa0_mode_get(&mut self) -> Result<SdoPuEn, Error<B::Error>> {
        let pin_ctrl = PinCtrl::read(self).await?;
        Ok(SdoPuEn::try_from(pin_ctrl.sdo_pu_en()).unwrap_or_default())
    }

    /// SPI Serial Interface Mode selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sim in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn spi_mode_set(&mut self, val: Sim) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self).await?;
        ctrl3_c.set_sim(val as u8);
        ctrl3_c.write(self).await
    }

    /// SPI Serial Interface Mode selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Sim`: Get the values of sim in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn spi_mode_get(&mut self) -> Result<Sim, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self).await?;
        Ok(Sim::try_from(ctrl3_c.sim()).unwrap_or_default())
    }

    /// Disable / Enable I2C interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of i2c_disable in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn i2c_interface_set(&mut self, val: I2cDisable) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self).await?;
        ctrl4_c.set_i2c_disable(val as u8);
        ctrl4_c.write(self).await
    }

    /// Disable / Enable I2C interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of i2c reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Returns no error if successful.
    pub async fn i2c_interface_get(&mut self) -> Result<I2cDisable, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self).await?;
        Ok(I2cDisable::try_from(ctrl4_c.i2c_disable()).unwrap_or_default())
    }

    /// Select the signal that need to route on int1 pad.
    ///
    /// # Arguments
    ///
    /// * `val`: `PinInt1Route`
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn pin_int1_route_set(
        &mut self,
        val: &mut PinInt1Route,
    ) -> Result<(), Error<B::Error>> {
        val.md1_cfg.write(self).await?;

        let mut int1_cfg = IntCfg1::read(self).await?;

        if (val.int1_ctrl.into_bits() | val.md1_cfg.into_bits()) != PROPERTY_DISABLE {
            int1_cfg.set_interrupts_enable(PROPERTY_ENABLE);
        } else {
            int1_cfg.set_interrupts_enable(PROPERTY_DISABLE);
        }

        int1_cfg.write(self).await
    }

    /// Select the signal that need to route on int1 pad.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `PinInt1Route`
    pub async fn pin_int1_route_get(&mut self) -> Result<PinInt1Route, Error<B::Error>> {
        Ok(PinInt1Route {
            int1_ctrl: Int1Ctrl::read(self).await?,
            md1_cfg: Md1Cfg::read(self).await?,
        })
    }

    /// Select the signal that need to route on int2 pad.
    ///
    /// # Arguments
    ///
    /// * `val`: `PinInt2Route`
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn pin_int2_route_set(
        &mut self,
        val: &mut PinInt2Route,
    ) -> Result<(), Error<B::Error>> {
        val.int2_ctrl.write(self).await?;
        val.md2_cfg.write(self).await?;

        let mut int_cfg1 = IntCfg1::read(self).await?;
        let pin_int1 = self.pin_int1_route_get().await?;

        if (val.int2_ctrl.into_bits()
            | val.md2_cfg.into_bits()
            | pin_int1.int1_ctrl.into_bits()
            | pin_int1.md1_cfg.into_bits())
            != PROPERTY_DISABLE
        {
            int_cfg1.set_interrupts_enable(PROPERTY_ENABLE);
        } else {
            int_cfg1.set_interrupts_enable(PROPERTY_DISABLE);
        }

        int_cfg1.write(self).await
    }

    /// Select the signal that need to route on int2 pad.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `PinInt2Route`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn pin_int2_route_get(&mut self) -> Result<PinInt2Route, Error<B::Error>> {
        Ok(PinInt2Route {
            int2_ctrl: Int2Ctrl::read(self).await?,
            md2_cfg: Md2Cfg::read(self).await?,
        })
    }

    /// Push-pull/open drain selection on interrupt pads.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of pp_od in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn pin_mode_set(&mut self, val: PpOd) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self).await?;
        ctrl3_c.set_pp_od(val as u8);
        ctrl3_c.write(self).await
    }

    /// Push-pull/open drain selection on interrupt pads.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `PpOd`: Get the values of pp_od in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn pin_mode_get(&mut self) -> Result<PpOd, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self).await?;
        Ok(PpOd::try_from(ctrl3_c.pp_od()).unwrap_or_default())
    }

    /// Interrupt active-high/low.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of h_lactive in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn pin_polarity_set(&mut self, val: HLactive) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self).await?;
        ctrl3_c.set_h_lactive(val as u8);
        ctrl3_c.write(self).await
    }

    /// Interrupt active-high/low.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of h_lactive in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn pin_polarity_get(&mut self) -> Result<HLactive, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self).await?;
        Ok(HLactive::try_from(ctrl3_c.h_lactive()).unwrap_or_default())
    }

    /// All interrupt signals become available on INT1 pin.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of int2_on_int1 in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub async fn all_on_int1_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self).await?;
        ctrl4_c.set_int2_on_int1(val);
        ctrl4_c.write(self).await
    }

    /// All interrupt signals become available on INT1 pin.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of int2_on_int1 in reg CTRL4_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn all_on_int1_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self).await?;
        Ok(ctrl4_c.int2_on_int1())
    }

    /// All interrupt signals notification mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of lir in reg INT_CFG0.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub async fn int_notification_set(&mut self, val: Lir) -> Result<(), Error<B::Error>> {
        let mut int_cfg0 = IntCfg0::read(self).await?;
        int_cfg0.set_lir((val as u8) & 0x01);
        int_cfg0.set_int_clr_on_read((val as u8) & 0x01);
        int_cfg0.write(self).await?;
        Ok(())
    }

    /// All interrupt signals notification mode.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Lir`: Get the values of lir in reg INT_CFG0.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn int_notification_get(&mut self) -> Result<Lir, Error<B::Error>> {
        let int_cfg0 = IntCfg0::read(self).await?;

        let val = (int_cfg0.lir() << 1) + int_cfg0.int_clr_on_read();
        Ok(Lir::try_from(val).unwrap_or_default())
    }

    /// Weight of 1 LSB of wakeup threshold.
    /// 0: 1 LSB =FS_XL  /  64
    /// 1: 1 LSB = FS_XL / 256
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of wake_ths_w in reg WAKE_UP_DUR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn wkup_ths_weight_set(&mut self, val: WakeThsW) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self).await?;
        wake_up_dur.set_wake_ths_w(val as u8);
        wake_up_dur.write(self).await
    }

    /// Weight of 1 LSB of wakeup threshold.
    ///
    /// 0: 1 LSB = FS_XL  /  64
    /// 1: 1 LSB = FS_XL / 256
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of wake_ths_w in reg WAKE_UP_DUR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn wkup_ths_weight_get(&mut self) -> Result<WakeThsW, Error<B::Error>> {
        let wake_up_dur = WakeUpDur::read(self).await?;
        Ok(WakeThsW::try_from(wake_up_dur.wake_ths_w()).unwrap_or_default())
    }

    /// Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in
    /// WAKE_UP_DUR.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of wk_ths in reg WAKE_UP_THS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn wkup_threshold_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self).await?;
        wake_up_ths.set_wk_ths(val);
        wake_up_ths.write(self).await
    }

    /// Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in WAKE_UP_DUR.
    pub async fn wkup_threshold_get(&mut self) -> Result<u8, Error<B::Error>> {
        let wake_up_ths = WakeUpThs::read(self).await?;
        Ok(wake_up_ths.wk_ths())
    }

    /// Wake up duration event (1LSb = 1 / ODR).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of usr_off_on_wu in reg WAKE_UP_THS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn xl_usr_offset_on_wkup_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self).await?;
        wake_up_ths.set_usr_off_on_wu(val);
        wake_up_ths.write(self).await
    }

    /// Wake up duration event (1LSb = 1 / ODR). Get the values of usr_off_on_wu in reg WAKE_UP_THS.
    pub async fn xl_usr_offset_on_wkup_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(WakeUpThs::read(self).await?.usr_off_on_wu())
    }

    /// Wake up duration event(1LSb = 1 / ODR).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of wake_dur in reg WAKE_UP_DUR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn wkup_dur_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self).await?;
        wake_up_dur.set_wake_dur(val);
        wake_up_dur.write(self).await
    }

    /// Wake up duration event (1LSb = 1 / ODR).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of wake_dur in reg WAKE_UP_DUR.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn wkup_dur_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(WakeUpDur::read(self).await?.wake_dur())
    }

    /// Enables gyroscope Sleep mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sleep_g in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_sleep_mode_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self).await?;
        ctrl4_c.set_sleep_g(val);
        ctrl4_c.write(self).await
    }

    /// Enables gyroscope Sleep mode.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of sleep_g in reg CTRL4_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn gy_sleep_mode_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self).await?;
        Ok(ctrl4_c.sleep_g())
    }

    /// Drives the sleep status instead of sleep change on INT pins
    /// (only if INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits
    /// are enabled).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sleep_status_on_int in reg INT_CFG0.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn act_pin_notification_set(
        &mut self,
        val: SleepStatusOnInt,
    ) -> Result<(), Error<B::Error>> {
        let mut int_cfg0 = IntCfg0::read(self).await?;
        int_cfg0.set_sleep_status_on_int(val as u8);
        int_cfg0.write(self).await
    }

    /// Drives the sleep status instead of sleep change on INT pins
    /// (only if INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits
    /// are enabled).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SleepStatusOnInt`: Get the values of sleep_status_on_int in reg INT_CFG0.
    pub async fn act_pin_notification_get(&mut self) -> Result<SleepStatusOnInt, Error<B::Error>> {
        let int_cfg0 = IntCfg0::read(self).await?;
        Ok(SleepStatusOnInt::try_from(int_cfg0.sleep_status_on_int()).unwrap_or_default())
    }

    /// Enable inactivity function.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of inact_en in reg INT_CFG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn act_mode_set(&mut self, val: InactEn) -> Result<(), Error<B::Error>> {
        let mut int_cfg1 = IntCfg1::read(self).await?;
        int_cfg1.set_inact_en(val as u8);
        int_cfg1.write(self).await
    }

    /// Enable inactivity function.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `InactEn`: Get the values of inact_en in reg INT_CFG1.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn act_mode_get(&mut self) -> Result<InactEn, Error<B::Error>> {
        let int_cfg1 = IntCfg1::read(self).await?;
        Ok(InactEn::try_from(int_cfg1.inact_en()).unwrap_or_default())
    }

    /// Duration to go in sleep mode (1 LSb = 512 / ODR).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sleep_dur in reg WAKE_UP_DUR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub async fn act_sleep_dur_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self).await?;
        wake_up_dur.set_sleep_dur(val);
        wake_up_dur.write(self).await
    }

    /// Duration to go in sleep mode.(1 LSb = 512 / ODR).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of sleep_dur in reg WAKE_UP_DUR.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn act_sleep_dur_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(WakeUpDur::read(self).await?.sleep_dur())
    }

    /// Threshold for 4D/6D function.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sixd_ths in reg THS_6D.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn six_d_threshold_set(&mut self, val: SixdThs) -> Result<(), Error<B::Error>> {
        let mut ths_6d = Ths6d::read(self).await?;
        ths_6d.set_sixd_ths(val as u8);
        ths_6d.write(self).await
    }

    /// Threshold for 4D/6D function.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SixdThs`: Get the values of sixd_ths in reg THS_6D.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn six_d_threshold_get(&mut self) -> Result<SixdThs, Error<B::Error>> {
        let ths_6d = Ths6d::read(self).await?;
        Ok(SixdThs::try_from(ths_6d.sixd_ths()).unwrap_or_default())
    }

    /// 4D orientation detection enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of d4d_en in reg THS_6D.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn four_d_mode_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ths_6d = Ths6d::read(self).await?;
        ths_6d.set_d4d_en(val);
        ths_6d.write(self).await
    }

    /// 4D orientation detection enable.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of d4d_en in reg THS_6D.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn four_d_mode_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ths_6d = Ths6d::read(self).await?;
        Ok(ths_6d.d4d_en())
    }

    /// Free fall threshold setting.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ff_ths in reg FREE_FALL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn ff_threshold_set(&mut self, val: FfThs) -> Result<(), Error<B::Error>> {
        let mut free_fall = FreeFall::read(self).await?;
        free_fall.set_ff_ths(val as u8);
        free_fall.write(self).await
    }

    /// Free fall threshold setting.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `FfThs`: Get the values of ff_ths in reg FREE_FALL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn ff_threshold_get(&mut self) -> Result<FfThs, Error<B::Error>> {
        let free_fall = FreeFall::read(self).await?;
        Ok(FfThs::try_from(free_fall.ff_ths()).unwrap_or_default())
    }

    /// Free-fall duration event(1LSb = 1 / ODR).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ff_dur in reg FREE_FALL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn ff_dur_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self).await?;
        wake_up_dur.set_ff_dur((val & 0x20) >> 5);
        wake_up_dur.write(self).await?;

        let mut free_fall = FreeFall::read(self).await?;
        free_fall.set_ff_dur(val & 0x1F);
        free_fall.write(self).await
    }

    /// Free-fall duration event(1LSb = 1 / ODR).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of ff_dur in reg FREE_FALL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn ff_dur_get(&mut self) -> Result<u8, Error<B::Error>> {
        let wake_up_dur = WakeUpDur::read(self).await?;
        let free_fall = FreeFall::read(self).await?;

        let val = (wake_up_dur.ff_dur() << 5) + free_fall.ff_dur();
        Ok(val)
    }

    /// FIFO watermark level selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of wtm in reg FIFO_CTRL1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_watermark_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl1 = FifoCtrl1::read(self).await?;
        let mut fifo_ctrl2 = FifoCtrl2::read(self).await?;

        let [val_l, val_h] = val.to_le_bytes();
        fifo_ctrl1.set_wtm(val_l);
        fifo_ctrl2.set_wtm(val_h & 0x01);

        fifo_ctrl1.write(self).await?;
        fifo_ctrl2.write(self).await
    }

    /// FIFO watermark level selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Change the values of wtm in reg FIFO_CTRL1.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_watermark_get(&mut self) -> Result<u16, Error<B::Error>> {
        let fifo_ctrl1 = FifoCtrl1::read(self).await?;
        let fifo_ctrl2 = FifoCtrl2::read(self).await?;
        Ok(u16::from_le_bytes([fifo_ctrl1.wtm(), fifo_ctrl2.wtm()]))
    }

    /// Enables ODR CHANGE virtual sensor to be batched in FIFO.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odrchg_en in reg FIFO_CTRL2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_virtual_sens_odr_chg_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self).await?;
        fifo_ctrl2.set_odrchg_en(val);
        fifo_ctrl2.write(self).await
    }

    /// Enables ODR CHANGE virtual sensor to be batched in FIFO.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of odrchg_en in reg FIFO_CTRL2.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_virtual_sens_odr_chg_get(&mut self) -> Result<u8, Error<B::Error>> {
        let fifo_ctrl2 = FifoCtrl2::read(self).await?;
        Ok(fifo_ctrl2.odrchg_en())
    }

    /// Sensing chain FIFO stop values memorization at threshold level.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of stop_on_wtm in reg FIFO_CTRL2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub async fn fifo_stop_on_wtm_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self).await?;
        fifo_ctrl2.set_stop_on_wtm(val);
        fifo_ctrl2.write(self).await
    }

    /// Sensing chain FIFO stop values memorization at threshold level.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of stop_on_wtm in reg FIFO_CTRL2.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_stop_on_wtm_get(&mut self) -> Result<u8, Error<B::Error>> {
        let fifo_ctrl2 = FifoCtrl2::read(self).await?;
        Ok(fifo_ctrl2.stop_on_wtm())
    }

    /// Selects Batching Data Rate (writing frequency in FIFO)
    /// for accelerometer data.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of bdr_xl in reg FIFO_CTRL3.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_xl_batch_set(&mut self, val: BdrXl) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl3 = FifoCtrl3::read(self).await?;
        fifo_ctrl3.set_bdr_xl(val as u8);
        fifo_ctrl3.write(self).await
    }

    /// Selects Batching Data Rate (writing frequency in FIFO)
    /// for accelerometer data.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `BdrXl`: Get the values of bdr_xl in reg FIFO_CTRL3.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_xl_batch_get(&mut self) -> Result<BdrXl, Error<B::Error>> {
        let fifo_ctrl3 = FifoCtrl3::read(self).await?;
        Ok(BdrXl::try_from(fifo_ctrl3.bdr_xl()).unwrap_or_default())
    }

    /// Selects Batching Data Rate (writing frequency in FIFO) for gyroscope data.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of bdr_gy in reg FIFO_CTRL3.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_gy_batch_set(&mut self, val: BdrGy) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl3 = FifoCtrl3::read(self).await?;
        fifo_ctrl3.set_bdr_gy(val as u8);
        fifo_ctrl3.write(self).await
    }

    /// Selects Batching Data Rate (writing frequency in FIFO) for gyroscope data.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `BdrGy`: Get the values of bdr_gy in reg FIFO_CTRL3.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_gy_batch_get(&mut self) -> Result<BdrGy, Error<B::Error>> {
        let fifo_ctrl3 = FifoCtrl3::read(self).await?;
        Ok(BdrGy::try_from(fifo_ctrl3.bdr_gy()).unwrap_or_default())
    }

    /// FIFO mode selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fifo_mode in reg FIFO_CTRL4.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_mode_set(&mut self, val: FifoMode) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self).await?;
        fifo_ctrl4.set_fifo_mode(val as u8);
        fifo_ctrl4.write(self).await
    }

    /// FIFO mode selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `FifoMode`: Get the values of fifo_mode in reg FIFO_CTRL4.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_mode_get(&mut self) -> Result<FifoMode, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self).await?;
        Ok(FifoMode::try_from(fifo_ctrl4.fifo_mode()).unwrap_or_default())
    }

    /// Selects Batching Data Rate (writing frequency in FIFO) for temperature data.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odr_t_batch in reg FIFO_CTRL4.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_temp_batch_set(&mut self, val: OdrTBatch) -> Result<(), Error<B::Error>> {
        let mut ctrl4 = FifoCtrl4::read(self).await?;
        ctrl4.set_odr_t_batch(val as u8);
        ctrl4.write(self).await
    }

    /// Selects Batching Data Rate (writing frequency in FIFO) for temperature data.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OdrTBatch`: Get the values of odr_t_batch in reg FIFO_CTRL4.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_temp_batch_get(&mut self) -> Result<OdrTBatch, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self).await?;
        Ok(OdrTBatch::try_from(fifo_ctrl4.odr_t_batch()).unwrap_or_default())
    }

    /// Selects decimation for timestamp batching in FIFO.
    /// Writing rate will be the maximum rate between XL and
    /// GYRO BDR divided by decimation decoder.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odr_ts_batch in reg FIFO_CTRL4.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No error.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_timestamp_decimation_set(
        &mut self,
        val: OdrTsBatch,
    ) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self).await?;
        fifo_ctrl4.set_dec_ts_batch(val as u8);
        fifo_ctrl4.write(self).await
    }

    /// Selects decimation for timestamp batching in FIFO.
    /// Writing rate will be the maximum rate between XL and
    /// GYRO BDR divided by decimation decoder.
    pub async fn fifo_timestamp_decimation_get(&mut self) -> Result<OdrTsBatch, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self).await?;
        Ok(OdrTsBatch::try_from(fifo_ctrl4.dec_ts_batch()).unwrap_or_default())
    }

    /// Selects the trigger for the internal counter of batching events
    /// between XL and gyro.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of trig_counter_bdr in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_cnt_event_batch_set(
        &mut self,
        val: TrigCounterBdr,
    ) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg1 = CounterBdrReg::read(self).await?;
        counter_bdr_reg1.set_trig_counter_bdr(val as u8);
        counter_bdr_reg1.write(self).await
    }

    /// Selects the trigger for the internal counter of batching events
    /// between XL and gyro.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of trig_counter_bdr
    ///   in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return 0 -> no Error).
    pub async fn fifo_cnt_event_batch_get(&mut self) -> Result<TrigCounterBdr, Error<B::Error>> {
        let counter_bdr_reg1 = CounterBdrReg::read(self).await?;
        Ok(TrigCounterBdr::try_from(counter_bdr_reg1.trig_counter_bdr()).unwrap_or_default())
    }

    /// Resets the internal counter of batching events for a single sensor.
    /// This bit is automatically reset to zero if it was set to '1'.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of rst_counter_bdr in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn rst_batch_counter_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg1 = CounterBdrReg::read(self).await?;
        counter_bdr_reg1.set_rst_counter_bdr(val);
        counter_bdr_reg1.write(self).await
    }

    /// Resets the internal counter of batching events for a single sensor.
    /// This bit is automatically reset to zero if it was set to '1'.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of rst_counter_bdr in reg COUNTER_BDR_REG1.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn rst_batch_counter_get(&mut self) -> Result<u8, Error<B::Error>> {
        let counter_bdr_reg1 = CounterBdrReg::read(self).await?;
        Ok(counter_bdr_reg1.rst_counter_bdr())
    }

    /// Batch data rate counter.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of cnt_bdr_th in reg COUNTER_BDR_REG2 and COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn batch_counter_threshold_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg = CounterBdrReg::read(self).await?;
        counter_bdr_reg.set_cnt_bdr_th(val);
        counter_bdr_reg.write(self).await
    }

    /// Batch data rate counter.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Change the values of cnt_bdr_th in reg COUNTER_BDR_REG2 and COUNTER_BDR_REG1.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn batch_counter_threshold_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(CounterBdrReg::read(self).await?.cnt_bdr_th())
    }

    /// Number of unread sensor data (TAG + 6 bytes) stored in FIFO.
    ///
    /// # Arguments
    ///
    /// * `val`: Read the value of diff_fifo in reg FIFO_STATUS1 and FIFO_STATUS2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_data_level_get(&mut self) -> Result<u16, Error<B::Error>> {
        // read both FIFO_STATUS1 + FIFO_STATUS2 regs
        let fifo_status1 = FifoStatus1::read(self).await?;
        let fifo_status2 = FifoStatus2::read(self).await?;

        Ok(u16::from_le_bytes([
            fifo_status1.diff_fifo(),
            fifo_status2.diff_fifo(),
        ]))
    }

    /// Smart FIFO status.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `fifo_status2_t`: Read registers FIFO_STATUS2.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_status_get(&mut self) -> Result<FifoStatus2, Error<B::Error>> {
        // todo: read both fifo_status1 and fifo_status2 -> FifoStatus1 and FifoStatus2 should be
        // merged in a single struct
        FifoStatus2::read(self).await
    }

    /// Smart FIFO full status.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Read the values of fifo_full_ia in reg FIFO_STATUS2.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_full_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.fifo_status_get().await?.fifo_full_ia())
    }

    /// FIFO overrun status.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Read the values of fifo_over_run_latched in reg FIFO_STATUS2.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_ovr_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.fifo_status_get().await?.fifo_ovr_ia())
    }

    /// FIFO watermark status.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Read the values of fifo_wtm_ia in reg FIFO_STATUS2.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_wtm_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.fifo_status_get().await?.fifo_wtm_ia())
    }

    /// Identifies the sensor in FIFO_DATA_OUT.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tag_sensor in reg FIFO_DATA_OUT_TAG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn fifo_sensor_tag_get(&mut self) -> Result<FifoTag, Error<B::Error>> {
        let fifo_data_out_tag = FifoDataOutTag::read(self).await?;
        let val = fifo_data_out_tag.tag_sensor();
        Ok(FifoTag::try_from(val).unwrap_or_default())
    }

    /// DEN functionality marking mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_mode in reg CTRL6_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_mode_set(&mut self, val: DenMode) -> Result<(), Error<B::Error>> {
        let mut ctrl6_c = Ctrl6C::read(self).await?;
        ctrl6_c.set_den_mode(val as u8);
        ctrl6_c.write(self).await
    }

    /// DEN functionality marking mode.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `DenMode`: Get the values of den_mode in reg CTRL6_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_mode_get(&mut self) -> Result<DenMode, Error<B::Error>> {
        let ctrl6_c = Ctrl6C::read(self).await?;
        Ok(DenMode::try_from(ctrl6_c.den_mode()).unwrap_or_default())
    }

    /// DEN active level configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_lh in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_polarity_set(&mut self, val: DenLh) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self).await?;
        ctrl9_xl.set_den_lh(val as u8);
        ctrl9_xl.write(self).await
    }

    /// DEN active level configuration.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `DenLh`: Get the values of den_lh in reg CTRL9_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_polarity_get(&mut self) -> Result<DenLh, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self).await?;
        Ok(DenLh::try_from(ctrl9_xl.den_lh()).unwrap_or_default())
    }

    /// DEN configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_xl_g in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_enable_set(&mut self, val: DenXlG) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self).await?;
        ctrl9_xl.set_den_xl_g(val as u8);
        ctrl9_xl.write(self).await
    }

    /// DEN configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of den_xl_g in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_enable_get(&mut self) -> Result<DenXlG, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self).await?;
        Ok(DenXlG::try_from(ctrl9_xl.den_xl_g()).unwrap_or_default())
    }

    /// DEN value stored in LSB of X-axis.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_z in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_mark_axis_x_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self).await?;
        ctrl9_xl.set_den_x(val);
        ctrl9_xl.write(self).await
    }

    /// DEN value stored in LSB of X-axis.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of den_z in reg CTRL9_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_mark_axis_x_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self).await?;
        Ok(ctrl9_xl.den_x())
    }

    /// DEN value stored in LSB of Y-axis.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_y in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_mark_axis_y_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self).await?;
        ctrl9_xl.set_den_y(val);
        ctrl9_xl.write(self).await
    }

    /// DEN value stored in LSB of Y-axis.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of den_y in reg CTRL9_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_mark_axis_y_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self).await?;
        Ok(ctrl9_xl.den_y())
    }

    /// DEN value stored in LSB of Z-axis.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_x in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_mark_axis_z_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self).await?;
        ctrl9_xl.set_den_z(val);
        ctrl9_xl.write(self).await
    }

    /// DEN value stored in LSB of Z-axis.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of den_x in reg CTRL9_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub async fn den_mark_axis_z_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self).await?;
        Ok(ctrl9_xl.den_z())
    }
}

/// @brief  Convert raw data from full-scale 2g to milligrams.
///
/// # Arguments
///
/// * `lsb`: Raw data in LSB.
#[bisync]
pub fn from_fs2g_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.061
}

/// @brief  Converts a raw value from 4g full-scale to milligals.
///
/// # Arguments
///
/// * `lsb`: Raw value in 16-bit signed integer format.
///
/// # Returns
///
/// * `f32`: Converted value in milligals.
#[bisync]
pub fn from_fs4g_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.122
}

/// @brief  Convert from full-scale 8g to mg.
///
/// # Arguments
///
/// * `lsb`: Value in LSB to convert.
///
/// # Returns
///
/// * `f32`: Converted value in mg.
#[bisync]
pub fn from_fs8g_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.244
}

/// @brief  Convert from full-scale 16g to milligrams.
///
/// # Arguments
///
/// * `lsb`: Value in LSB (Least Significant Bit).
///
/// # Returns
///
/// * `Result`
///     * `f32`: Value converted to milligrams.
#[bisync]
pub fn from_fs16g_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.488
}

/// Convert from full scale 125 dps to milli degrees per second.
///
/// # Arguments
///
/// * `lsb`: The value in LSB to be converted.
///
/// # Returns
///
/// * `f32`: The converted value in milli degrees per second.
#[bisync]
pub fn from_fs125dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 4.375
}

/// @brief  Convert from full-scale 250 dps to milli-degrees per second.
///
/// # Arguments
///
/// * `lsb`: The value in LSB to convert.
///
/// # Returns
///
/// * `f32`: The converted value in milli-degrees per second.
#[bisync]
pub fn from_fs250dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 8.75
}

/// @brief  Convert from full-scale 500 dps to milli-degrees per second.
///
/// # Arguments
///
/// * `lsb`: Value in LSB to be converted.
///
/// # Returns
///
/// * `f32`: Converted value in milli-degrees per second.
#[bisync]
pub fn from_fs500dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 17.50
}

/// @brief  Convert from full-scale 1000 dps to milli-degrees per second.
///
/// # Arguments
///
/// * `lsb`: Value in LSB to be converted.
///
/// # Returns
///
/// * `f32`: Converted value in milli-degrees per second.
#[bisync]
pub fn from_fs1000dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 35.0
}

/// @brief  Convert from full-scale 2000 dps to milli-degrees per second.
///
/// # Arguments
///
/// * `lsb`: The value in LSB to convert.
///
/// # Returns
///
/// * `f32`: The converted value in milli-degrees per second.
#[bisync]
pub fn from_fs2000dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 70.0
}

/// @brief  Convert from full scale 4000 dps to milli degrees per second.
///
/// # Arguments
///
/// * `lsb`: Value in LSB.
#[bisync]
pub fn from_fs4000dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 140.0
}

/// @brief  Convert LSB to Celsius.
///
/// # Arguments
///
/// * `lsb`: The value in LSB to convert.
///
/// # Returns
///
/// * `f32`: The temperature in Celsius.
#[bisync]
pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    (lsb as f32 / 256.0) + 25.0
}

/// @brief  Convert LSB value to nanoseconds.
///
/// # Arguments
///
/// * `lsb`: The LSB value to convert.
///
/// # Returns
///
/// * `u64`: The converted value in nanoseconds.
#[bisync]
pub fn from_lsb_to_nsec(lsb: u32) -> u64 {
    (lsb as u64) * 25000
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
#[bisync]
pub enum I2CAddress {
    /// I²C address when the SA0 pin is low.
    I2cAddL = 0x6A,

    /// I²C address when the SA0 pin is high.
    I2cAddH = 0x6B,
}

///
/// ROBERT2 Device ID.
///
#[bisync]
pub const ID: u8 = 0x6B;

#[bisync]
pub const PROPERTY_ENABLE: u8 = 1;
#[bisync]
pub const PROPERTY_DISABLE: u8 = 0;
