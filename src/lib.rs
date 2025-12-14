#![no_std]

use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
use core::result::Result;
use core::result::Result::Ok;
use core::result::Result::Err;

pub enum WindowDetectionMode {
    /// <summary>
    /// Object under a certain distance.
    /// </summary>
    Below = 0,

    /// <summary>
    /// Object beyond a certain distance.
    /// </summary>
    Above = 1,

    /// <summary>
    /// Object out of a window limited by a near and far threshold.
    /// </summary>
    Out = 2,

    /// <summary>
    /// Object within a window limited by a near and far threshold.
    /// </summary>
    In = 3,
}

pub enum Precision {
    /// <summary>
    /// Short distance. Maximum distance is limited to 1.3m but results in a better ambient immunity.
    /// </summary>
    Short = 1,

    /// <summary>
    /// Long distance. Can range up to 4 m in the dark with a timing budget of 200 ms.
    /// </summary>
    Long = 2,

    /// <summary>
    /// Unknown distance mode or the distance mode is not configured.
    /// </summary>
    Unknown = 3,
}

pub enum TimingBudget {
    /// <summary>
    /// Unknown budget or the budget is not configured
    /// </summary>
    BudgetUnknown = 0,

    /// <summary>
    /// Budget of 15 ms
    /// </summary>
    Budget15 = 15,

    /// <summary>
    /// Budget of 20 ms
    /// </summary>
    Budget20 = 20,

    /// <summary>
    /// Budget of 33 ms
    /// </summary>
    Budget33 = 33,

    /// <summary>
    /// Budget of 50 ms
    /// </summary>
    Budget50 = 50,

    /// <summary>
    /// Budget of 100 ms
    /// </summary>
    Budget100 = 100,

    /// <summary>
    /// Budget of 200 ms
    /// </summary>
    Budget200 = 200,

    /// <summary>
    /// Budget of 500 ms
    /// </summary>
    Budget500 = 500,
}

pub enum RangeStatus {
    /// <summary>
    /// Range Status value is unknown to current device specifications.
    /// </summary>
    Unknown = -1,

    /// <summary>
    /// No error has occured.
    /// </summary>
    NoError = 0,

    /// <summary>
    /// SigmaFailure.
    /// This means that the repeatability or standard deviation of the measurement is bad due to a decreasing signal noise ratio.
    /// Increasing the timing budget can improve the standard deviation and avoid a range status 1.
    /// </summary>
    SigmaFailure = 1,

    /// <summary>
    /// SignalFailure.
    /// This means that the return signal is too weak to return a good answer.
    /// The reason is that the target is too far, or the target is not reflective enough, or the target is too small.
    /// Increasing the timing budget might help, but there may simply be no target available.
    /// </summary>
    SignalFailure = 2,

    /// <summary>
    /// OutOfBounds.
    /// This means that the sensor is ranging in a "non-appropriated" zone and the measured result may be inconsistent.
    /// This status is considered as a warning but, in general, it happens when a target is at the maximum distance possible from the sensor, i.e. around 5 m.
    /// However, this is only for very bright targets.
    /// </summary>
    OutOfBounds = 4,

    /// <summary>
    /// WrapAround.
    /// This situation may occur when the target is very reflective and the distance to the target/sensor is longer than the physical limited distance measurable by the sensor.
    /// Such distances include approximately 5 m when the senor is in Long distance mode and approximately 1.3 m when the sensor is in Short distance mode.
    /// </summary>
    WrapAround = 7,
}

enum Register {
    Vl53l1xI2cSlaveDeviceAddress = 0x0001,
    Vl53l1xVhvConfigTimeoutMacropLoopBound = 0x0008,
    AlgoCrosstalkCompensationPlaneOffsetKcps = 0x0016,
    AlgoCrosstalkCompensationXPlaneGradientKcps = 0x0018,
    AlgoCrosstalkCompensationYPlaneGradientKcps = 0x001A,
    AlgoPartToPartRangeOffsetMm = 0x001E,
    MmConfigInnerOffsetMm = 0x0020,
    MmConfigOuterOffsetMm = 0x0022,
    GpioHvMuxCtrl = 0x0030,
    GpioTioHvStatus = 0x0031,
    SystemInterruptConfigGpio = 0x0046,
    PhasecalConfigTimeoutMacrop = 0x004B,
    RangeConfigTimeoutMacropAHi = 0x005E,
    RangeConfigVcselPeriodA = 0x0060,
    RangeConfigVcselPeriodB = 0x0063,
    RangeConfigTimeoutMacropBHi = 0x0061,
    RangeConfigTimeoutMacropBLo = 0x0062,
    RangeConfigSigmaThresh = 0x0064,
    RangeConfigMinCountRateRtnLimitMcps = 0x0066,
    RangeConfigValidPhaseHigh = 0x0069,
    Vl53l1xSystemIntermeasurementPeriod = 0x006C,
    SystemThreshHigh = 0x0072,
    SystemThreshLow = 0x0074,
    SdConfigWoiSd0 = 0x0078,
    SdConfigInitialPhaseSd0 = 0x007A,
    RoiConfigUserRoiCentreSpad = 0x007F,
    RoiConfigUserRoiRequestedGlobalXySize = 0x0080,
    SystemSequenceConfig = 0x0081,
    Vl53l1xSystemGroupedParameterHold = 0x0082,
    SystemInterruptClear = 0x0086,
    SystemModeStart = 0x0087,
    Vl53l1xResultRangeStatus = 0x0089,
    Vl53l1xResultDssActualEffectiveSpadsSd0 = 0x008C,
    ResultAmbientCountRateMcpsSd = 0x0090,
    Vl53l1xResultFinalCrosstalkCorrectedRangeMmSd0 = 0x0096,
    Vl53l1xResultPeakSignalCountRateCrosstalkCorrectedMcpsSd0 = 0x0098,
    Vl53l1xResultOscCalibrateVal = 0x00DE,
    Vl53l1xFirmwareSystemStatus = 0x00E5,
    Vl53l1xIdentificationModelId = 0x010F,
    Vl53l1xRoiConfigModeRoiCentreSpad = 0x013E,
    Vl53l1xDefaultDeviceAddress = 0x29,
}

pub struct Vl53l1x<I2c: embedded_hal::i2c::I2c, Delay: embedded_hal::delay::DelayNs> {
    device: I2c,
    address: u8,
    timeout_ms: u32,
    delay: Delay,
    ranging_initialized: bool,
}

impl<I2c: embedded_hal::i2c::I2c, Delay: embedded_hal::delay::DelayNs> Vl53l1x<I2c, Delay> {
    pub fn new(device: I2c, address: u8, timeout_ms: u32, delay: Delay) -> Result<Self, ErrorKind> {
        let mut d = Vl53l1x {
            device,
            address,
            timeout_ms,
            delay,
            ranging_initialized: false,
        };

        match d.wait_for_booted() {
            Ok(_) => {
                match d.init_sensor() {
                    Ok(_) => Ok(d),
                    Err(e) => Err(e)
                }
            }
            Err(e) => Err(e)
        }
    }

    pub fn get_distance_mm(&mut self) -> Result<u16, ErrorKind> {
        if !self.ranging_initialized {
            self.start_ranging().unwrap();
        }

        match self.wait_for_data_ready() {
            Ok(_) => {
                let distance = self.read_word(Register::Vl53l1xResultFinalCrosstalkCorrectedRangeMmSd0);
                self.clear_interrupt().unwrap();
                Ok(distance.unwrap())
            }
            Err(e) => Err(e)
        }
    }

    pub fn set_distance_threshold(&mut self, low_mm: u16, high_mm: u16, window_detection_mode: WindowDetectionMode) -> Result<(), <I2c>::Error> {
        let mut temp: u8 = self.read_byte(Register::SystemInterruptConfigGpio)?;
        temp &= 0x47;
        self.write_register(Register::SystemInterruptConfigGpio, temp | (((window_detection_mode as u8) & 0x07) | 0x40))?;
        self.write_word(Register::SystemThreshHigh, high_mm)?;
        self.write_word(Register::SystemThreshLow, low_mm)
    }

    pub fn get_range_status(&mut self) -> Result<RangeStatus, <I2c>::Error> {
        match self.read_byte(Register::Vl53l1xResultRangeStatus) {
            Ok(b) => match b & 0x1F {
                9 => Ok(RangeStatus::NoError),
                6 => Ok(RangeStatus::SigmaFailure),
                4 => Ok(RangeStatus::SignalFailure),
                5 => Ok(RangeStatus::OutOfBounds),
                7 => Ok(RangeStatus::WrapAround),
                _ => Ok(RangeStatus::Unknown)
            },
            Err(e) => Err(e)
        }
    }

    pub fn get_precision(&mut self) -> Result<Precision, <I2c>::Error> {
        match self.read_byte(Register::PhasecalConfigTimeoutMacrop) {
            Ok(temp) => {
                if temp != 0x14 && temp != 0xA {
                    return Ok(Precision::Unknown);
                }
                if temp == 0x14 { Ok(Precision::Short) } else { Ok(Precision::Long) }
            }
            Err(e) => Err(e)
        }
    }

    pub fn set_precision(&mut self, precision: Precision) -> Result<(), <I2c>::Error> {
        let timing_budget = self.get_timing_budget_ms().unwrap_or(TimingBudget::BudgetUnknown);

        match precision {
            Precision::Short => {
                self.write_register(Register::PhasecalConfigTimeoutMacrop, 0x14)?;
                self.write_register(Register::RangeConfigVcselPeriodA, 0x07)?;
                self.write_register(Register::RangeConfigVcselPeriodB, 0x05)?;
                self.write_register(Register::RangeConfigValidPhaseHigh, 0x38)?;
                self.write_word(Register::SdConfigWoiSd0, 0x0705)?;
                self.write_word(Register::SdConfigInitialPhaseSd0, 0x0606)?
            }
            Precision::Long => {
                self.write_register(Register::PhasecalConfigTimeoutMacrop, 0x0A)?;
                self.write_register(Register::RangeConfigVcselPeriodA, 0x0F)?;
                self.write_register(Register::RangeConfigVcselPeriodB, 0x0D)?;
                self.write_register(Register::RangeConfigValidPhaseHigh, 0xB8)?;
                self.write_word(Register::SdConfigWoiSd0, 0x0F0D)?;
                self.write_word(Register::SdConfigInitialPhaseSd0, 0x0E0E)?
            }
            _ => {}
        }
        self.set_timing_budget_ms(timing_budget)
    }

    pub fn get_timing_budget_ms(&mut self) -> Result<TimingBudget, <I2c>::Error> {
        return Ok(match self.read_word(Register::RangeConfigTimeoutMacropAHi) {
            Ok(budget) => {
                match budget {
                    0x001D => TimingBudget::Budget15,
                    0x0051 | 0x001E => TimingBudget::Budget20,
                    0x00D6 | 0x0060 => TimingBudget::Budget33,
                    0x1AE | 0x00AD => TimingBudget::Budget50,
                    0x02E1 | 0x01CC => TimingBudget::Budget100,
                    0x03E1 | 0x02D9 => TimingBudget::Budget200,
                    0x0591 | 0x048F => TimingBudget::Budget500,
                    _ => TimingBudget::BudgetUnknown
                }
            }
            Err(e) => return Err(e)
        });
    }

    pub fn set_timing_budget_ms(&mut self, timing_budget: TimingBudget) -> Result<(), <I2c>::Error> {
        return Ok(match self.get_precision() {
            Ok(p) => {
                match p {
                    Precision::Short => {
                        match timing_budget {
                            TimingBudget::Budget15 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x01D)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x0027)?;
                            }
                            TimingBudget::Budget20 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x0051)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x006E)?;
                            }
                            TimingBudget::Budget33 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x00D6)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x006E)?;
                            }
                            TimingBudget::Budget50 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x1AE)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x01E8)?;
                            }
                            TimingBudget::Budget100 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x02E1)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x0388)?;
                            }
                            TimingBudget::Budget200 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x03E1)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x0496)?;
                            }
                            TimingBudget::Budget500 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x0591)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x05C1)?;
                            }
                            _ => ()
                        }
                    }
                    Precision::Long => {
                        match timing_budget {
                            TimingBudget::Budget20 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x001E)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x0022)?;
                            }
                            TimingBudget::Budget33 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x0060)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x006E)?;
                            }
                            TimingBudget::Budget50 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x00AD)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x00C6)?;
                            }
                            TimingBudget::Budget100 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x01CC)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x01EA)?;
                            }
                            TimingBudget::Budget200 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x02D9)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x02F8)?;
                            }
                            TimingBudget::Budget500 => {
                                self.write_word(Register::RangeConfigTimeoutMacropAHi, 0x048F)?;
                                self.write_word(Register::RangeConfigTimeoutMacropBHi, 0x04A4)?;
                            }
                            _ => ()
                        }
                    }
                    _ => {}
                }
            }
            Err(e) => return Err(e)
        });
    }

    pub fn get_sensor_id(&mut self) -> Result<u16, <I2c>::Error> {
        self.read_word(Register::Vl53l1xIdentificationModelId)
    }

    fn init_sensor(&mut self) -> Result<(), ErrorKind> {
        for i in 0x2Du8..0x87u8 {
            self.write_raw_register(i as u16, DEFAULT_CONF[(i - 0x2D) as usize]).unwrap();
        }

        self.start_ranging().unwrap();
        match self.wait_for_data_ready() {
            Ok(_) => {
                self.clear_interrupt().unwrap();
                self.stop_ranging().unwrap();
                self.write_register(Register::Vl53l1xVhvConfigTimeoutMacropLoopBound, 0x9).unwrap();
                self.write_raw_register(0xB, 0).unwrap();
                Ok(())
            }
            Err(e) => Err(e)
        }
    }

    fn wait_for_data_ready(&mut self) -> Result<(), ErrorKind> {
        let mut data_ready = self.data_ready().unwrap();
        let mut elapsed = 0u32;

        while !data_ready && elapsed <= self.timeout_ms {
            self.delay.delay_ms(50);
            elapsed += 50;
            data_ready = self.data_ready().unwrap();
        }

        if !data_ready {
            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
        }

        Ok(())
    }

    fn data_ready(&mut self) -> Result<bool, <I2c>::Error> {
        match self.read_byte(Register::GpioTioHvStatus) {
            Ok(b) => Ok(b != 0),
            Err(e) => Err(e)
        }
    }

    fn wait_for_booted(&mut self) -> Result<(), ErrorKind> {
        let mut ready = self.has_booted().unwrap();
        let mut elapsed = 0u32;

        while !ready && elapsed <= self.timeout_ms {
            self.delay.delay_ms(10);
            elapsed += 10;
            ready = self.has_booted().unwrap();
        }

        if !ready {
            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown));
        }

        Ok(())
    }

    fn has_booted(&mut self) -> Result<bool, <I2c>::Error> {
        match self.read_byte(Register::Vl53l1xFirmwareSystemStatus) {
            Ok(b) => Ok(b > 0),
            Err(e) => Err(e)
        }
    }

    fn start_ranging(&mut self) -> Result<(), <I2c>::Error> {
        match self.clear_interrupt() {
            Ok(_) => {
                match self.write_register(Register::SystemModeStart, 0x40) {
                    Ok(_) => {
                        self.ranging_initialized = true;
                        Ok(())
                    }
                    Err(e) => Err(e)
                }
            }
            Err(e) => Err(e)
        }
    }

    fn stop_ranging(&mut self) -> Result<(), <I2c>::Error> {
        match self.write_register(Register::SystemModeStart, 0x00) {
            Ok(_) => {
                self.ranging_initialized = false;
                Ok(())
            }
            Err(e) => Err(e)
        }
    }

    fn clear_interrupt(&mut self) -> Result<(), <I2c>::Error> {
        self.write_register(Register::SystemInterruptClear, 0x01)
    }

    fn read_word(&mut self, register: Register) -> Result<u16, I2c::Error> {
        let mut register_buffer: [u8; 2] = [0, 0];
        let mut word_buffer: [u8; 2] = [0, 0];
        let register_as_ushort = register as u16;

        register_buffer[0] = (register_as_ushort >> 8) as u8;
        register_buffer[1] = (register_as_ushort & 0xFF) as u8;

        match self.device.write_read(self.address, &register_buffer, &mut word_buffer) {
            Ok(_) => {
                // MSB at index 0, LSB at index 1
                let word = ((word_buffer[0] as u16) << 8) | (word_buffer[1] as u16);
                Ok(word)
            }
            Err(e) => Err(e)
        }
    }

    fn read_byte(&mut self, register: Register) -> Result<u8, <I2c>::Error> {
        let mut register_buffer: [u8; 2] = [0, 0];
        let mut byte: [u8; 1] = [0];
        let register_as_ushort = register as u16;

        register_buffer[0] = (register_as_ushort >> 8) as u8;
        register_buffer[1] = (register_as_ushort & 0xFF) as u8;

        match self.device.write_read(self.address, &register_buffer, &mut byte) {
            Ok(_) => Ok(byte[0]),
            Err(e) => Err(e)
        }
    }

    fn write_register(&mut self, register: Register, param: u8) -> Result<(), <I2c>::Error> {
        let mut write_buffer: [u8; 3] = [0, 0, 0];
        let register_as_ushort = register as u16;

        write_buffer[0] = (register_as_ushort >> 8) as u8;
        write_buffer[1] = (register_as_ushort & 0xFF) as u8;
        write_buffer[2] = param;

        self.device.write(self.address, &mut write_buffer)
    }

    fn write_raw_register(&mut self, register: u16, param: u8) -> Result<(), <I2c>::Error> {
        let mut write_buffer: [u8; 3] = [0, 0, 0];

        write_buffer[0] = (register >> 8) as u8;
        write_buffer[1] = (register & 0xFF) as u8;
        write_buffer[2] = param;

        self.device.write(self.address, &mut write_buffer)
    }

    fn write_word(&mut self, register: Register, word: u16) -> Result<(), <I2c>::Error> {
        let mut write_buffer: [u8; 4] = [0u8; 4];
        let register_as_ushort = register as u16;

        write_buffer[0] = (register_as_ushort >> 8) as u8;
        write_buffer[1] = (register_as_ushort & 0xFF) as u8;
        write_buffer[2] = (word >> 8) as u8;
        write_buffer[3] = (word & 0xFF) as u8;

        self.device.write(self.address, &mut write_buffer)
    }
}

// gently stolen from Microsoft
const DEFAULT_CONF: [u8; 91] = [
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
    0x01, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
    0x00, /* 0x32 : not user-modifiable */ 0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */ 0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */ 0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */ 0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */ 0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */ 0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */ 0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */ 0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */ 0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */ 0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
    0x0b, /* 0x47 : not user-modifiable */ 0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */ 0x02, /* 0x4a : not user-modifiable */
    0x0a, /* 0x4b : not user-modifiable */ 0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */ 0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */ 0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */ 0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */ 0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */ 0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */ 0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */ 0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */ 0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */ 0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */ 0x0f, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */ 0xf1, /* 0x62 : not user-modifiable */
    0x0d, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
    0x80, /* 0x67 : Min count Rate LSB */ 0x08, /* 0x68 : not user-modifiable */
    0xb8, /* 0x69 : not user-modifiable */ 0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
    0x00, /* 0x6d : Intermeasurement period */ 0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */ 0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x75 : distance threshold low LSB */ 0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */ 0x0f, /* 0x78 : not user-modifiable */
    0x0d, /* 0x79 : not user-modifiable */ 0x0e, /* 0x7a : not user-modifiable */
    0x0e, /* 0x7b : not user-modifiable */ 0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */ 0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : ROI center, use SetROI() */ 0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
    0x9B, /* 0x81 : not user-modifiable */ 0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */ 0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */ 0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
    0x00 /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
];