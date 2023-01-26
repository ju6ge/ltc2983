//! LTC2983
//!
//! This create provides a complete implemantition of the communication with the
//! `LTC2983` (Multi Sensor High Accuracy Digital Temperature Measurement System) via
//! SPI. Not all sensor types are supported yet.
//!
//! Contributions welcome 💪
//!
//! - [x] Theromcouple J,K,E,N,R,S,T,B
//! - [ ] Custom Thermocouple
//! - [ ] RTD
//! - [ ] Thermistor
//! - [x] Sense Resistor
//! - [x] Diode
//! - [ ] Direct ADC

use std::convert::TryInto;

use bytebuffer::ByteBuffer;
use embedded_hal::spi::{SpiDevice, SpiBus};
use fixed::{FixedU32, types::extra::{U25, U10, U20}, FixedI32};
use thiserror::Error;

const LTC2983_WRITE: u8 = 0x2;
const LTC2983_READ: u8 = 0x3;

const STATUS_REGISTER: u16 = 0x000;
const GLOBAL_CONFIG_REGISTER: u16 = 0x0F0;
const MULTI_CHANNEL_MASK_REGISTER: u16 = 0x0F4;

#[derive(Debug)]
pub enum SensorConfiguration {
    SingleEnded,
    Differential
}

impl Default for SensorConfiguration {
    fn default() -> Self {
        Self::SingleEnded
    }
}

impl SensorConfiguration {
    pub fn identifier(&self) -> u64 {
        match self {
            SensorConfiguration::SingleEnded => 1,
            SensorConfiguration::Differential => 0,
        }
    }
}

#[derive(Debug)]
pub struct ThermocoupleParameters {
    cold_junction_channel: Option<LTC2983Channel>,
    sensor_configuration: SensorConfiguration,
    oc_current: LTC2983OcCurrent,
    custom_address: Option<u16>
}

impl Default for ThermocoupleParameters {
    fn default() -> Self {
        Self { cold_junction_channel: None,
               sensor_configuration: Default::default(),
               oc_current: Default::default(),
               custom_address: None }
    }
}

impl ThermocoupleParameters {
    pub fn cold_junction(mut self, chan: LTC2983Channel) -> Self {
        self.cold_junction_channel = Some(chan);
        self
    }

    pub fn sensor_configuration(mut self, config: SensorConfiguration) -> Self {
        self.sensor_configuration = config;
        self
    }

    pub fn custom_address(mut self, addr: u16) -> Self {
        self.custom_address = Some(addr);
        self
    }

    pub fn config_to_bits(&self) -> u64 {
        0x0 | (self.sensor_configuration.identifier() << 3) | self.oc_current.identifier()
    }
}

#[derive(Debug)]
pub enum DiodeReadingCount {
    READ2,
    READ3
}

impl Default for DiodeReadingCount {
    fn default() -> Self {
        Self::READ2
    }
}

impl DiodeReadingCount {
    pub fn identifier(&self) -> u64 {
        match self {
            DiodeReadingCount::READ2 => 0,
            DiodeReadingCount::READ3 => 1,
        }
    }
}

#[derive(Debug)]
pub enum DiodeExcitationCurrent {
    I10uA,
    I20uA,
    I40uA,
    I80uA
}

impl Default for DiodeExcitationCurrent {
    fn default() -> Self {
        Self::I10uA
    }
}

impl DiodeExcitationCurrent {
    pub fn identifier(&self) -> u64 {
        match self {
            DiodeExcitationCurrent::I10uA => 0,
            DiodeExcitationCurrent::I20uA => 1,
            DiodeExcitationCurrent::I40uA => 2,
            DiodeExcitationCurrent::I80uA => 3,
        }
    }
}

#[derive(Debug)]
pub struct DiodeParameters {
    sensor_configuration: SensorConfiguration,
    num_reading: DiodeReadingCount,
    avg: bool,
    excitation_current: DiodeExcitationCurrent,
    idealitiy_factor: Option<f32>
}

impl Default for DiodeParameters {
    fn default() -> Self {
        Self {
            sensor_configuration: Default::default(),
            num_reading: Default::default(),
            excitation_current: Default::default(),
            idealitiy_factor: None,
            avg: true,
        }
    }
}

impl DiodeParameters {
    pub fn sensor_configuration(mut self, config: SensorConfiguration) -> Self {
        self.sensor_configuration = config;
        self
    }

    pub fn num_reading(mut self, cnt: DiodeReadingCount) -> Self {
        self.num_reading = cnt;
        self
    }

    pub fn excitation_current(mut self, current: DiodeExcitationCurrent) -> Self {
        self.excitation_current = current;
        self
    }

    pub fn use_avg(mut self, flag: bool) -> Self {
        self.avg = flag;
        self
    }

    pub fn ideality_factor(mut self, factor: f32) -> Self {
        self.idealitiy_factor = Some(factor);
        self
    }

    pub fn to_bits(&self) -> u64 {
        0x0 | (self.sensor_configuration.identifier() << 26)
            | (self.num_reading.identifier() << 25)
            | ((self.avg as u64) << 24)
            | (self.excitation_current.identifier() << 22)
            | ( match self.idealitiy_factor {
                None => 0x0,
                Some(factor) => {
                    let factor_fixed_point = FixedU32::<U20>::from_num(factor);
                    (factor_fixed_point.to_bits() & 0x3fffff) as u64 //mask the upper bits to only include the lower 22 bits
                }
            })
    }
}


#[allow(non_camel_case_types)]
#[derive(Debug)]
pub enum ThermalProbeType {
    Thermocouple_J(ThermocoupleParameters),
    Thermocouple_K(ThermocoupleParameters),
    Thermocouple_E(ThermocoupleParameters),
    Thermocouple_N(ThermocoupleParameters),
    Thermocouple_R(ThermocoupleParameters),
    Thermocouple_S(ThermocoupleParameters),
    Thermocouple_T(ThermocoupleParameters),
    Thermocouple_B(ThermocoupleParameters),
    RTD_PT10,
    RTD_PT50,
    RTD_PT100,
    RTD_PT200,
    RTD_PT500,
    RTD_PT1000,
    RTD_1000,
    RTD_NI120,
    Thermistor_44004_44033,
    Thermistor_44005_44030,
    Thermistor_44007_44034,
    Thermistor_44006_44031,
    Thermistor_44008_44032,
    Thermistor_YSI400,
    Thermistor_Spectrum,
    Diode(DiodeParameters),
    SenseResistor(f32)
}

impl ThermalProbeType {
    pub fn identifier(&self) -> u64 {
        match self {
            ThermalProbeType::Thermocouple_J(_)      => 1,
            ThermalProbeType::Thermocouple_K(_)      => 2,
            ThermalProbeType::Thermocouple_E(_)      => 3,
            ThermalProbeType::Thermocouple_N(_)      => 4,
            ThermalProbeType::Thermocouple_R(_)      => 5,
            ThermalProbeType::Thermocouple_S(_)      => 6,
            ThermalProbeType::Thermocouple_T(_)      => 7,
            ThermalProbeType::Thermocouple_B(_)      => 8,
            ThermalProbeType::RTD_PT10               => 10,
            ThermalProbeType::RTD_PT50               => 11,
            ThermalProbeType::RTD_PT100              => 12,
            ThermalProbeType::RTD_PT200              => 13,
            ThermalProbeType::RTD_PT500              => 14,
            ThermalProbeType::RTD_PT1000             => 15,
            ThermalProbeType::RTD_1000               => 16,
            ThermalProbeType::RTD_NI120              => 17,
            ThermalProbeType::Thermistor_44004_44033 => 19,
            ThermalProbeType::Thermistor_44005_44030 => 20,
            ThermalProbeType::Thermistor_44007_44034 => 21,
            ThermalProbeType::Thermistor_44006_44031 => 22,
            ThermalProbeType::Thermistor_44008_44032 => 23,
            ThermalProbeType::Thermistor_YSI400      => 24,
            ThermalProbeType::Thermistor_Spectrum    => 25,
            ThermalProbeType::Diode(_)               => 28,
            ThermalProbeType::SenseResistor(_)       => 29
        }
    }
}

#[derive(Debug)]
pub enum LTC2983Result {
    SensorHardFault,
    HardADCOutOfRange,
    CJHardFault,
    CJSoftFault(f32),
    SensorOverRange(f32),
    SensorUnderRange(f32),
    ADCOutOfRange(f32),
    Valid(f32)
}

impl From<[u8; 4]> for LTC2983Result {
    fn from(bytes: [u8; 4]) -> Self {
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(bytes[1..=3].try_into().unwrap()));
        match bytes[0] {
            0x01 => {
                LTC2983Result::Valid(value.to_num())
            }
            0x02 => {
                LTC2983Result::ADCOutOfRange(value.to_num())
            }
            0x04 => {
                LTC2983Result::SensorUnderRange(value.to_num())
            }
            0x08 => {
                LTC2983Result::SensorOverRange(value.to_num())
            }
            0x10 => {
                LTC2983Result::CJSoftFault(value.to_num())
            }
            0x20 => {
                LTC2983Result::CJHardFault
            }
            0x40 => {
                LTC2983Result::HardADCOutOfRange
            }
            0x80 => {
                LTC2983Result::SensorHardFault
            }
            _ => {
                //error codes are bit masks, only one bit should be active at a time
                println!("Error code unknown: 0x{:x}",bytes[0]);
                unreachable!()
            }
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum LTC2983Channel {
    CH1,
    CH2,
    CH3,
    CH4,
    CH5,
    CH6,
    CH7,
    CH8,
    CH9,
    CH10,
    CH11,
    CH12,
    CH13,
    CH14,
    CH15,
    CH16,
    CH17,
    CH18,
    CH19,
    CH20
}

impl LTC2983Channel {
    pub fn start_address(&self) -> u16 {
        match self {
            LTC2983Channel::CH1  => 0x200,
            LTC2983Channel::CH2  => 0x204,
            LTC2983Channel::CH3  => 0x208,
            LTC2983Channel::CH4  => 0x20C,
            LTC2983Channel::CH5  => 0x210,
            LTC2983Channel::CH6  => 0x214,
            LTC2983Channel::CH7  => 0x218,
            LTC2983Channel::CH8  => 0x21C,
            LTC2983Channel::CH9  => 0x220,
            LTC2983Channel::CH10 => 0x224,
            LTC2983Channel::CH11 => 0x228,
            LTC2983Channel::CH12 => 0x22C,
            LTC2983Channel::CH13 => 0x230,
            LTC2983Channel::CH14 => 0x234,
            LTC2983Channel::CH15 => 0x238,
            LTC2983Channel::CH16 => 0x23C,
            LTC2983Channel::CH17 => 0x240,
            LTC2983Channel::CH18 => 0x244,
            LTC2983Channel::CH19 => 0x248,
            LTC2983Channel::CH20 => 0x24C
        }
    }

    pub fn result_address(&self) -> u16 {
        match self {
            LTC2983Channel::CH1  => 0x010,
            LTC2983Channel::CH2  => 0x014,
            LTC2983Channel::CH3  => 0x018,
            LTC2983Channel::CH4  => 0x01C,
            LTC2983Channel::CH5  => 0x020,
            LTC2983Channel::CH6  => 0x024,
            LTC2983Channel::CH7  => 0x028,
            LTC2983Channel::CH8  => 0x02C,
            LTC2983Channel::CH9  => 0x030,
            LTC2983Channel::CH10 => 0x034,
            LTC2983Channel::CH11 => 0x038,
            LTC2983Channel::CH12 => 0x03C,
            LTC2983Channel::CH13 => 0x040,
            LTC2983Channel::CH14 => 0x044,
            LTC2983Channel::CH15 => 0x048,
            LTC2983Channel::CH16 => 0x04C,
            LTC2983Channel::CH17 => 0x050,
            LTC2983Channel::CH18 => 0x054,
            LTC2983Channel::CH19 => 0x058,
            LTC2983Channel::CH20 => 0x05C,
        }
    }

    pub fn identifier(&self) -> u64 {
        match self {
            LTC2983Channel::CH1  => 1,
            LTC2983Channel::CH2  => 2,
            LTC2983Channel::CH3  => 3,
            LTC2983Channel::CH4  => 4,
            LTC2983Channel::CH5  => 5,
            LTC2983Channel::CH6  => 6,
            LTC2983Channel::CH7  => 7,
            LTC2983Channel::CH8  => 8,
            LTC2983Channel::CH9  => 9,
            LTC2983Channel::CH10 => 10,
            LTC2983Channel::CH11 => 11,
            LTC2983Channel::CH12 => 12,
            LTC2983Channel::CH13 => 13,
            LTC2983Channel::CH14 => 14,
            LTC2983Channel::CH15 => 15,
            LTC2983Channel::CH16 => 16,
            LTC2983Channel::CH17 => 17,
            LTC2983Channel::CH18 => 18,
            LTC2983Channel::CH19 => 19,
            LTC2983Channel::CH20 => 20,
        }
    }

    pub fn mask(&self) -> u32 {
       0x1 << (self.identifier() - 1)
    }
}

#[derive(Debug)]
pub struct LTC2983Status {
    start: bool,
    done: bool,
    //1 bit unused
    channel_selection: u8
}

impl LTC2983Status {
    pub fn done(&self) -> bool {
        self.done
    }
}

impl From<u8> for LTC2983Status {
    fn from(data: u8) -> Self {
        LTC2983Status {
            start: data & 0x80 == 0x80,
            done: data & 0x40 == 0x40,
            channel_selection: data & 0x1f
        }
    }
}

#[derive(Debug)]
pub enum LTC2983OcCurrent {
    External,
    I10uA,
    I100uA,
    I500uA,
    I1mA
}

impl Default for LTC2983OcCurrent {
    fn default() -> Self {
        Self::I10uA
    }
}

impl LTC2983OcCurrent {
    pub fn identifier(&self) -> u64 {
        match self {
            LTC2983OcCurrent::External => 0,
            LTC2983OcCurrent::I10uA => 4,
            LTC2983OcCurrent::I100uA => 5,
            LTC2983OcCurrent::I500uA => 6,
            LTC2983OcCurrent::I1mA => 7,
        }
    }
}

#[derive(Debug, Error)]
pub enum LTC2983Error<SPI> {
    #[error("SPI communication error: {0:?}")]
    SpiError(#[from] SPI),
    #[error("Channel {0:?} not configured!")]
    ChannelUnconfigured(LTC2983Channel),
}

pub struct LTC2983<SPI> {
    spi_device: SPI,
}

impl<SPI> LTC2983<SPI> where SPI: SpiDevice, SPI::Bus: SpiBus {
    pub fn new(spi_device: SPI) -> Self {
        LTC2983 { spi_device }
    }

    //read device satatus
    pub fn status(&mut self) -> Result<LTC2983Status, LTC2983Error<SPI::Error>> {
        let mut read_status_bytes = ByteBuffer::new();
        read_status_bytes.write_u8(LTC2983_READ);
        read_status_bytes.write_u16(STATUS_REGISTER);
        read_status_bytes.write_u8(0x0); //Dummy Data

        let mut recv: [u8; 4] = [0, 0, 0, 0];
        match self.spi_device.transfer(&mut recv, read_status_bytes.as_bytes()) {
            Ok(_) => {
                Ok(LTC2983Status::from(recv[3]))
            }
            Err(err) => Err(LTC2983Error::SpiError(err))
        }

    }

    //write channel configuration
    pub fn setup_channel(&mut self,
                         probe: ThermalProbeType,
                         channel: LTC2983Channel) -> Result<(), LTC2983Error<SPI::Error>>
    {
        match &probe {
            ThermalProbeType::Thermocouple_J(param) |
            ThermalProbeType::Thermocouple_K(param) |
            ThermalProbeType::Thermocouple_E(param) |
            ThermalProbeType::Thermocouple_N(param) |
            ThermalProbeType::Thermocouple_R(param) |
            ThermalProbeType::Thermocouple_S(param) |
            ThermalProbeType::Thermocouple_T(param) |
            ThermalProbeType::Thermocouple_B(param) => {
                let mut write_sequence = ByteBuffer::new();
                write_sequence.write_u8(LTC2983_WRITE);              //the first byte of the communication indicates a read or write operation
                write_sequence.write_u16(channel.start_address());   //the second two bytes hold the address to ẁrite to
                // The 32 bit data to be written to the channel configuration register has the following format for thermocouples
                // |31-27| Thermocouple Type
                write_sequence.write_bits(probe.identifier(), 5);
                // |26-22| Could Junction Channel ID -> if no cold junction compensation is used this value will be 0
                write_sequence.write_bits(match &param.cold_junction_channel { None => 0, Some(chan) => chan.identifier() }, 5);
                // |21-18| Sensor Configuration
                write_sequence.write_bits(param.config_to_bits(), 4);
                // |17-12| Unused => equals 0
                write_sequence.write_bits(0, 6);
                // |11-0| Custom Thermocouple Data Pointer
                write_sequence.write_bits(match &param.custom_address { None => 0, Some(addr) => *addr}.into(), 12);

                self.spi_device.write(write_sequence.as_bytes())?;
                Ok(())
            }
            ThermalProbeType::RTD_PT10   |
            ThermalProbeType::RTD_PT50   |
            ThermalProbeType::RTD_PT100  |
            ThermalProbeType::RTD_PT200  |
            ThermalProbeType::RTD_PT500  |
            ThermalProbeType::RTD_PT1000 |
            ThermalProbeType::RTD_1000   |
            ThermalProbeType::RTD_NI120  => {
                unimplemented!();
            }
            ThermalProbeType::Thermistor_44004_44033 |
            ThermalProbeType::Thermistor_44005_44030 |
            ThermalProbeType::Thermistor_44007_44034 |
            ThermalProbeType::Thermistor_44006_44031 |
            ThermalProbeType::Thermistor_44008_44032 |
            ThermalProbeType::Thermistor_YSI400      |
            ThermalProbeType::Thermistor_Spectrum    => {
                unimplemented!();
            }
            ThermalProbeType::Diode(param) => {
                let mut write_sequence = ByteBuffer::new();
                write_sequence.write_u8(LTC2983_WRITE);              //the first byte of the communication indicates a read or write operation
                write_sequence.write_u16(channel.start_address());   //the second two bytes hold the address to ẁrite to
                write_sequence.write_bits(probe.identifier(), 5);
                write_sequence.write_bits(param.to_bits(), 27);

                self.spi_device.write(write_sequence.as_bytes())?;
                Ok(())
            }
            ThermalProbeType::SenseResistor(resistance) => {
                let mut write_sequence = ByteBuffer::new();
                write_sequence.write_u8(LTC2983_WRITE);              //the first byte of the communication indicates a read or write operation
                write_sequence.write_u16(channel.start_address());   //the second two bytes hold the address to ẁrite to
                // The 32 bit data to be written to the channel configuration register has the following format for sense resistors
                // |31-27| Thermocouple Type
                write_sequence.write_bits(probe.identifier(), 5);
                // |26-0| Fixed Point Floating point (17,10) no sign bit representing the resistance
                let resistance_fixed_point = FixedU32::<U10>::from_num(*resistance);
                write_sequence.write_bits(resistance_fixed_point.to_bits().into(), 27);

                self.spi_device.write(write_sequence.as_bytes())?;
                Ok(())
            }
        }
    }

    //check if the channel is configured
    pub fn channel_enabled(&mut self, channel: LTC2983Channel) -> bool {
        let mut read_sequence = ByteBuffer::new();
        read_sequence.write_u8(LTC2983_READ);
        read_sequence.write_u16(channel.start_address());
        read_sequence.write_u8(0); //Dummy Data for read

        let mut recv: [u8; 4] = [0, 0, 0, 0];
        match self.spi_device.transfer(&mut recv, read_sequence.as_bytes()) {
            Ok(_) => {
                //if the upper 5bits of the channel are zero, then the channel is disabled so checking for not zero means the channel is enabled
                if recv[3] & 0xf8 != 0 {
                    true
                } else {
                    false
                }
            }
            Err(_err) => {
                //on communication error assume unconfigured channel
                false
            }
        }
    }

    pub fn start_conversion(&mut self, channel: LTC2983Channel) -> Result<(), LTC2983Error<SPI::Error>> {
        //start measurement
        let mut start_command_bytes = ByteBuffer::new();
        start_command_bytes.write_u8(LTC2983_WRITE);
        start_command_bytes.write_u16(STATUS_REGISTER);
        start_command_bytes.write_bits(0x4, 3);
        start_command_bytes.write_bits(channel.identifier(), 5);

        self.spi_device.write(start_command_bytes.as_bytes())?;

        Ok(())
    }

    pub fn start_multi_conversion(&mut self, channels: Vec<LTC2983Channel>) -> Result<(), LTC2983Error<SPI::Error>> {
        let mut write_channel_mask = ByteBuffer::new();
        let mut mask: u32 = 0x0;
        for chan in channels {
            mask |= chan.mask();
        }
        write_channel_mask.write_u8(LTC2983_WRITE);
        write_channel_mask.write_u16(MULTI_CHANNEL_MASK_REGISTER);
        write_channel_mask.write_u32(mask);
        self.spi_device.write(write_channel_mask.as_bytes())?;

        let mut start_multi_conversion_bytes = ByteBuffer::new();
        start_multi_conversion_bytes.write_u8(LTC2983_WRITE);
        start_multi_conversion_bytes.write_u16(STATUS_REGISTER);
        start_multi_conversion_bytes.write_bits(0x4, 3);
        start_multi_conversion_bytes.write_bits(0x0, 5);

        self.spi_device.write(start_multi_conversion_bytes.as_bytes())?;
        Ok(())
    }

    pub fn read_temperature(&mut self, channel: LTC2983Channel) -> Result<LTC2983Result, LTC2983Error<SPI::Error>> {
        let mut read_temperature_bytes = ByteBuffer::new();
        read_temperature_bytes.write_u8(LTC2983_READ);
        read_temperature_bytes.write_u16(channel.result_address());
        read_temperature_bytes.write_u32(0x0); //Dummy bytes for reading

        let mut recv: [u8; 7] = [0, 0, 0, 0, 0, 0, 0];
        self.spi_device.transfer(&mut recv, read_temperature_bytes.as_bytes())?;

        Ok(LTC2983Result::from([recv[3], recv[4], recv[5], recv[6]]))
    }

    pub fn read_multi_temperature(&mut self, channels: Vec<LTC2983Channel>) -> Vec<Result<LTC2983Result, LTC2983Error<SPI::Error>>> {
        channels.iter().map(|chan| {
            self.read_temperature(*chan)
        }).collect()
    }
}

fn reformat_fixedf24_to_fixed_f32(bytes_f24: &[u8; 3]) -> [u8; 4]{
    if bytes_f24[0] & 0x80 == 0x80 {
        [0xff, bytes_f24[0], bytes_f24[1], bytes_f24[2]]
    } else {
        [0x00, bytes_f24[0], bytes_f24[1], bytes_f24[2]]
    }
}

#[cfg(test)]
mod tests {
    use fixed::{FixedI32, types::extra::U10};

    use super::*;

    #[test]
    fn test_fixedf24_u10_to_f32_signed() {
        let bytes: [u8; 3] = [ 0x7f, 0xff, 0xff ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (8191.999 as f32) < 1./1024.); // error should be smaller than smallest fixed point value 1./1024.

        let bytes: [u8; 3] = [ 0x10, 0x00, 0x00 ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (1024 as f32) < 1./1024.); // error should be smaller than smallest fixed point value 1./1024.

        let bytes: [u8; 3] = [ 0x00, 0x04, 0x00 ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (1 as f32) < 1./1024.); // error should be smaller than smallest fixed point value 1./1024.

        let bytes: [u8; 3] = [ 0x00, 0x00, 0x01 ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (1./1024. as f32) < 1./1024.); // error should be smaller than smallest fixed point value 1./1024.

        let bytes: [u8; 3] = [ 0x00, 0x00, 0x00 ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (0. as f32) < 1./1024.); // error should be smaller than smallest fixed point value 1./1024.

        let bytes: [u8; 3] = [ 0xff, 0xff, 0xff ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (-1./1024. as f32) < 1./1024.); // error should be smaller than smallest fixed point value 1./1024.

        let bytes: [u8; 3] = [ 0xff, 0xfc, 0x00 ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (-1 as f32) < 1./1024.); // error should be smaller than smallest fixed point value 1./1024.

        let bytes: [u8; 3] = [ 0xfb, 0xbb, 0x67 ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (-273.15 as f32) < 1./1024.); // error should be smaller than smallest fixed point value 1./1024.

        let bytes: [u8; 3] = [ 0xf8, 0xd1, 0x52 ];
        let value = FixedI32::<U10>::from_be_bytes(reformat_fixedf24_to_fixed_f32(&bytes));
        assert!(value.to_num::<f32>() - (-459.67 as f32) < 1./1027.); // error should be smaller than smallest fixed point value 1./1024.
    }
}
