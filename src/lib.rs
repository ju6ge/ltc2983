//! LTC2983
//!
//! This create provides a complete implemantition of the communication with the
//! `LTC2983` (Multi Sensor High Accuracy Digital Temperature Measurement System) via
//! SPI. Not all sensor types are supported yet.
//!
//! Contributions welcome 💪
//!
//! - [x] Theromcouple J,K,E,N,R,S,T,B
//! - [x] RTD
//! - [x] Thermistor
//! - [ ] Diode
//! - [ ] Sense Resistor
//! - [ ] Direct ADC

use embedded_hal::spi::{SpiDevice, SpiBus};

const LTC2983_WRITE: u8 = 0x2;
const LTC2983_READ: u8 = 0x3;

const STATUS_REGISTER: u16 = 0x000;

#[allow(non_camel_case_types)]
pub enum ThermalProbeType {
    Thermocouple_J,
    Thermocouple_K,
    Thermocouple_E,
    Thermocouple_N,
    Thermocouple_R,
    Thermocouple_S,
    Thermocouple_T,
    Thermocouple_B,
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
    Thermistor_Spectrum
}

impl ThermalProbeType {
    pub fn to_bytes(&self) -> u8 {
        match self {
            ThermalProbeType::Thermocouple_J => 1,
            ThermalProbeType::Thermocouple_K => 2,
            ThermalProbeType::Thermocouple_E => 3,
            ThermalProbeType::Thermocouple_N => 4,
            ThermalProbeType::Thermocouple_R => 5,
            ThermalProbeType::Thermocouple_S => 6,
            ThermalProbeType::Thermocouple_T => 7,
            ThermalProbeType::Thermocouple_B => 8,
            ThermalProbeType::RTD_PT10 => 10,
            ThermalProbeType::RTD_PT50 => 11,
            ThermalProbeType::RTD_PT100 => 12,
            ThermalProbeType::RTD_PT200 => 13,
            ThermalProbeType::RTD_PT500 => 14,
            ThermalProbeType::RTD_PT1000 => 15,
            ThermalProbeType::RTD_1000 => 16,
            ThermalProbeType::RTD_NI120 => 17,
            ThermalProbeType::Thermistor_44004_44033 => 19,
            ThermalProbeType::Thermistor_44005_44030 => 20,
            ThermalProbeType::Thermistor_44007_44034 => 21,
            ThermalProbeType::Thermistor_44006_44031 => 22,
            ThermalProbeType::Thermistor_44008_44032 => 23,
            ThermalProbeType::Thermistor_YSI400 => 24,
            ThermalProbeType::Thermistor_Spectrum => 25,
        }
    }
}

pub enum LTC2983Result {
    SensorHardFault,
    HardADCOutOfRange,
    CJHardFault(f32),
    CJSoftFault(f32),
    SensorOverRange(f32),
    SensorUnderRange(f32),
    ADCOutOfRange(f32),
    Valid(f32)
}

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
}

pub struct LTC2983Status {

}

pub enum LTC2983OcCurrent {
    External,
    I10uA,
    I100uA,
    I500uA,
    I1mA
}

pub struct LTC2983<SPI> {
    spi_device: SPI,
}

impl<SPI> LTC2983<SPI> where SPI: SpiDevice, SPI::Bus: SpiBus {
    pub fn new(spi_device: SPI) -> Self {
        LTC2983 { spi_device }
    }

    pub fn setup_channel(&self,
                         probe: ThermalProbeType,
                         channel: LTC2983Channel,
                         cold_channel: LTC2983Channel,
                         differential: bool,
                         oc_current: LTC2983OcCurrent)
    {
        todo!();
    }

    pub async fn read_temperature(&self, channel: LTC2983Channel) -> Result<LTC2983Result, std::io::Error> {
        todo!();
    }

    pub async fn read_multi_temperature(&self, channels: Vec<LTC2983Channel>) -> Result<Vec<LTC2983Result>, std::io::Error> {
        todo!();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
    }
}
