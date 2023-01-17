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
//! - [ ] Diode
//! - [ ] Direct ADC

use bytebuffer::ByteBuffer;
use embedded_hal::spi::{SpiDevice, SpiBus};
use fixed::{FixedU32, types::extra::U25};

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

impl ThermocoupleParameters {
    pub fn sensor_configuration(&self) -> u64 {
        let mut buf = ByteBuffer::new();
        buf.write_bits(0, 4);
        buf.write_bits(self.sensor_configuration.identifier(), 1);
        buf.write_bits(self.oc_current.identifier(), 3);
        buf.read_u8().unwrap().into()
    }
}

pub struct SenseResistorParameters {

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
    SenseResistor(f32)
}

impl ThermalProbeType {
    pub fn identifier(&self) -> u64 {
        match self {
            ThermalProbeType::Thermocouple_J(_) => 1,
            ThermalProbeType::Thermocouple_K(_) => 2,
            ThermalProbeType::Thermocouple_E(_) => 3,
            ThermalProbeType::Thermocouple_N(_) => 4,
            ThermalProbeType::Thermocouple_R(_) => 5,
            ThermalProbeType::Thermocouple_S(_) => 6,
            ThermalProbeType::Thermocouple_T(_) => 7,
            ThermalProbeType::Thermocouple_B(_) => 8,
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
            ThermalProbeType::SenseResistor(_) => 29
        }
    }
}

#[derive(Debug)]
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

#[derive(Debug)]
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
            LTC2983Channel::CH1 => 1,
            LTC2983Channel::CH2 => 2,
            LTC2983Channel::CH3 => 3,
            LTC2983Channel::CH4 => 4,
            LTC2983Channel::CH5 => 5,
            LTC2983Channel::CH6 => 6,
            LTC2983Channel::CH7 => 7,
            LTC2983Channel::CH8 => 8,
            LTC2983Channel::CH9 => 9,
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
}

#[derive(Debug)]
pub struct LTC2983Status {
    start: bool,
    done: bool,
    //1 bit unused
    channel_selection: u8
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

pub struct LTC2983<SPI> {
    spi_device: SPI,
}

impl<SPI> LTC2983<SPI> where SPI: SpiDevice, SPI::Bus: SpiBus {
    pub fn new(spi_device: SPI) -> Self {
        LTC2983 { spi_device }
    }

    pub fn status(&mut self) -> Result<LTC2983Status, SPI::Error> {
        let mut transfer_bytes = ByteBuffer::new();
        transfer_bytes.write_u8(LTC2983_READ);
        transfer_bytes.write_u16(STATUS_REGISTER);
        transfer_bytes.write_u8(0x0); //Dummy Data

        let mut recv: [u8; 4] = [0, 0, 0, 0];
        match self.spi_device.transfer(&mut recv, transfer_bytes.as_bytes()) {
            Ok(_) => {
                Ok(LTC2983Status::from(recv[3]))
            }
            Err(err) => Err(err)
        }

    }

    pub fn setup_channel(&mut self,
                         probe: ThermalProbeType,
                         channel: LTC2983Channel) -> Result<(), SPI::Error>
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
                write_sequence.write_bits(param.sensor_configuration(), 4);
                // |17-12| Unused => equals 0
                write_sequence.write_bits(0, 6);
                // |11-0| Custom Thermocouple Data Pointer
                write_sequence.write_bits(match &param.custom_address { None => 0, Some(addr) => *addr}.into(), 12);

                self.spi_device.write(write_sequence.as_bytes())
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
            ThermalProbeType::SenseResistor(resistance) => {
                let mut write_sequence = ByteBuffer::new();
                write_sequence.write_u8(LTC2983_WRITE);              //the first byte of the communication indicates a read or write operation
                write_sequence.write_u16(channel.start_address());   //the second two bytes hold the address to ẁrite to
                // The 32 bit data to be written to the channel configuration register has the following format for sense resistors
                // |31-27| Thermocouple Type
                write_sequence.write_bits(probe.identifier(), 5);
                // |26-0| Fixed Point Floating point (17,10) no sign bit representing the resistance
                let resistance_fixed_point = FixedU32::<U25>::from_num(*resistance);
                write_sequence.write_bits(resistance_fixed_point.to_bits().into(), 27);

                self.spi_device.write(write_sequence.as_bytes())
            }
        }
    }

    pub async fn read_temperature(&self, channel: LTC2983Channel) -> Result<LTC2983Result, std::io::Error> {
        todo!();
    }

    pub async fn read_multi_temperature(&self, channels: Vec<LTC2983Channel>) -> Result<Vec<LTC2983Result>, std::io::Error> {
        todo!();
    }
}

//#[cfg(test)]
//mod tests {
//    use super::*;
//
//    #[test]
//    fn it_works() {
//    }
//}
