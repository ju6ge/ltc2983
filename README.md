LTC2983
=======

A create to provide an implementation of the communication with the
`LTC2983` (Multi Sensor High Accuracy Digital Temperature Measurement System) via
SPI. Not all sensor types are supported yet.

Contributions welcome 💪

- [x] Theromcouple J,K,E,N,R,S,T,B
- [ ] Custom Theromcouple
- [ ] RTD
- [ ] Thermistor
- [x] Sense Resistor
- [x] Diode
- [ ] Direct ADC

# Example of readout

``` rust
    let mut ltc = LTC2983::new(device);

    let _ = ltc.setup_channel(ltc2983::ThermalProbeType::Diode(ltc2983::DiodeParameters::default().ideality_factor(1.).excitation_current(ltc2983::DiodeExcitationCurrent::I20uA).num_reading(ltc2983::DiodeReadingCount::READ3)), ltc2983::LTC2983Channel::CH2);
    let _ = ltc.setup_channel(ltc2983::ThermalProbeType::Thermocouple_T(ThermocoupleParameters::default().cold_junction(ltc2983::LTC2983Channel::CH2)), ltc2983::LTC2983Channel::CH1);

    loop {
        let _ = ltc.start_conversion(ltc2983::LTC2983Channel::CH1);
        let mut status = ltc.status().unwrap();
        while !status.done() {
            status = ltc.status().unwrap();
        }
        let result = ltc.read_temperature(ltc2983::LTC2983Channel::CH1);
        println!("{result:#?}");
        sleep(Duration::new(1, 0));
    }

```

