# asm330lhh-rs
[![Crates.io][crates-badge]][crates-url]
[![BSD 3-Clause licensed][bsd-badge]][bsd-url]

[crates-badge]: https://img.shields.io/crates/v/asm330lhh-rs
[crates-url]: https://crates.io/crates/asm330lhh-rs
[bsd-badge]: https://img.shields.io/crates/l/asm330lhh-rs
[bsd-url]: https://opensource.org/licenses/BSD-3-Clause

Provides a platform-agnostic, no_std-compatible driver for the ST ASM330LHH sensor, supporting both I2C and SPI communication interfaces.

## Sensor Overview


The ASM330LHH is a system-in-package featuring a 3D digital accelerometer and a 3D digital gyroscope with an extended temperature range up to +105 °C and designed to address automotive non-safety applications.

ST's family of MEMS sensor modules leverages the robust and mature manufacturing processes already used for the production of micromachined accelerometers and gyroscopes to serve both the automotive and consumer market. The ASM330LHH is AEC-Q100 compliant and industrialized through a dedicated MEMS production flow to meet automotive reliability standards. All the parts are fully tested with respect to temperature to ensure the highest quality level.

The sensing elements are manufactured using ST's proprietary micromachining processes, while the IC interfaces are developed using CMOS technology that allows the design of a dedicated circuit which is trimmed to better match the characteristics of the sensing element.

The ASM330LHH has a full-scale acceleration range of ±2/±4/±8/±16 g and a wide angular rate range of ±125/±250/±500/±1000/±2000/±4000 dps that enables its usage in a broad range of automotive applications.

All the design aspects of the ASM330LHH have been optimized to reach superior output stability, extremely low noise and full data synchronization to the benefit of sensor-assisted applications like dead reckoning and sensor fusion.

The ASM330LHH is available in a 14-lead plastic land grid array (LGA) package.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/asm330lhh.html](https://www.st.com/en/mems-and-sensors/asm330lhh.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
asm330lhh-rs = "1.0.0"
```

Or, add it directly from the terminal:

```sh
cargo add asm330lhh-rs
```

## Usage

By default, the create exposes the **asynchronous** API, and it could be included using:
```rust
use asm330lhh::asynchronous as asm330lhh;
use asm330lhh::*;
use asm330lhh::prelude::*;
```

### Blocking API (optional feature)

To use the **blocking** API instead of the asynchronous one, disable default features and enable the `blocking` feature in your Cargo.toml
```toml
[dependencies]
asm330lhh-rs = { version = "1.0.0", default-features = false, features = ["blocking"] }
```
or from the terminal:
```sh
cargo add asm330lhh-rs --no-default-features --features blocking
```

Then import the blocking API:
```rust
use asm330lhh::blocking as asm330lhh;
use asm330lhh::*;
use asm330lhh::prelude::*;
```

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance, along with a timing peripheral.

An example with I2C:

```rust
let mut sensor = Asm330lhh::new_i2c(i2c, I2CAddress::I2cAddH, delay);
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.device_id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
// Restore default configuration
sensor.reset_set(PROPERTY_ENABLE).unwrap();

loop {
    let rst = sensor.reset_get().unwrap();
    if rst == 0 {
        break;
    }
}

// Start device configuration
sensor.device_conf_set(PROPERTY_ENABLE).unwrap();
// Enable Block Data Update
sensor.block_data_update_set(PROPERTY_ENABLE).unwrap();

// Set Output Data Rate
sensor.xl_data_rate_set(OdrXl::_12_5hz).unwrap();
sensor.gy_data_rate_set(OdrGy::_12_5hz).unwrap();
// Set Full Scale
sensor.xl_full_scale_set(FsXl::_2g).unwrap();
sensor.gy_full_scale_set(FsGy::_2000dps).unwrap();
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2026 STMicroelectronics**
