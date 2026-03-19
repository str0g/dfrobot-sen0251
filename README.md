# How to build
Preferable building method for testing, development, release

## List build profiles
```make list_presets```

## Release
```make release```

## Debug
```make debug```

## Package for your distribution
Currently only ArchLinux is supported

```make pkg```

# How to test
```make test```

## unit tests
```cmake --build build/debug/ && ctest --test-dir build/debug/ --output-on-failure```

# How to Develope with it
  Namespaces:
  - Commands - I2C command enumeration (nop, extmode_en_middle, fifo_flush, softreset)
  - PowerControl - Power management flags (off, pressure_on, temperature_on, force_on_a/b, normal)
  - Oversampling - Sample rate options (no, x2, x4, x8, x16, x32)
  - IirFilter - Digital filter coefficients

  Core Methods:
  - get_readings(): Returns temperature, pressure, and altitude
  - power_control(): Manages sensor power states
  - set_oversampling(): Configures measurement resolution
  - soft_reset(): Resets sensor configuration

  Main Class Sen0251:
  - Public API for reading/writing sensor data
  - Calibration methods (temperature and pressure)
  - Power control and configuration
  - Test support via TestSen0251 class

  Key Features:
  - Non-copyable class (deleted copy/move constructors)
  - I2C communication via file descriptor
  - FIFO buffer management
  - Calibration data handling (11 pressure coefficients, 3 temperature coefficients)
  - Altitude calculation using sea level pressure

# Troubleshooting
## How to detect i2c devices in system?
`ls /dev/i2c-*`

## Which device might be suitable for reading?
`sudo i2cdetect -y -r [number]`

## Does library and/or test application needs superuser rights?
yes - i2c access requires it for i/o.

## Does project requires external libraries?
* i2c-tools >= 4.3

## How to setup system for this solution
* Pick embedded system of choice and install favorite linux distribution

![missing_embedded system](https://github.com/str0g/dfrobot-sen0251/raw/master/docs/bmp388.jpg "embedded system")
