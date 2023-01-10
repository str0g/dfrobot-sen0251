### How to detect i2c devices in system?
`ls /dev/i2c-*`
### Which device might be suitable for reading?
`sudo i2cdetect -y -r [number]`

### Does library and/or test application needs superuser rights?
yes - i2c access requires it for i/o.

### Does project requires external libraries?
* i2c-tools >= 4.3

### How to setup system for this solution
* Pick embedded system of choice and install favorite linux distribution

![missing_embedded system](https://github.com/str0g/dfrobot-sen0251/raw/master/docs/bmp388.jpg "embedded system")