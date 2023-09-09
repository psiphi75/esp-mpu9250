# MPU9250 C driver for ESP8266 and ESP32

This contains a driver for the [MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/),
a Nine-Axis (Gyro + Accelerometer + Compass) Motion Processing Unit. It also contains the
[Madgwick AHRS](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/) algorithm.

The MPU9250 uses the i2c bus.

The project has been configured for the ESP8266, but should also work on the ESP32 with a little
configuration.

This code is based on the following: https://github.com/miniben-90/mpu9250

## Pin assignment

- master for ESP8266:
  - GPIO14 is assigned as the data signal of i2c master port
  - GPIO2 is assigned as the clock signal of i2c master port

## Configure the project

```
idf.py menuconfig
```

A special "MPU9250 Configuration" menu is avialble. There is a calibration option which needs
to be set the first time such that you can calibrate your MPU9250 device. See the #calibration
section.

## Calibration

Each MPU9250 device will have different calibration parameters at different temperatures. You
will need to calibrate your device for it to work accurately. Each component has different
calibration requirements:

- Gyroscope: The gyroscope only needs the bias calculated. This measures the average noise
  from the gyro, then uses it as a bias amount.
- Accelerometer: This also calculates the bias. But also the average 'g' force needs to be
  calculated.
- Magnetometer: This calibration (based on [this](http://www.camelsoftware.com/2016/03/13/imu-maths-calculate-orientation-pt3/))
  will only work for your location on Earth, give or take a hundred kilometers. The reason
  is that it uses the Earth's magnetic field.

See the `calibration.c` file for details.

### Using the results

You should consider increasing the monitor window's print buffer so that the large amount of printing for magnetometer does not override the other calibration output.

Open the project configuration menu (`idf.py menuconfig`). Then go into `Example Configuration` menu.

- You can choose whether or not to save command history into flash in `Store command history in flash` option.
- You can set the maximum number of command line arguments under `Maximum number of command line arguments` option.
- You can set the command line buffer length under `Command line buffer length` option.

After running the calibration, **copy and paste the values printed by the calibration tool to main.c's cal variable.**

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py flash monitor
```

(To exit the serial monitor, type `Ctrl-]`.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

Below is an example of the output when _not_ in calibration mode.

```
I (493280) main: heading: 138.749°, pitch: -0.508°, roll: -91.158°, Temp 24.163°C
I (493341) main: heading: 138.746°, pitch: -0.532°, roll: -91.190°, Temp 24.019°C
```
