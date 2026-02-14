# Preamble

See [this entry](../../DOCS/nov16_IMU.md) on using a Mahony filter to improve readings from the ICM42688. The filter automatically rejects vibration-induced accelerometer noise and uses gyro bias correction to keep the angle accurate over time. The code used for SPI communications with the ICM42688 was slightly modified from [here](https://github.com/finani/ICM42688.git) and the Mahony filter in [supervisor.cpp](src/supervisor.cpp) in `mahonyUpdateIMU()`. Also have a look at `controlLoop()` in [balance_TWR_mode.cpp](src/balance_TWR_mode.cpp) to get a general idea. This work was done at this commit: hash id: `9633b44677406c790000eb56cd1858c216e7f7b5`

Running `./IMU_test.py -p /dev/cu.usbmodem178888901` created the IMU plots of [this entry](../../DOCS/nov16_IMU.md) 

## It's time to test a few things. 

- Quantify the noise the motor injects into the IMU.
- Verify IMU reports correct angle vs. actual mechanical tilt (see: [this](../../DOCS/nov16_IMU.md))
- Determine whether torque request → torque output is linear and symmetric.
- Verify identical torque commands produce identical reaction torque.
- Check whether identical torque commands produce repeatable motor currents.
- Timing of torque command applied to motor. Use Teensy pin toggle to mark when torque command is sent. Use oscilloscope to measure phase current rise time, voltage behavior, ESC PWM switching pattern. 

## IMU noise

During the process comparing the IMU with the mechanical angle of the wheel encoders I tested this

I put this the code:
```
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    Serial.printf("a_mag=%.5f\r\n", a_mag);
```

and when the motor is running, I get these type of values: 
```
a_mag=3.05135
a_mag=5.25944
a_mag=3.85885
a_mag=6.80106
a_mag=3.69635
a_mag=5.77368
a_mag=9.36287
a_mag=10.89755
a_mag=4.79244
a_mag=2.57672
a_mag=8.32236
a_mag=1.29584
a_mag=2.59398
```

That's pretty horrendous. To reduce noise, I changed the FOC PWM from 20,000 to 10,000, and fooled around with physical dampening of the IMU. and used this to view the vibration: 

`./IMU_test.py -p /dev/cu.usbmodem178888901` 

Also, [supervisor.cpp](src/supervisor.cpp) applies a Mahony filter which fuses acceleration  and gyro values from the IMU. I added this code: 

```c
  if (accMag > 1e-6f) {
    float recip = 1.0f / accMag;
    ax *= recip;
    ay *= recip;
    az *= recip;

    if (accMag > 0.85f && accMag < 1.15f) {
      accelValid = true;
    } else {
      accelValid = false;
    }
  }
```

- Takes `a_mag` to < 2
- By no means noise free
- accel + gyro if vibrations are at a dull roar. 
- When accelValid goes false often, you'll see major problems with drift
- ChatG says there are improved gating strategies (a hysteresis-based accel valid window, or a frequency-domain vibration rejection trick) which I did not try. 
- There's a product, moongel, place under the PCB and IMU

This also helped to follow the status of vibration reduction:

`./plot_amag.py -p /dev/cu.usbmodem178888901`

