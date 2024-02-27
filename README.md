<div align="center">

# Raspberry Pi Pico GPS Driver

GPS driver for raspberry pi pico (pico c/c++ sdk projects).
this code libraray developed as part of my main project "FCP" (autonomous flight controller project),
it can be used for other projects thats need gps sensor position data,
based on the Adafruit Ultimate V3 GPS board.

Hardware:

[Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) •
[Adafruit Ultimate V3 GPS](https://learn.adafruit.com/adafruit-ultimate-gps/overview)

</div>

<div>

# Example Of Use

```c++
GPSdriver gps_sensor;
GPSdata gps_container;
gps_sensor.read_data(gps_container);
std::cout << "[" << gps_container.hr << ":" << gps_container.min << "] - Position => " << gps_container.N << " N, " << gps_container.E << " E." << std::endl;
```

</div>
