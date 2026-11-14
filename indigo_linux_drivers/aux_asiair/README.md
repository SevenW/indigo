# ZWO ASIAIR Power Ports AUX Controller driver

https://www.raspberrypi.org

## Supported devices

ZWO Power Ports ASIAIR for RPi Models.

## Supported platforms

This driver is supported on Linux (ARM v6+).

## License

INDIGO Astronomy open-source license.

## Use

indigo_server indigo_aux_asiair

## Status: More testing needed

Driver is developed and tested with:
* ZWO ASIAIR PRO RPi
* ZWO ASIAIR Plus RPi

Later models of ASIAIR Plus are no longer based on a Raspberry Pi 4B.

## GPIO Pin mapping:

This driver supports the 4 power ports on the ASIAIR PRO and Plus RPi models

### Outputs
* Output 1 -> GPIO 12 (PWM0*)
* Output 2 -> GPIO 13 (PWM1*)
* Output 3 -> GPIO 26
* Output 4 -> GPIO 18

*Two-channel PWM is supported. To enable it add "dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4" to the bottom of /boot/config.txt:
```
$ sudo echo "dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4" >>/boot/config.txt
```

NOTE: As of version 3.0.0.4 PWM is opt-in, enable the X_AUX_PWM property. The existence of a PWM chip does not mean that a header pin is routed to it, so the driver no longer assumes it and Outputs #1 and #2 work as plain GPIO until PWM is switched on.
On a Raspberry Pi 5 the pwm-2chan overlay does not route any header pin to the RP1 PWM controller, so PWM is not usable on that model.

## Testing

- 2026-09-27 02:07 3.0.0.7 linux x64 fake SDK 20/20 OK
- 2026-10-25 09:11 3.0.0.8 linux arm64 fake SDK 24/24 OK
- 2026-10-25 09:11 3.0.0.8 linux arm64 Raspberry Pi 5 3/3 OK
- 2026-10-25 09:13 3.0.0.8 linux arm64 fake SDK (Raspberry Pi 3B+) 24/24 OK
- 2026-10-25 09:14 3.0.0.8 linux arm64 Raspberry Pi 3B+ 3/3 OK
