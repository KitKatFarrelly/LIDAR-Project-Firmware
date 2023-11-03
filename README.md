# LIDAR Project Firmware
 Firmware for the LIDAR board. The target for this code is an ESP32.

# Warning to those who stumbled upon this
I am not maintaining this code. Unfortunately, while I was able to communicate with the
epc 611 with this code, this is not a proper driver for the chip and I should warn that
I was unable to get useful data readings out of the chip. If you are an industry person I
would highly recommend you contact the ESPROS company themselves for support on making a
driver, and if you are a hobbyist I strongly recommend against using this chip in your
projects - while it is powerful, it requires a lot of work to get anything useful out of
it. If you want an affordable and powerful ToF sensor, I would recommend ST's VL53x line,
or OSRAM's TMF882X line of sensors. Both of which perform out of the box and have substantial
documentation and free drivers for hobbyist support.
