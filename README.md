# HLW8012-Driver

## Description
This is a driver for the hlw8012 powermeter module. It uses the iio subsystem of the linux kernel.
This can be used on any microcontroller.

## Usage
Include the source file in your kernel environment and build the module for your system. In your device-tree-file you
have to define three pins:

gpio_sel - this is for selection of the output either being current oder voltage
gpio_cf1 - this is the data pin for current or voltage depending on the sel-pin
gpio_cf - this is a pin for the active power data

Insert these names in you DTS-File, compile it and flash it to your MC. Obviously you have to connect the module
to the proper pins as defined in your DTS-File. Load the module (or autoload it) on your system. 

The driver opens three channels for you to get data from. Voltage, current and power. Using this is straight forward.

## DTS-File
An example of a DTS-File could look like this (here for AT91SAM9G25):


```
...
hlw8012@0 {
        compatible = "hlw8012";
        pinctrl-names = "default";
        status = "okay";
        sel-gpio = <&pioC 29 0>;
        cf-gpio = <&pioC 31 0>;
        cf1-gpio = <&pioC 30 0>;
};
...
```

If you don't use DTS-Files, you will need to allocate the proper gpios in the kernel on your own.

## Author
Felix Froehling <felix.froehling1@gmail.com>


