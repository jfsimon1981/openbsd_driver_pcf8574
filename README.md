# Additional drivers for OpenBSD

### NXP PCF8574(A) Remote I/O port expander

*Description:*  
GPIO driver for the I2C device PCF8574(A) from NXP


*File:*  
sys/dev/i2c/pcf8574.c

*Device Tree Source Example:*  
```
&i2c0 {
	status = "okay";

	pcf8574: gpio@20 {
		compatible = "nxp,pcf8574";
        status = "okay";
        reg = <0x20>;
        #gpio-cells = <2>;
        gpio-controller;
	};
};
```
