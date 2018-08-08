/*
 * Copyright (c) 2018 Johannes Krottmayer <krjdev@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* Driver for the NXP PCF8574(A) Remote 8-bit I/O expander */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/gpio.h>

#include <dev/i2c/i2cvar.h>
#include <dev/gpio/gpiovar.h>

#define PCFGPIO_NPINS		8

struct pcfgpio_softc {
	struct device 			sc_dev;
	i2c_tag_t				sc_tag;
	i2c_addr_t				sc_addr;
	int						sc_node;

	u_int8_t				sc_npins;

    struct gpio_chipset_tag	sc_gpio_gc;
    gpio_pin_t				sc_gpio_pins[PCFGPIO_NPINS];
};

int pcfgpio_match(struct device *, void *, void *);
void pcfgpio_attach(struct device *, struct device *, void *);
uint8_t pcfgpio_read(struct pcfgpio_softc *);
void pcfgpio_write(struct pcfgpio_softc *, uint8_t);
int pcfgpio_pin_read(void *, int);
void pcfgpio_pin_write(void *, int, int);
void pcfgpio_pin_ctl(void *, int, int);

struct cfattach pcfgpio_ca = {
	sizeof(struct pcfgpio_softc), pcfgpio_match, pcfgpio_attach
};

struct cfdriver pcfgpio_cd = {
	NULL, "pcfgpio", DV_DULL
};

int
pcfgpio_match(struct device *parent, void *match, void *aux)
{
	struct i2c_attach_args *ia = aux;

	if ((strcmp(ia->ia_name, "nxp,pcf8574") == 0) ||
        (strcmp(ia->ia_name, "nxp,pcf8574a") == 0))
		return (1);
	return (0);
}

void
pcfgpio_attach(struct device *parent, struct device *self, void *aux)
{
	struct pcfgpio_softc *sc = (struct pcfgpio_softc *)self;
	struct i2c_attach_args *ia = aux;
	struct gpiobus_attach_args gba;
	int i;

	sc->sc_tag = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;
	sc->sc_npins = PCFGPIO_NPINS;
	sc->sc_node = *(int *)ia->ia_cookie;

	printf("\n");

	for (i = 0; i < sc->sc_npins; i++) {
		sc->sc_gpio_pins[i].pin_num = i;
		sc->sc_gpio_pins[i].pin_caps = GPIO_PIN_INOUT | 
			GPIO_PIN_OPENDRAIN | GPIO_PIN_INVOUT;

		sc->sc_gpio_pins[i].pin_flags = GPIO_PIN_INPUT;
		sc->sc_gpio_pins[i].pin_state = 0;
	}

	pcfgpio_write(sc, 0xFF);

	sc->sc_gpio_gc.gp_cookie = sc;
	sc->sc_gpio_gc.gp_pin_read = pcfgpio_pin_read;
	sc->sc_gpio_gc.gp_pin_write = pcfgpio_pin_write;
	sc->sc_gpio_gc.gp_pin_ctl = pcfgpio_pin_ctl;

	gba.gba_name = "gpio";
	gba.gba_gc = &sc->sc_gpio_gc;
	gba.gba_pins = sc->sc_gpio_pins;
	gba.gba_npins = sc->sc_npins;
	config_found(&sc->sc_dev, &gba, gpiobus_print);
}

uint8_t
pcfgpio_read(struct pcfgpio_softc *sc)
{
	uint8_t val;

	iic_acquire_bus(sc->sc_tag, 0);
	if (iic_exec(sc->sc_tag, I2C_OP_READ_WITH_STOP, sc->sc_addr,
		NULL, 0, &val, sizeof(val), 0)) {
		printf("%s: pcfgpio_read: failed to read\n",
			sc->sc_dev.dv_xname);
		iic_release_bus(sc->sc_tag, 0);
		return (0);
	}
	iic_release_bus(sc->sc_tag, 0);
	return val;
}

void
pcfgpio_write(struct pcfgpio_softc *sc, uint8_t val)
{
	iic_acquire_bus(sc->sc_tag, 0);
	if (iic_exec(sc->sc_tag, I2C_OP_WRITE_WITH_STOP, sc->sc_addr,
		&val, sizeof val, NULL, 0, 0)) {
		printf("%s: pcfgpio_write: failed to write\n",
			sc->sc_dev.dv_xname);
		iic_release_bus(sc->sc_tag, 0);
		return;
	}
	iic_release_bus(sc->sc_tag, 0);
}

int
pcfgpio_pin_read(void *arg, int pin)
{
    struct pcfgpio_softc *sc = arg;
	uint8_t tmp;

	if (pin >= sc->sc_npins)
		return 0;
	if (!ISSET(sc->sc_gpio_pins[pin].pin_flags, GPIO_PIN_INPUT))
		pcfgpio_pin_write(sc, pin, GPIO_PIN_HIGH);
	tmp = pcfgpio_read(sc);
	if (tmp & (1 << pin))
		sc->sc_gpio_pins[pin].pin_state = GPIO_PIN_HIGH;
	else
		sc->sc_gpio_pins[pin].pin_state = GPIO_PIN_LOW;
	return sc->sc_gpio_pins[pin].pin_state;
}

void
pcfgpio_pin_write(void *arg, int pin, int val)
{
	struct pcfgpio_softc *sc = arg;
	int i;
	uint8_t tmp = 0x00;

	if (pin >= sc->sc_npins)
		return;
	for (i = 0; i < sc->sc_npins; i++)
		if (sc->sc_gpio_pins[i].pin_state == GPIO_PIN_LOW)
			tmp |= (1 << i);
	if (val == GPIO_PIN_HIGH) {
		tmp &= ~(1 << pin);
		pcfgpio_write(sc, tmp);
		sc->sc_gpio_pins[pin].pin_state = GPIO_PIN_HIGH;
	} else {
		tmp |= 1 << pin;
		pcfgpio_write(sc, tmp);
		sc->sc_gpio_pins[pin].pin_state = GPIO_PIN_LOW;
	}
}

void
pcfgpio_pin_ctl(void *arg, int pin, int flags)
{
	struct pcfgpio_softc *sc = arg;

	if (ISSET(flags, GPIO_PIN_INPUT)) {
		pcfgpio_pin_write(sc, pin, GPIO_PIN_HIGH);
		sc->sc_gpio_pins[pin].pin_flags = GPIO_PIN_INPUT;
    } else
		sc->sc_gpio_pins[pin].pin_flags = GPIO_PIN_OUTPUT;
}
