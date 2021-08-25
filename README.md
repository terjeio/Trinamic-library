## Trinamic driver library

21-08-10: Refactored to support ganged motors, some API changes.

Supports TMC2130, TMC2209 and TMC5160 drivers.

Written in plain C, processor agnostic. Processor specific low-level communications layer has to be added by user.

This library is used by some [grblHAL](https://github.com/grblHAL) drivers and examples of low-level communications layers and a [higher level](https://github.com/grblHAL/Plugins_motor) configuration/reporting layer implementation can be found there.

A [SPI <> I2C](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) bridge implemented on a TI MSP430G2553 processor that may be used for systems with limited IO capabilities.

---
2021-08-10
