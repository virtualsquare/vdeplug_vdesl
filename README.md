# `vdeplug_vdesl`

This libvdeplug plugin module adds the support of vde over serial lines.

This module of libvdeplug4 can be used in any program supporting VDE like `vde_plug, vdens, kvm, qemu, user-mode-linux`
and `virtualbox`.

## install 	`vdeplug_vdesl`

Requirements: [vdeplug4](https://github.com/rd235/vdeplug4),

`vdeplug_vdesl` uses cmake, so the standard procedure to build and install
this vdeplug plugin module is the following:

```sh
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

## usage examples (tutorial)

### create a point to point ethernet on a serial line

Create a physical serial link between two linux hosts.
e.g.:

 * connect two raspberry PIs using the UART pins: pin6 to pin6 (ground), pin 8 to pin 10 and
 pin 10 to pin 8 (RTX to TXD and viceversa).
 * connect two /dev/ttyS0 ports using a null-modem cable
 * use two usb-to-ttl cables: connect RXD to TXD and viceversa (+ ground)

(If needed, disable the login prompt from the serial lines)

On both hosts run:
```
$ sudo vde_plug tap://vdesl vdesl:///dev/ttyAMA0
```

(change the name of the serial port to suit your scenario, it could be `/dev/ttyS0` or `/dev/ttyUSB0`).

The virtual ethernet interfaces named `vdesl` on the two linux hosts will be connected as if it were
a point-to-point ethernet cable between them.

The default baud rate is 38400. The `speed` option sets a different baud rate.

```
$ sudo vde_plug tap://vdesl vdesl:///dev/ttyAMA0[speed=2000000]
```

`vdesl` supports the following baud rates:
9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000.

High baud rates may not be supported by the uart hardware or could provide an unreliable service due to communication errors. In order to achieve high speeds the cables should be short and properly shielded against electro-magnetic noise. 
