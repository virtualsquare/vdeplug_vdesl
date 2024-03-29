<!--
.\" Copyright (C) 2020 VirtualSquare. Project Leader: Renzo Davoli
.\"
.\" This is free documentation; you can redistribute it and/or
.\" modify it under the terms of the GNU General Public License,
.\" as published by the Free Software Foundation, either version 2
.\" of the License, or (at your option) any later version.
.\"
.\" The GNU General Public License's references to "object code"
.\" and "executables" are to be interpreted as the output of any
.\" document formatting or typesetting system, including
.\" intermediate and printed output.
.\"
.\" This manual is distributed in the hope that it will be useful,
.\" but WITHOUT ANY WARRANTY; without even the implied warranty of
.\" MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
.\" GNU General Public License for more details.
.\"
.\" You should have received a copy of the GNU General Public
.\" License along with this manual; if not, write to the Free
.\" Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
.\" MA 02110-1301 USA.
.\"
-->
# NAME

`libvdeplug_vdesl` -- vdesl vdeplug module: ethernet over serial link

# SYNOPSIS
libvdeplug_vdesl.so

# DESCRIPTION

This libvdeplug module creates point-to-point virtual ethernet networks over
serial links.

This  module of libvdeplug4 can be used in any program supporting vde like
`vde_plug`, `vdens`, `kvm`, `qemu`, `user-mode-linux` and `virtualbox`.

The vde_plug_url syntax of this module is the following:

  `vdesl://`*device_path* [ `[` OPTION [ /OPTION]...`]` ]

# OPTIONS

  `speed`=_baudrate_
: Set the baud rate of the serial link. This plugin supports the following baud rates:
: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000,
: 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000.
: High baud rates may not be supported by the uart hardware or could provide an unreliable service
: due to communication errors. In order to achieve high speeds the cables should be short and
: properly shielded from electro-magnetic noise.


# EXAMPLES

Scenario: two hosts connected by a serial line. e.g.: RS-232 serial ports connected by a null modem cable,
USB-ttl cables or USB-FTDI boards or UART pins of System-on-Chip point-to-point connected so that the
RXD on one end is connected to TXD on the other and viceversa.

In the following we use the device `/dev/ttyAMA0`, this may be changed to address the actual device (e.g. `/dev/ttyS0`,
`/dev/ttyS1`, `/dev/ttyUSB0`, `/dev/ttyACM0`...).
No other processes should be using the serial port (e.g. no getty-login process should be active on it).

The following command (on both hosts) creates two vde namespaces connected by a virtual point-to-point ethernet
network.

```
vdens vdesl:///dev/ttyAMA0
```

Run the following command on both hosts to create a virtual interface named `vdesl0` on each end. The two
interfaces are connected by a 4Mbaud link.

```
sudo vde_plug -d tap://vdesl0 vdesl:///dev/ttyAMA0[speed=4000000]
```

# NOTICE

Virtual  Distributed  Ethernet  is not related in any way with www.vde.com ("Verband der Elektrotechnik, Elektronik
und Informationstechnik" i.e. the German "Association for Electrical, Electronic & Information Technologies").

# SEE ALSO
`vde_plug`(1), `vdens`(1)

# AUTHOR
VirtualSquare. Project leader: Renzo Davoli
