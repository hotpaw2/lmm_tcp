# lmm_tcp
## An rtl-tcp compatible, IQ server for LimeSDR Mini

This repository contains source code for lmm_tcp, a TCP server for
[LimeSDR Mini](https://wiki.myriadrf.org/LimeSDR-Mini)
SDR radio USB devices.
The lmm_tcp server emulates the rtl_tcp protocol as implemented by the
[rtl_tcp](https://github.com/osmocom/rtl-sdr)
command-line tool developed by OsmoCom and others
for various Realtek RTL2832-based DVB-T USB peripherals.
There are multiple SDR applications,
available for Linux, macOS, or Wintel systems,
that can connect to a local or remote SDR radio peripheral
via the rtl_tcp protocol.
This server allows using many of those applications with a
LimeSDR Mini
(among those applications are the iOS apps
_rtl_tcp SDR_
and
_SDR Receiver_,
for SDR on an iPhone or iPad).

The requirements for using lmm_tcp include first installing
[LimeSuite](https://github.com/myriadrf/LimeSuite).
Instructions for building and installing LimeSuite are here:
[https://wiki.myriadrf.org/Lime_Suite](https://wiki.myriadrf.org/Lime_Suite) .

After installing LimeSuite on a Raspberry Pi 3 Model B,
this is a command-line that I used for compiling lmm_tcp :
```
cc -lm -lLimeSuite -O2 -o lmm_tcp lmm_tcp.c
```

--  
Distribution permitted under the Mozilla Public License version 1.1.

My websites: http://www.nicholson.com/rhn/  and  http://www.hotpaw.com/rhn/hotpaw/
