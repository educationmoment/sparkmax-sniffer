# 

```
include/sm_can.h      frame struct + transport vtable, SLCAN + SocketCAN openers
include/sparkmax.h    ID packing, codec (pure), bus API (heartbeat, setpoints, status cache)
src/sparkmax.c        codec + bus           (C11, pthreads)
src/slcan.c           Lawicel SLCAN over termios
src/socketcan.c       Linux AF_CAN
tools/sm_sniff.cpp    C++17 decoder / RE tool built ON the C library
examples/sm_drive.c   ramp-and-telemetry example in plain C
tests/test_codec.c    codec + SLCAN parser tests
tests/fake_slcan.py   pty-based fake adapter + simulated SPARK MAX
```

## Build

```sh
cmake -S . -B build && cmake --build build -j && ctest --test-dir build
```

## Use

```sh
./build/sm_sniff --slcan /dev/ttyUSB0            # default UART 3 Mbaud, CAN 1 Mbit
./build/sm_sniff --slcan /dev/ttyUSB0 --dev 5 --csv log.csv

sudo slcand -o -c -s8 -S3000000 /dev/ttyUSB0 can0 && sudo ip link set can0 up
./build/sm_sniff --socketcan can0
./build/sm_drive --socketcan can0 --dev 5




```c
sm_transport t;
slcan_open(&t, "/dev/ttyUSB0", 0, SLCAN_1M);       // or socketcan_open(&t, "can0")
sm_bus *bus = sm_bus_create(t);                    
sm_bus_set_enabled(bus, 5, true);

// I/O thread (or your main loop): every few ms
sm_bus_tick(bus, 20);                               // heartbeat, SPARK times out at ~100 ms
sm_bus_poll(bus, 5, NULL);                          // rx + status cache

sm_set_duty(bus, 5, 0.2f);                          // also: voltage, velocity, position, current
sm_device_state s; sm_get_state(bus, 5, &s);        // s.s1.velocity_rpm, s.s2.position_rot ...
sm_bus_destroy(bus);                                // zero setpoints, disable, close
```

## tested with firmware 25
