# Getting MAVLink to work

*(accurate as of 13 May 2022)*

If you want to get MAVLink set up on your computer, here are some pointers. I
spent 2.5 hours getting `Mavlink.cpp` and `Mavlink.hpp` to build correctly so
that I could modify them. Here's what you need to know:

## Installing MAVLink

MAVLink hates freedom so they disable MAVLink Passthrough by default. It can be
tricky to get everything to work, so we have to build it ourself. **Releases
from the MAVSDK GitHub page do not work.**

The best option to get everything you need is definitely still AggieCap3's
[`install_mavlink.sh`](https://github.com/AggieAir/AggieCap3/blob/master/install_mavlink.sh)
script, which downloads and installs the `main` branch, which currently tracks
release `1.4.0`.

## Compiling

Compile using C++ 17. If you do not, the header `plugin-base.h` (located at
`/usr/local/include/mavsdk/plugin_base.h`) will not work.

If you're using a `Makefile` the way to do this is by adding `-std=c++17` to
`CFLAGS`. If you are using another build tool, look it up.

## Linking

Assuming you installed MAVLink the way I descirbed above, you will need to
include link `mavsdk` (can be done by adding `-lmavsdk` to `LDFLAGS`). No other
libraries are necessary to get MAVLink to work.

This is how I eventually forced MAVLink into submission with my Makefile:

```
CC=g++
CFLAGS=-c -Wall -std=c++17 -I /usr/local/include/mavsdk -I /usr/local/include/ -libxml++-2.6 -g
LDFLAGS=-L./ -lpthread -lmavsdk
```

## Linking Part 2

If you still can't get it to link, make sure you ran `sudo ldconfig`. If it
still doesn't link, check `/usr/local/lib/`. You should have three
mavlink-related files there:

- `libmavsdk.so.1.4.0`
- `libmavsdk.so.1`
- `libmavsdk.so`

If the first is missing, you didn't install MAVLink the way I described above.
Go do that.

If the second and/or the third are missing, then one of two things has
happened.

1. You didn't run `sudo ldconfig`. Go do that.
2. `ldconfig` is whigging out. In this case, create the missing files as
   symlinks to the one that exists. C++ doesn't always look at versioned `.so`
   files; `ldconfig` is supposed to create the bare `.so` file, but sometimes
   it doesn't. This solution is messy, but sometimes the only one we have.

## Linking Part 3

If somehow you got to this point and it still doesn't find it, random googling
is what got me the furthest. Here are some things you can try to diagnose weird
linking errors:

* `ld -lmavsdk --verbose` to see where `ld` is looking for your library.
* set `LD_DEBUG=all` to see a ton of debugging information about linking.

## Cmake

The next frustrating thing I found was that you need some extra lines of code
to get this all to work with Cmake (which is required if you're doing ROS
stuff). [This page](https://mavsdk.mavlink.io/main/en/cpp/guide/toolchain.html)
had some useful tips but it was also a bit misleading. In my experience, all
you need to add to your `CMakeLists.txt` is:

```
target_link_libraries(target MAVSDK::mavsdk)
```

## But what about plugins?

A lot of MAVSDK documentation (like the documentation I used when in the
Linking Part 1 and Cmake sections) said I needed to separate mavsdk and the
plugins when I link. This turned out to be untrue in my experience, and trying
to pass `-lmavsdk_passthrough` or include `MAVSDK::mavsdk_passthrough` in my
link libraries, for instance, made things fail. Just `mavsdk` is sufficient.

Because computers are dumb, your setup may not work the way mine does. If so,
try adding these additional flags and see how it goes.

## Questions or Comments

I am not qualified to give high-quality help, but feel free to reach out to me
on slack (I'm Richard Snider btw) if you need help with this garbage.
