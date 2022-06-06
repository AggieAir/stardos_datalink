# STARDOS datalink node

This package provides a link between two STARDOS systems. It depends on:

* [`stardos_interfaces`](https://github.com/AggieAir/stardos_interfaces)
* [MAVSDK](https://github.com/mavlink/MAVSDK) (must be compiled with MAVLink
  Passthrough support, see the [section](#getting-mavlink-to-work) on this
  below)
* [JsonCpp](https://github.com/open-source-parsers/jsoncpp)

This readme includes information on how to use this node more generally, how to
configure it, and the FloatTelem protocol.

## The Basic Gist

Let's say you have a whole bunch of heartbeats coming in from devices on the
aircraft. You want those to be visible with ground control. This node fills
that niche. It serializes and broadcasts heartbeat messages across a MAVLink
connection. It can also transmit control messages with up to 12 characters of
data in a similar fashion.

### Making it Listen

This node subscribes to control messages coming from `[name]/control` where
`[name]` is the name of the node. These control messages are expected to be
JSON strings with a format similar to:

```json
{
  "heartbeat": {
    "pub": ["/other_system/a/heartbeat", "/other_system/b/heartbeat"],
    "sub": ["/this_system/c/heartbeat", "/this_system/d/heartbeat"]
  },
  "control": {
    "pub": ["/other_system/action_a", "/other_system/action_b"],
    "sub": ["/this_system/action_c", "/this_system/action_d"]
  }
}
```

The `"heartbeat"` object contains a list (`"sub"`) of systems to subscribe to on
this side and send to the other side of the MAVLink connection and a list
(`"pub"`) of systems to listen to over mavlink and publish on this side.

The `"control"` object functions much the same way, but with
`stardos_interfaces/msg/Control` message instead of
`stardos_interfaces/msg/NodeHeartbeat` messages.

Send the same thing on the other side, but swap which are `pub`'d and which are
`sub`'d.

## Configuration

This package ships with two executables, one (`ground.cpp`) configured to run
in a ground control system, the other (`copilot.cpp`) configured to run on the
aircraft's copilot computer. They both instantiate the same node (the
`Datalink` class), but have different options.

### Names and namespaces

The ground executable creates a node called `/datalink_ground` by default. The
copilot executable creates a node called `/datalink_copilot`.  You may want to
[remap these names][ros2_rebinding] so that they fit in with the STARDOS
hierarchy.

### Parameters

Other options are modeled as [ROS2 parameters][ros2_params]. If you want to
configure the node some other way, you'll first need the full list of
parameters (run `ros2 param dump`). As of today (6 Jun 2022), these lists are:

```yaml
/datalink_copilot:
  ros__parameters:
    connection_url: udp://127.0.0.1:11001
    sysid: 1
    compid: 220
    targetsysid: 190
    targetcompid: 190
    heartbeat: true
    autopilot_telemetry: true
    use_sim_time: false

/datalink_ground:
  ros__parameters:
    connection_url: udp://127.0.0.1:11002
    sysid: 190
    compid: 190
    targetsysid: 1
    targetcompid: 220
    heartbeat: true
    autopilot_telemetry: false
    use_sim_time: false
```

**PLEASE NOTE:** if you remap the name or namespace of the node, the YAML file
containing the parameters will also need to change, particularly the line that
has the node name. Seems obvious, but it caught me for a bit.

Here's a rundown of all of the parameters:

#### `connection_url`

This is how you're going to connect to MAVLink. On the copilot side, you will
connect to the autopilot, which has a radio on board. On the GCS side, you will
most likely connect to an instance of [`mavlink-router`][mavlink_router], which
will provide radio access to the aircraft.

In a production environment, you won't be using UDP on the copilot, you'll be
using UART. Change `connection_url` to a serial connection as described on
[this page][mavlink_connecting]. If you don't know the baudrate, there are ways
to figure it out. Trial and error works, but I like to use the [mavlink
console][mavlink_console], which provides [quite a few useful
functions][px4_modules]. On the cubes, you can get to this interface by
plugging into the microUSB port on the side of the cube itself). From there,
run `mavlink status` and you'll see every instance of MAVLink running on the
cube as well as their baudrates.

#### `sysid`, `compid`, `targetsysid`, `targetcompid`

These are also MAVLink concepts. There are generally going to be two systems on
our MAVLink networks, one for the ground control and one for the aircraft. Each
unique device connecting to MAVLink is a component of the system and needs a
unique Component ID. That in mind:

* `sysid`: our system ID
* `compid`: our component ID
* `targetsysid`: the system ID of the computer that we want to send FloatTelem
  data to
* `targetcompid`: that computer's component ID

I don't think you should need to change these. I pulled the default values off
of some older code. I expect that other code will be working with these
constants in mind.

#### `heartbeat`

Whether or not MAVSDK should send heartbeats over MAVLink. I don't know the
downsides of doing this vs. not doing it, so both do it for now. If we're ever
constrained by bandwidth we can maybe see about getting rid of the heartbeats.

#### `autopilot_telemetry`

Whether to publish telemetry captured from the autopilot. This is only useful
to the payload right now, and there's no way to send it over FloatTelem yet, so
it's only true on the copilot.

These are just subscriptions to MAVLink messages that I forward to ROS2. Right
now, I publish:

* `gps_position` ([`GPS_RAW_INT`][mavlink_gps] messages)
* `attitude` ([`ATTITUDE`][mavlink_att] messages)
* `system_time` ([`SYSTEM_TIME`][mavlink_systime] messages)

## FloatTelem

FloatTelem is the name I've given to the protocol that I invented to get
telemetry over MAVLink. It hijacks the [`DEBUG_FLOAT_ARRAY`][mavlink_dfa]
message.

### Header

```
 bits    type    field
   0- 7   uint8   msg_type
   8-15   uint8   msg_length
  16-23   uint8   topic_id
```

The message type is just a constant that represents what kind of message it is
(only heartbeat or control at the moment).

The message length tells us in *bytes* how much space the message takes. For
some messages we need this information to decode it, for some it's just some
extra verification.

The topic ID represents which topic this should be published on. This will just
be the index of the topic name as given in the [control
message](#making-it-listen).

### Heartbeat Message

* Message ID: 0x1
* Message Length: 12

```
 bits    type     field
   0-23            header
  24-31            unused
  32-47   uint16   state
  48-63   uint16   errors
  64-79   uint16   requests
  80-95   uint16   failures
```

The contents of the fields here are an implemenation detail of STARDOS nodes.

### Control Message

* Message ID: 0x2
* Message Length: 3-16

```
 bits    type     field
   0-23            header
  24- n   string   message
```

The topic ID here represents which kind of message this is. The string is just
any extra data and can be up to 13 bytes long.

## Getting MAVLink to work

*(accurate as of 16 May 2022)*

If you want to get MAVLink set up on your computer, here are some pointers. I
spent 2.5 hours getting `Mavlink.cpp` and `Mavlink.hpp` to build correctly so
that I could modify them. Here's what you need to know:

### Installing MAVLink

MAVLink hates freedom so they disable MAVLink Passthrough by default. It can be
tricky to get everything to work, so we have to build it ourself. **Releases
from the MAVSDK GitHub page do not work.**

The best option to get everything you need is definitely still AggieCap3's
[`install_mavlink.sh`][install_mavlink] script, which downloads and installs
the `main` branch, which currently tracks release `1.4.0`.

### Compiling

Compile using C++ 17. If you do not, the header `plugin-base.h` (located at
`/usr/local/include/mavsdk/plugin_base.h`) will not work.

If you're using a `Makefile` the way to do this is by adding `-std=c++17` to
`CFLAGS`. If you are using another build tool, look it up.

### Linking

Assuming you installed MAVLink the way I descirbed above, you will need to
include link `mavsdk` (can be done by adding `-lmavsdk` to `LDFLAGS`). No other
libraries are necessary to get MAVLink to work.

This is how I eventually forced MAVLink into submission with my Makefile:

```
CC=g++
CFLAGS=-c -Wall -std=c++17 -I /usr/local/include/mavsdk -I /usr/local/include/ -libxml++-2.6 -g
LDFLAGS=-L./ -lpthread -lmavsdk
```

### Linking Part 2

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

### Linking Part 3

If somehow you got to this point and it still doesn't find it, random googling
is what got me the furthest. Here are some things you can try to diagnose weird
linking errors:

* `ld -lmavsdk --verbose` to see where `ld` is looking for your library.
* set `LD_DEBUG=all` to see a ton of debugging information about linking.

### Cmake

The next frustrating thing I found was that you need some extra lines of code
to get this all to work with Cmake (which is required if you're doing ROS
stuff). [This page][mavlink_cmake] had some useful tips but it was also a bit
misleading. In my experience, all you need to add to your `CMakeLists.txt` is:

```
target_link_libraries(target MAVSDK::mavsdk)
```

### But what about plugins?

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

[ros2_params]: https://docs.ros.org/en/foxy/Tutorials/Parameters/Understanding-ROS2-Parameters.html
[ros2_rebinding]: https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html#name-remapping
[install_mavlink]: https://github.com/AggieAir/AggieCap3/blob/master/install_mavlink.sh
[mavlink_cmake]: https://mavsdk.mavlink.io/main/en/cpp/guide/toolchain.html
[mavlink_connecting]: https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html
[mavlink_router]: https://github.com/mavlink-router/mavlink-router
[mavlink_console]: https://docs.qgroundcontrol.com/master/en/analyze_view/mavlink_console.html
[mavlink_gps]: https://mavlink.io/en/messages/common.html#GPS_RAW_INT
[mavlink_att]: https://mavlink.io/en/messages/common.html#ATTITUDE
[mavlink_dfa]: https://mavlink.io/en/messages/common.html#DEBUG_FLOAT_ARRAY
[mavlink_systime]: https://mavlink.io/en/messages/common.html#SYSTEM_TIME
[px4_modules]: https://docs.px4.io/master/en/modules/modules_main.html
