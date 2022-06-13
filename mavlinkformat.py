#!/usr/bin/python3

import sys

f = open(sys.argv[1], 'rb')

buf = f.read()

it = iter(buf)

try:
    while True:
        while next(it) != 0xfd:
            pass

        length = next(it)

        iflags = next(it)
        cflags = next(it)

        seq = next(it)

        sysid = next(it)
        compid = next(it)

        msgid = 0

        for i in range(3):
            msgid |= next(it) << (8 * i)

        msg = []

        for i in range(length):
            msg.append(next(it))

        print(f"fd {length:02x} {iflags:02x}{cflags:02x} {seq:02x} {sysid:02x}{compid:02x}  {msgid:06x}", end=" ")

        mit = iter(msg)

        while True:
            try:
                for i in range(2):
                    print(f"{next(mit):02x}", end="")
                print(" ", end="")
            except StopIteration:
                print()
                break

        # crc
        next(it)
        next(it)
except StopIteration:
    print()
