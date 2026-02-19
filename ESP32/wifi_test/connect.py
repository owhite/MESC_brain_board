#!/usr/bin/env python3

import socket, time
s = socket.create_connection(("twr-repeater.local", 9000), timeout=5)
s.settimeout(1.0)
t0 = time.time()
total = 0
while time.time() - t0 < 5:
    try:
        d = s.recv(4096)
        if not d: break
        total += len(d)
    except socket.timeout:
        pass
print("received", total, "bytes")
s.close()
