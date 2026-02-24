#!/usr/bin/env python3
import socket, struct, zlib
from dataclasses import dataclass

HOST = "twr-repeater.local"
PORT = 9000

MAGIC = b"TWR1"
HEADER_SZ = 32
TRAILER_SZ = 4

@dataclass
class Header:
    magic: bytes
    version: int
    msg_type: int
    sample_rate_hz: int
    sample_bytes: int
    sample_count: int
    start_index: int
    payload_bytes: int
    payload_crc32: int
    header_crc32: int

def recv_exact(s: socket.socket, n: int) -> bytes:
    chunks = []
    got = 0
    while got < n:
        b = s.recv(n - got)
        if not b:
            raise ConnectionError("Socket closed while receiving")
        chunks.append(b)
        got += len(b)
    return b"".join(chunks)

def recv_until_magic(s: socket.socket, magic: bytes) -> bytes:
    # Read byte-by-byte until we see MAGIC. Simple + robust.
    buf = bytearray()
    while True:
        b = s.recv(1)
        if not b:
            raise ConnectionError("Socket closed before magic")
        buf += b
        if len(buf) >= len(magic) and bytes(buf[-len(magic):]) == magic:
            return bytes(buf[-len(magic):])  # return MAGIC

def read_header(s: socket.socket) -> Header:
    # Find magic, then read rest of header
    recv_until_magic(s, MAGIC)
    rest = recv_exact(s, HEADER_SZ - 4)
    raw = MAGIC + rest

    # Little-endian: 4s H H H H I I I I I
    # Fields:
    # magic[4]
    # version u16
    # msg_type u16
    # sample_rate_hz u16
    # sample_bytes u16
    # sample_count u32
    # start_index u32
    # payload_bytes u32
    # payload_crc32 u32
    # header_crc32 u32
    fmt = "<4s H H H H I I I I I"
    vals = struct.unpack(fmt, raw)
    return Header(*vals)

def unpack_samples(payload: bytes, sample_bytes: int):
    if sample_bytes != 29:
        raise ValueError(f"Expected sample_bytes=29, got {sample_bytes}")

    # Sample layout (packed, little-endian):
    # 6x i16, 1x u8, 2x i32, 1x i32, 2x i16
    # => "<hhhhhhBiiihh" is 12+1+12+4 = 29 bytes
    sample_fmt = "<hhhhhhBiiihh"
    assert struct.calcsize(sample_fmt) == 29

    samples = []
    for off in range(0, len(payload), sample_bytes):
        chunk = payload[off:off+sample_bytes]
        if len(chunk) != sample_bytes:
            raise ValueError("Truncated sample at end of payload")

        (posL, posR, velL, velR, dL, dR, flags,
         unwrapL, unwrapR, x_mm, xdot_mm_s, xdot_from_pos_mm_s) = struct.unpack(sample_fmt, chunk)

        samples.append({
            "pos_L_raw_mrad": posL,
            "pos_R_raw_mrad": posR,
            "vel_L_raw_mrad_s": velL,
            "vel_R_raw_mrad_s": velR,
            "dL_mrad": dL,
            "dR_mrad": dR,
            "accept_L": bool(flags & 0x01),
            "accept_R": bool(flags & 0x02),
            "unwrap_L_mrad": unwrapL,
            "unwrap_R_mrad": unwrapR,
            "x_wheel_mm": x_mm,
            "x_dot_mm_s": xdot_mm_s,
            "x_dot_from_pos_mm_s": xdot_from_pos_mm_s,
        })
    return samples

def main():
    with socket.create_connection((HOST, PORT), timeout=10) as s:
        s.settimeout(None)
        print(f"Connected to {HOST}:{PORT}, waiting for TWR1...")

        hdr = read_header(s)
        print("Header:", hdr)

        payload = recv_exact(s, hdr.payload_bytes)
        trailer_crc = struct.unpack("<I", recv_exact(s, 4))[0]

        calc_crc = zlib.crc32(payload) & 0xFFFFFFFF

        print(f"CRC header=0x{hdr.payload_crc32:08X} trailer=0x{trailer_crc:08X} calc=0x{calc_crc:08X}")
        if calc_crc != hdr.payload_crc32 or trailer_crc != hdr.payload_crc32:
            raise ValueError("CRC mismatch")

        if hdr.sample_bytes * hdr.sample_count != hdr.payload_bytes:
            raise ValueError("Header length fields inconsistent")

        samples = unpack_samples(payload, hdr.sample_bytes)
        print(f"Unpacked {len(samples)} samples. First sample:", samples[0])

if __name__ == "__main__":
    main()
