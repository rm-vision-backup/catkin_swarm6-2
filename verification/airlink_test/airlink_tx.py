#!/usr/bin/env python3
"""airlink_tx.py — 单向上行流量发生器（飞机 → GCS），只发不收。

每包载荷固定 --size 字节：前 16 B = seq(u64 LE) + send_time(f64 LE, unix 秒)，
其余填 0。接收端 airlink_rx.py 按 seq 算丢包、按 send_time 算单向延迟。

用途：判断链路瓶颈是否在电台"发射端"。
  - UDP：不重传、不等 ACK → 真实反映 TX 侧能打出多少（丢包直接暴露）。
  - TCP：可靠但会因重传/停等自限速（作对比）。

例：
  python3 airlink_tx.py --proto udp --host 192.168.5.241 --rate 1M --duration 60
  python3 airlink_tx.py --proto tcp --host 192.168.5.241 --rate 1M --duration 60
"""
import argparse
import socket
import struct
import sys
import time

HDR = struct.Struct("<Qd")          # seq(u64), send_time(f64 unix 秒)


def parse_rate(s):
    t = s.strip().lower()
    for suf, mul in (("gbps", 1e9), ("mbps", 1e6), ("kbps", 1e3),
                     ("g", 1e9), ("m", 1e6), ("k", 1e3)):
        if t.endswith(suf):
            return int(float(t[:-len(suf)]) * mul)
    return int(float(t))


def main():
    ap = argparse.ArgumentParser(description="one-way uplink sender (aircraft -> GCS)")
    ap.add_argument("--proto", choices=("udp", "tcp"), default="udp")
    ap.add_argument("--host", required=True, help="GCS IP (e.g. 192.168.5.241)")
    ap.add_argument("--port", type=int, default=5700)
    ap.add_argument("--rate", type=parse_rate, default=parse_rate("1M"),
                    help="target bit rate, e.g. 1M / 2M / 800k")
    ap.add_argument("--size", type=int, default=1200, help="payload bytes per packet")
    ap.add_argument("--duration", type=float, default=60.0, help="seconds")
    args = ap.parse_args()

    if args.size < HDR.size:
        print("--size 至少 %d 字节" % HDR.size, file=sys.stderr)
        return 2

    interval = args.size * 8.0 / args.rate            # 秒/包
    print("proto=%s dest=%s:%d rate=%.3f Mbps size=%dB interval=%.2f ms duration=%ss"
          % (args.proto, args.host, args.port, args.rate / 1e6, args.size,
             interval * 1000.0, args.duration))

    payload = bytearray(args.size)
    if args.proto == "udp":
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest = (args.host, args.port)
    else:
        sock = socket.create_connection((args.host, args.port), timeout=10)

    start = time.monotonic()
    last_rep = start
    sent = 0
    sent_bytes = 0
    i = 0
    try:
        while True:                                  # 以时间窗为准；TCP 阻塞时包数自然变少
            el = time.monotonic() - start
            if el >= args.duration:
                break
            sched = i * interval
            if sched > el:
                time.sleep(sched - el)
            HDR.pack_into(payload, 0, i, time.time())
            if args.proto == "udp":
                sock.sendto(payload, dest)
            else:
                sock.sendall(payload)
            sent += 1
            sent_bytes += args.size
            i += 1
            now = time.monotonic()
            if now - last_rep >= 1.0:
                el2 = now - start
                print("  t=%5.1fs sent=%8d pkt avg=%6.3f Mbps"
                      % (el2, sent, sent_bytes * 8 / el2 / 1e6))
                last_rep = now
    except KeyboardInterrupt:
        print("(Ctrl-C)")
    finally:
        sock.close()

    el = time.monotonic() - start
    print("[done] packets=%d bytes=%d avg=%.3f Mbps over %.1fs (target %.3f Mbps)"
          % (sent, sent_bytes, sent_bytes * 8 / el / 1e6, el, args.rate / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
