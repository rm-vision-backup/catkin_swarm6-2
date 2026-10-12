#!/usr/bin/env python3
"""airlink_rx.py — 接收端（GCS）：统计每台发送机的到达速率、丢包、单向延迟。

与 airlink_tx.py 配套。UDP 按来源 IP 分机；TCP 每条连接 = 一台机。
单向延迟 = 收包时刻 - 包内 send_time。前提是飞机板钟已与 GCS 对齐
（onboard_clock_fix.sh 做过）；若未对齐，绝对延迟含固定偏置，看抖动更稳。

例：
  python3 airlink_rx.py --proto udp --port 5700 --size 1200 --duration 65
  python3 airlink_rx.py --proto tcp --port 5700 --size 1200 --duration 65
"""
import argparse
import socket
import struct
import sys
import threading
import time

HDR = struct.Struct("<Qd")          # seq(u64), send_time(f64 unix 秒)
LOCK = threading.Lock()
SRCS = {}                           # ip -> Src


class Src:
    def __init__(self, ip):
        self.ip = ip
        self.cnt = 0
        self.bytes = 0
        self.seq_min = None
        self.seq_max = None
        self.delays = []            # 秒
        self.first_arr = None
        self.last_arr = None
        self.win_bytes = 0
        self.win_t = None

    def on_packet(self, seq, ts, nbytes, now):
        self.cnt += 1
        self.bytes += nbytes
        self.win_bytes += nbytes
        if self.seq_min is None or seq < self.seq_min:
            self.seq_min = seq
        if self.seq_max is None or seq > self.seq_max:
            self.seq_max = seq
        self.delays.append(now - ts)
        if self.first_arr is None:
            self.first_arr = now
        self.last_arr = now

    def loss_pct(self):
        if self.seq_min is None:
            return 0.0
        expected = self.seq_max - self.seq_min + 1
        return 100.0 * (expected - self.cnt) / expected if expected > 0 else 0.0

    def delay_stats(self):
        if not self.delays:
            return None
        d = sorted(self.delays)
        n = len(d)
        return d[0], d[n // 2], d[min(n - 1, int(n * 0.99))], d[-1]


def record(ip, buf):
    if len(buf) < HDR.size:
        return
    seq, ts = HDR.unpack_from(buf, 0)
    now = time.time()
    with LOCK:
        s = SRCS.get(ip)
        if s is None:
            s = SRCS[ip] = Src(ip)
            s.win_t = time.monotonic()
        s.on_packet(seq, ts, len(buf), now)


def printer(stop, interval):
    while not stop.is_set():
        time.sleep(interval)
        with LOCK:
            if not SRCS:
                continue
            tot_win = 0
            for ip in sorted(SRCS):
                s = SRCS[ip]
                dt = max(1e-6, time.monotonic() - s.win_t)
                rate = s.win_bytes * 8 / dt / 1e6
                tot_win += s.win_bytes
                s.win_bytes = 0
                s.win_t = time.monotonic()
                ds = s.delay_stats()
                dstr = ("%.1f/%.1f/%.1f ms" % (ds[0] * 1e3, (sum(s.delays) / len(s.delays)) * 1e3, ds[3] * 1e3)) if ds else "-"
                print("  %-15s %6.3f Mbps  recv=%-8d loss=%5.1f%%  owdelay(min/avg/max)=%s"
                      % (ip, rate, s.cnt, s.loss_pct(), dstr))
            print("  %-15s %6.3f Mbps   [aggregate]" % ("TOTAL", tot_win * 8 / interval / 1e6))


def summary():
    with LOCK:
        if not SRCS:
            print("(未收到任何包)")
            return
        print("\n=== 汇总 ===")
        for ip in sorted(SRCS):
            s = SRCS[ip]
            dur = max(1e-6, s.last_arr - s.first_arr)
            ds = s.delay_stats()
            if ds:
                dst = "min=%.1f p50=%.1f p99=%.1f max=%.1f ms" % (
                    ds[0] * 1e3, ds[1] * 1e3, ds[2] * 1e3, ds[3] * 1e3)
            else:
                dst = "-"
            print("  %-15s pkts=%-8d rate=%6.3f Mbps  loss=%5.1f%%  delay(%s)"
                  % (ip, s.cnt, s.bytes * 8 / dur / 1e6, s.loss_pct(), dst))


def run_udp(port, size, duration, stop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    end = time.monotonic() + duration
    buf = bytearray(65535)
    while time.monotonic() < end:
        try:
            n, addr = sock.recvfrom_into(buf)
        except socket.timeout:
            continue
        record(addr[0], memoryview(buf)[:n])
    sock.close()


def handle_tcp(conn, size):
    try:
        conn.settimeout(5)
        ip = conn.getpeername()[0]
        need = size
        acc = bytearray()
        while True:
            chunk = conn.recv(65536)
            if not chunk:
                break
            acc += chunk
            while len(acc) >= need:
                record(ip, memoryview(acc)[:need])
                del acc[:need]
    except OSError:
        pass
    finally:
        conn.close()


def run_tcp(port, size, duration, stop):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(16)
    srv.settimeout(0.5)
    end = time.monotonic() + duration
    while time.monotonic() < end:
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        threading.Thread(target=handle_tcp, args=(conn, size), daemon=True).start()
    srv.close()


def main():
    ap = argparse.ArgumentParser(description="one-way uplink receiver (GCS side)")
    ap.add_argument("--proto", choices=("udp", "tcp"), default="udp")
    ap.add_argument("--port", type=int, default=5700)
    ap.add_argument("--size", type=int, default=1200, help="每包字节数，须与发送端一致")
    ap.add_argument("--duration", type=float, default=65.0, help="监听秒数")
    ap.add_argument("--interval", type=float, default=1.0, help="打印周期秒")
    args = ap.parse_args()

    print("listening proto=%s port=%d size=%dB duration=%ss"
          % (args.proto, args.port, args.size, args.duration))
    stop = threading.Event()
    th = threading.Thread(target=printer, args=(stop, args.interval), daemon=True)
    th.start()
    try:
        if args.proto == "udp":
            run_udp(args.port, args.size, args.duration, stop)
        else:
            run_tcp(args.port, args.size, args.duration, stop)
    except KeyboardInterrupt:
        print("(Ctrl-C)")
    stop.set()
    time.sleep(0.1)
    summary()
    return 0


if __name__ == "__main__":
    sys.exit(main())
