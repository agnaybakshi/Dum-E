from __future__ import annotations

import argparse
import socket
import time

from transport import TeleopCommand


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send teleop-compatible UDP test packets to the ESP32 firmware.")
    parser.add_argument("--host", default="192.168.137.50", help="ESP32 IP or hostname.")
    parser.add_argument("--port", type=int, default=4210, help="ESP32 UDP listen port.")
    parser.add_argument("--mode", choices=("A", "H", "S", "M"), default="A")
    parser.add_argument("--base", type=float, default=0.0, help="Normalized base command in [-1, 1].")
    parser.add_argument("--lower", type=float, default=0.0, help="Lower joint angle in degrees.")
    parser.add_argument("--middle", type=float, default=0.0, help="Middle joint angle in degrees.")
    parser.add_argument("--upper", type=float, default=0.0, help="Upper joint angle in degrees.")
    parser.add_argument("--gripper", type=float, default=0.5, help="Gripper openness in [0, 1].")
    parser.add_argument("--rate", type=float, default=20.0, help="Send rate in Hz.")
    parser.add_argument("--count", type=int, default=1, help="Number of packets to send. Use 0 for continuous.")
    parser.add_argument(
        "--listen-timeout",
        type=float,
        default=0.0,
        help="Wait this many seconds for an ESP32 UDP reply after each packet. Use 0 to disable.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    address = (args.host, args.port)
    sequence = 0
    interval = 1.0 / max(args.rate, 1e-3)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        if args.listen_timeout > 0.0:
            sock.settimeout(args.listen_timeout)
        try:
            while args.count == 0 or sequence < args.count:
                sequence += 1
                command = TeleopCommand(
                    mode=args.mode,
                    base_command=args.base,
                    lower_deg=args.lower,
                    middle_deg=args.middle,
                    upper_deg=args.upper,
                    gripper_open=args.gripper,
                    sequence=sequence,
                )
                payload = command.encode()
                sock.sendto(payload, address)
                print(payload.decode("ascii").strip())
                if args.listen_timeout > 0.0:
                    try:
                        reply, source = sock.recvfrom(256)
                        print(f"<- {source[0]}:{source[1]} {reply.decode('ascii', errors='replace').strip()}")
                    except socket.timeout:
                        print("<- no reply")
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
