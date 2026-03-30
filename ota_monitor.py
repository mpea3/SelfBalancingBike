"""Simple OTA serial monitor for the BikeProject firmware."""

import argparse
import socket
import sys
import time


def monitor(host: str, port: int, reconnect: bool, reconnect_delay: float) -> None:
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)

        try:
            print(f"Connecting to {host}:{port} ...")
            sock.connect((host, port))
            sock.settimeout(None)
            print("Connected. Press Ctrl+C to stop.")

            while True:
                data = sock.recv(1024)
                if not data:
                    raise ConnectionError("Connection closed by device")
                sys.stdout.buffer.write(data)
                sys.stdout.flush()

        except KeyboardInterrupt:
            print("\nStopped.")
            return
        except Exception as exc:
            print(f"Monitor error: {exc}")
            if not reconnect:
                raise SystemExit(1) from exc
            print(f"Reconnecting in {reconnect_delay:.1f}s ...")
            time.sleep(reconnect_delay)
        finally:
            sock.close()


def get_host_from_pio() -> str | None:
    try:
        import configparser
        config = configparser.ConfigParser()
        config.read("platformio.ini")
        return config["env:nano33iot"].get("upload_port")
    except Exception:
        return None

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OTA serial monitor client")
    
    default_host = get_host_from_pio()
    parser.add_argument(
        "--host", 
        default=default_host,
        required=(default_host is None),
        help="Board IP address (auto-detected from platformio.ini if available)"
    )
    parser.add_argument("--port", type=int, default=65281, help="OTA serial TCP port")
    parser.add_argument(
        "--reconnect",
        action="store_true",
        help="Reconnect automatically if the connection drops",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=2.0,
        help="Seconds to wait before reconnecting",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    monitor(args.host, args.port, args.reconnect, args.reconnect_delay)
