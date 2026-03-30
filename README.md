## BikeProject OTA

### Upload firmware over OTA

1. Set your board IP in `platformio.ini` with `upload_port`.
2. Run:

```bash
pio run -e nano33iot -t upload
```

### Serial monitor over OTA

The firmware exposes a TCP serial stream on `custom_ota_monitor_port` (default `65281`).

Run:

```bash
python ota_monitor.py --host 192.168.137.86 --port 65281 --reconnect
```

If you enable telemetry (`balancer.consoleLog();`) you will see the same logs on USB serial and OTA monitor.
