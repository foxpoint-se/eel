# Pressure / depth (Bar02)

## Hardware path

```
Bar02 (MS5837-02BA, I²C)
  → Arduino Nano running foxpoint-se/i2c-proxy
  → USB serial (binary floats)
  → eel pressure node
```

Bar02 itself is I²C only. We keep that bus short and put the Nano next to the sensor so the long run to the Pi is serial (less noise than a long I²C cable).

## i2c-proxy wire format

Repo: https://github.com/foxpoint-se/i2c-proxy

- Model: `MS5837_02BA`, freshwater density `997` kg/m³ (set on the Arduino)
- Every 200 ms (5 Hz): `depthSensor.depth()` as **4 little-endian float bytes** on UART @ **9600**
- No header, checksum, or delimiter — just raw `float32` frames

Bar02 install / I²C: https://bluerobotics.com/learn/bar-sensors-guide/
