# Host tests

Compiles the driver against a mock Arduino/Wire implementation and feeds it
synthetic SHTP cargoes, then checks the decoded values against the numbers in
the CEVA documents. No hardware required.

```bash
cd test
make
```

98 assertions covering:

| Area | What is checked |
|---|---|
| Start-up | SHTP advertisement parsing, report-length TLV, Product ID decode, `verifyChip()` |
| Quaternions | Q14 / Q12 scaling, identity, ±90° yaw, compass heading wrap |
| Euler | gimbal-lock singularity, clamped `asin`, no NaN on denormalised input |
| Physical sensors | accel Q8, gyro Q9, mag Q4, deg/s helper |
| Batched cargo | base timestamp + three reports in one packet, per-report accuracy, timestamp reconstruction |
| Engines | step counter, tap latch/clear, activity classifier confidences |
| Channel 5 | gyro-integrated RV with no report prefix, Q10 angular velocity |
| Raw reports | ADC counts and sensor timestamp offsets |
| Command encoding | Set Feature (all 17 bytes), Tare, ME calibration, Save DCD, executable sleep/on |
| FRS | serial number read, empty-record error path |
| Transport | cargo larger than one I2C transaction, unknown report ID resync |
| UART-RVC | the worked example and checksum from the datasheet |

The stub headers under `stub/` are a minimal Arduino API — they exist only so
the driver can be compiled on a PC, and are not part of the library.
