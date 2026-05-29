# Drum_intheloop TODO

## Frame-Level SIL

- [x] Replace named pipe ingress with SocketCAN and DXL serial protocol ingress.
- [x] Add `setup_sil.sh` for `vcan0..3` and `/dev/ttyUSB0` setup.
- [x] Split protocol code into `decoder.py`, `encoder.py`, `router.py`, and `mapping.py`.
- [ ] Validate TMotor discovery timing against `DrumRobot2::setMotorsSocket()`.
- [ ] Validate Maxon SDO setup response coverage during startup.
- [ ] Validate Dynamixel ping, sync write, and sync read with the SDK.
- [ ] Tune PyBullet motion timing beyond immediate `resetJointState()` updates.

## Notes

- `vcan` is a SocketCAN frame boundary simulator; it does not model bitrate, ACK, CRC, arbitration, or transceiver behavior.
- `DrumRobot2` uses real `can*` if any real CAN interface exists. It uses `vcan*` only when no real CAN interface exists.
- DXL ID 1 and ID 2 share one serial bus endpoint, `/dev/ttyUSB0`.
