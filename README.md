# External XYZ velocity bridge

This is a separate Cargo package. It does not change or replace the existing
`rustbot_cntrl` binary, modules, or Cargo manifest.

Protocol: newline-delimited JSON over TCP, default `127.0.0.1:45890`.

The Python adapter sends:

```json
{"type":"velocity_command","sequence":1,"timestamp_ns":0,"valid_until_ns":0,"frame":"abb_base","velocity_mm_s":[1,0,0],"rotation_velocity_rad_s":[0,0,0],"valid":true}
```

The bridge validates the frame, sequence, finite values, zero rotational
velocity, deadline, component speed limits, and watchdog. It sends ABB EGM
speed packets with the current EGM feedback XYZ pose and quaternion. The
orientation target is never changed. A live command is accepted only while
fresh EGM feedback is present.

## Dry-run

```powershell
cargo run --release -- --mode dry-run --fake-eef-mm 300,150,600
```

No ABB TCP/UDP socket is opened. A fake EEF position is integrated so the
Python end-to-end path can be tested.

## Live gate

Live EGM is deliberately opt-in:

```powershell
cargo run --release -- --mode live --arm
```

Before this command, verify the ABB control-box IP/port, the PC ABB network
interface, the EGM local bind address, RAPID EGM setup, and a zero-speed packet
with the supervisor present. If the Python client disconnects or stops sending,
the bridge sends zero speed after 250 ms. A live run requires the explicit
`--arm` flag and the Python adapter also refuses incomplete 8D observations.
