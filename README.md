# USB-CAN-B Code 

This repository contains CAN examples for a dual-channel USB-CAN adapter on Linux (USB-CAN-B adapter by Waveshare)
<img width="960" height="630" alt="USB-CAN-B-details-1" src="https://github.com/user-attachments/assets/cc49116f-2bb5-44b3-b323-8de12e3f8982" />


It includes:

- A Python Tkinter GUI for sending, receiving, decoding, and exporting CAN traffic.
- C++ sample programs linked against `libcontrolcan.so`.

## Licensing and Third-Party Materials

- Original implementation by the repository author: `python/can_gui.py`.
- Third-party code/resources: all remaining CAN-related files in this repository are based on official Waveshare materials.
- Upstream source reference: [Waveshare USB-CAN-B Linux guide](https://www.waveshare.com/wiki/USB-CAN-B#Working_with_Linux).
- Third-party copyrights and related rights remain with their respective owners.
- Use and redistribution of Waveshare-provided files should follow Waveshare terms and permissions.
- This project is an independent work and is not officially affiliated with or endorsed by Waveshare.

## Repository Layout

```text
.
|- c/
|  |- Makefile
|  |- main.cpp
|  |- can1_tx_can2_rx.cpp
|  |- controlcan.h
|  |- libcontrolcan.so
|  |- libcontrolcan.a
|  |- hello_cpp
|  `- can1_tx_can2_rx
`- python/
   |- can_gui.py
   |- README.md
   |- can_gui_session.json
   `- python3-64bit.py
```

## Python GUI

<img width="1206" height="802" alt="image" src="https://github.com/user-attachments/assets/f90b47ef-080a-4eb1-bc97-72b6a85d2e7e" />


Main entry point:

- `python/can_gui.py`

Detailed GUI documentation:

- `python/README.md`

### GUI Capabilities

- Connect/disconnect CAN1 and CAN2 with selectable baud rates.
- Create and manage message templates.
- Send one-shot or periodic CAN frames.
- Run CAN1<->CAN2 self-test.
- View grouped raw and decoded traffic by CAN ID.
- Configure decode rules per CAN ID.
- Export RX logs to CSV.
- Persist session state in `python/can_gui_session.json`.

## Requirements

- Linux.
- Python 3.
- Tkinter (`python3-tk`).
- ZLGCAN-compatible `libcontrolcan.so`.
- USB-CAN hardware.

Install Tkinter if needed:

```bash
sudo apt update
sudo apt install -y python3-tk
```

## Quick Start (GUI)

From repository root:

```bash
cd python
python3 can_gui.py
```

If device permissions require elevated access:

```bash
cd python
sudo python3 can_gui.py
```

In the GUI:

1. Set Library path to `../c/libcontrolcan.so`.
2. Select CAN1/CAN2 baud rates.
3. Click Connect.
4. Optionally run Self Test.

## Quick Start (C++ Samples)

From repository root:

```bash
cd c
make
```

Run samples:

```bash
./hello_cpp
./can1_tx_can2_rx
```

## Important Architecture Note

On Jetson (aarch64), use `c/libcontrolcan.so`.

The `python/libcontrolcan.so` file in this repository may be x86-64 and can fail to load on Jetson.

Check library architecture with:

```bash
file c/libcontrolcan.so
```

## Troubleshooting

### GUI cannot connect or open device

- Verify adapter is connected.
- Verify no other process is using the device.
- Try running GUI with `sudo`.

### `libcontrolcan.so` load error

- Confirm GUI library path points to `../c/libcontrolcan.so`.
- Confirm `.so` architecture matches the system.

### No RX frames

- Verify CAN wiring, ground, and termination.
- Verify matching baud rates on both nodes.

## Notes

- `python/can_gui_session.json` is auto-generated/updated by the GUI to persist state.
- CSV exports are generated from the GUI RX export buttons.
