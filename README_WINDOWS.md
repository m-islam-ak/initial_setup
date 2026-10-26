# XBee teleop on Windows (no ROS)

## 1. One-time setup
1. Install Python 3.10 or newer from python.org. Tick "Add python.exe to PATH" in the installer.
2. Unzip this folder anywhere, for example `C:\xbee_teleop`.
3. Double-click `run_teleop.bat`. The first run creates a `.venv` and installs the packages
   (digi-xbee, pyserial, pygame, PyQt5), then starts the app. Later runs just start it.
   Manual alternative, in a terminal inside the folder:
       py -m venv .venv
       .venv\Scripts\activate
       pip install -r requirements.txt
       python xbee_teleop_windows.py
4. Plug in the XBee USB adapter. If no COM port appears, install its driver
   (FTDI for XBee Explorer / many adapters, Silicon Labs CP210x for others).
   Find the port in Device Manager -> Ports (COM & LPT), for example COM5.
5. Close XCTU, PuTTY or anything else that has the COM port open.

## 2. Check / fix the radios (Radio Check tab)
Both radios need AP=1 (API mode), the same ID (3196) and BD=7 (115200).
1. Select the port and baud 115200, press **Check**.
2. If AP is not 1, press **Fix (set AP=1 + save)**, unplug and replug the radio, then Check again.
3. The Check output shows the radio's 64-bit address. TX radio: 0013A200425DD789.
   RX radio: 0013A200425E9501.

## 3. Gamepad PC (TX mode)
1. Plug in the gamepad (Xbox-type pads work out of the box).
2. Start the app. Mode = **TX**, pick the XBee COM port, baud 115200.
   "Other radio addr" must be the RX (robot) radio address: 0013A200425E9501 (filled in by default).
3. Press **Connect**. The XBee TX panel turns green and shows the frame rate (about 20 Hz).
4. Drive: left stick = forward/back, right stick = turn. Deadman is off by default.
   To require it, set `require_enable=True` in the script, then hold LB.
5. No gamepad? Tick "Keyboard drive": arrows or WASD, with the window focused.
   Do not type in the address box while driving.
6. If the stick directions or buttons are wrong, read the Joystick panel (raw SDL axis values
   and pressed button numbers) and edit `axis_linear`, `axis_angular`, `enable_button`
   in the CONFIG block of `xbee_teleop_windows.py`.

## 4. Robot PC (RX mode)
1. Mode = **RX**, pick the XBee port. "Other radio addr" becomes the TX radio address
   0013A200425DD789 automatically.
2. Press **Connect**. The panel shows "receiving" while commands arrive.
3. Windows has no ROS /cmd_vel, so commands go to the hook `RxCore.default_on_cmd(vx, wz)`
   in the script. Put your motor-driver code there. It is called every 50 ms and gets 0, 0
   whenever no command has arrived for 0.5 s.
   If the robot stays on Ubuntu with ROS 2, keep running the original `rx_node` there
   and use only the TX mode on Windows.

## 5. Safety behaviour
- No gamepad data for 0.5 s: TX sends zero commands.
- No radio data for 0.5 s: RX outputs zeros to the motor hook.
- Closing the window sends a final zero command.
- Disconnect really disconnects; the app only auto-retries while connected.

## Troubleshooting
- "could not open port": wrong COM number, or another program holds the port.
- "radio not in API mode": Radio Check -> Fix.
- RX panel says "wrong source address": the address box on the RX side is not the TX radio.
- TX shows delivered=0 and failed>0: radios are on different IDs, out of range, or the remote address is wrong.
- Gamepad not found: plug it in; the app rescans every second.
