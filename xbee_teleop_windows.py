#!/usr/bin/env python3
"""
xbee_teleop_tx_windows.py -- Windows TRANSMITTER for the ROS 2 'xbee_teleop' package (no ROS needed).

  gamepad / keyboard -> velocity command -> XBee (API mode, AP=1) -> the robot's rx_node (ROS 2)
  GUI: Joystick | Command | XBee TX | Robot ACK panels, plus a Radio Check tab.

Payload (identical to the ROS package):  CMD 'C', seq u8, vx i16, wz i16 (value*1000)
Safety (identical): optional deadman; no gamepad data for cmd_timeout -> zeros are sent;
                    closing the window sends a final zero command.

Run:  pip install pygame PyQt5 digi-xbee pyserial   then   python xbee_teleop_tx_windows.py

CHANGES vs. previous version
  * Gamepad detection: no more joystick.quit()/init() cycling (unreliable on pygame 2).
    Uses get_count() + JOYDEVICEADDED/JOYDEVICEREMOVED events (hot-plug works).
  * pygame is initialised BEFORE the timers start.
  * "XBee TX" panel is now clearly labelled RADIO-LEVEL: "Delivered" only means the robot's
    XBee *hardware* acknowledged the frame - it says nothing about software running on the robot.
  * "Robot ACK" panel is the ONLY indicator that the robot's rx_node is alive
    (it needs a real 'A' packet from the robot radio). Stale ACKs are cleared on disconnect.
  * Extra diagnostics: number of devices pygame sees, ACK counter.
"""
import ctypes
import os
import sys
import struct
import threading
import time

os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')   # keep reading the pad when the window is unfocused
os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')

# --------------------------------------------------------------------------
# CONFIG
# --------------------------------------------------------------------------
ROBOT_RADIO_ADDR = '0013A200425E9501'  # 64-bit address of the radio on the robot

CONFIG = dict(
    port='COM3',
    baud=115200,
    joy_index=0,
    joy_deadzone=0.05,
    send_rate_hz=20.0,
    cmd_timeout=0.5,
    # pygame/SDL numbering for an Xbox-type pad:
    #   axes   0 LX, 1 LY, 2 RX, 3 RY, 4 LT, 5 RT   (SDL: up / left = NEGATIVE; flipped below so up/left = +)
    #   buttons 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Back, 7 Start, 8 L-stick click, 9 R-stick click
    # Watch the raw values in the Joystick panel and change these if your pad differs.
    enable_button=4,                   # LB
    require_enable=False,              # True -> must hold LB to drive
    scale_linear=0.5,
    scale_angular=1.0,
    drive_mode='arcade',               # 'arcade' | 'tank'
    axis_linear=1,                     # arcade: left stick Y  -> vx
    axis_angular=2,                    # arcade: right stick X -> wz
    axis_left=1,                       # tank:   left stick Y
    axis_right=3,                      # tank:   right stick Y
)

# ==========================================================================
# 1. Payloads
# ==========================================================================
_CMD = struct.Struct('<cBhh')
_ACK = struct.Struct('<cBH')


def _i16(v):
    return max(-32768, min(32767, int(round(v * 1000))))


def pack_cmd(seq, vx, wz):
    return _CMD.pack(b'C', seq & 0xFF, _i16(vx), _i16(wz))


def unpack(data):
    """-> ('cmd', seq, vx, wz) | ('ack', seq, lost) | None"""
    data = bytes(data)
    if data[:1] == b'C' and len(data) == _CMD.size:
        _, seq, vx, wz = _CMD.unpack(data)
        return ('cmd', seq, vx / 1000.0, wz / 1000.0)
    if data[:1] == b'A' and len(data) == _ACK.size:
        _, seq, lost = _ACK.unpack(data)
        return ('ack', seq, lost)
    return None


def norm_addr(a):
    return str(a).strip().upper().replace('0X', '').zfill(16)


# ==========================================================================
# 1b. Direct XInput reader (Xbox 360 / One / Series pads) - no pygame needed
# ==========================================================================
class XInputPad:
    """Reads the pad straight from Windows XInput; output uses the same layout as pygame/SDL
    (axes 0 LX,1 LY,2 RX,3 RY,4 LT,5 RT with up = negative; buttons 0 A,1 B,2 X,3 Y,4 LB,5 RB,6 Back,7 Start,8 LS,9 RS)."""
    _BTN = [(0x1000, 0), (0x2000, 1), (0x4000, 2), (0x8000, 3), (0x0100, 4),
            (0x0200, 5), (0x0020, 6), (0x0010, 7), (0x0040, 8), (0x0080, 9)]

    class _State(ctypes.Structure):
        _fields_ = [("pkt", ctypes.c_uint32), ("btn", ctypes.c_uint16), ("lt", ctypes.c_ubyte),
                    ("rt", ctypes.c_ubyte), ("lx", ctypes.c_short), ("ly", ctypes.c_short),
                    ("rx", ctypes.c_short), ("ry", ctypes.c_short)]

    def __init__(self):
        self.fn = None
        if sys.platform == 'win32':
            for dll in ('xinput1_4', 'xinput1_3', 'xinput9_1_0'):
                try:
                    self.fn = getattr(ctypes.windll, dll).XInputGetState
                    break
                except Exception:
                    pass

    @staticmethod
    def _n(v):
        return max(-1.0, min(1.0, v / 32767.0))

    def read(self, index):
        """-> (axes, buttons) or None when no pad in that slot."""
        if self.fn is None:
            return None
        st = self._State()
        if self.fn(int(index), ctypes.byref(st)) != 0:
            return None
        axes = [self._n(st.lx), -self._n(st.ly), self._n(st.rx), -self._n(st.ry),
                st.lt / 127.5 - 1.0, st.rt / 127.5 - 1.0]
        buttons = [0] * 10
        for mask, i in self._BTN:
            buttons[i] = 1 if st.btn & mask else 0
        return axes, buttons


# ==========================================================================
# 2. XBeeLink
# ==========================================================================
HAVE_XBEE = True
try:
    from digi.xbee.devices import RemoteXBeeDevice, XBeeDevice
    from digi.xbee.models.address import XBee64BitAddress
    from digi.xbee.models.status import TransmitStatus
    from digi.xbee.packets.common import TransmitStatusPacket
except ImportError:
    HAVE_XBEE = False


class XBeeLink:
    def __init__(self, port, baud, remote_addr, on_message):
        self.port, self.baud = port, int(baud)
        self.remote_addr = norm_addr(remote_addr)
        self.on_message = on_message          # called from the XBee thread
        self.dev = self.remote = None
        self.local_addr = ''
        self.err = ''
        self.sent = self.tx_ok = self.tx_fail = 0
        self.rx = self.rx_ignored = 0
        self.last_src = ''
        self.last_fail = ''

    @property
    def connected(self):
        return self.dev is not None

    def connect(self):
        try:
            dev = XBeeDevice(self.port, self.baud)
            dev.open()
            dev.set_sync_ops_timeout(1.0)
            dev.add_data_received_callback(self._on_data)
            dev.add_packet_received_callback(self._on_packet)
            self.remote = RemoteXBeeDevice(dev, XBee64BitAddress.from_hex_string(self.remote_addr))
            self.local_addr = str(dev.get_64bit_addr())
            self.dev, self.err = dev, ''
        except Exception as e:
            msg = str(e) or type(e).__name__
            if 'operating mode' in msg.lower():
                msg += '  -> radio not in API mode: Radio Check tab -> Fix'
            self.err, self.dev = msg, None

    def send(self, payload):
        dev = self.dev
        if not dev:
            return False
        try:
            dev.send_data_async(self.remote, payload)
            self.sent += 1
            return True
        except Exception as e:
            self.err = f'send failed: {e}'
            self.close()
            return False

    def _on_data(self, msg):
        src = str(msg.remote_device.get_64bit_addr())
        self.last_src = src
        if norm_addr(src) != self.remote_addr:      # compare normalised (case / leading zeros)
            self.rx_ignored += 1
            return
        self.rx += 1
        self.on_message(bytes(msg.data))

    def _on_packet(self, pkt):
        # NOTE: this is the XBee *hardware* delivery status (MAC-level ACK from the robot's radio).
        # It is SUCCESS whenever the robot's radio is powered and in range, even if no software runs on the robot.
        if isinstance(pkt, TransmitStatusPacket):
            if pkt.transmit_status == TransmitStatus.SUCCESS:
                self.tx_ok += 1
            else:
                self.tx_fail += 1
                st = pkt.transmit_status
                self.last_fail = str(getattr(st, 'description', st))

    def close(self):
        dev, self.dev = self.dev, None
        try:
            if dev and dev.is_open():
                dev.close()
        except Exception:
            pass


# ==========================================================================
# 3. Radio check / fix / send
# ==========================================================================
BD_NAMES = {3: '9600', 6: '57600', 7: '115200'}


def radio_at_session(port, baud, fix):
    import serial
    s = serial.Serial(port, baud, timeout=1.5)
    try:
        time.sleep(1.1)
        s.write(b'+++')
        time.sleep(1.5)
        if not s.read(3).startswith(b'OK'):
            raise RuntimeError('no reply to +++ (radio already in API mode? wrong baud?)')

        def at(cmd):
            s.reset_input_buffer()
            s.write(f'AT{cmd}\r'.encode())
            return s.read_until(b'\r').decode().strip()

        out = {k: at(k) for k in ('SH', 'SL', 'VR', 'AP', 'BD', 'ID')}
        if fix:
            out['fix'] = f'AP1->{at("AP1")}  WR->{at("WR")}'
            out['AP'] = at('AP')
        at('CN')
        return out
    finally:
        s.close()


def radio_api_session(port, baud, send, log=print):
    dev = XBeeDevice(port, baud)
    dev.open()
    try:
        out = dict(addr=str(dev.get_64bit_addr()), proto=str(dev.get_protocol()),
                   fw=dev.get_firmware_version(),
                   AP=dev.get_parameter('AP').hex(), BD=dev.get_parameter('BD').hex(),
                   ID=dev.get_parameter('ID').hex())
        if send:
            log('broadcasting 20 zero frames (watch the TX LED) ...')
            for i in range(20):
                dev.send_data_broadcast(pack_cmd(i, 0.0, 0.0))
                time.sleep(0.05)
            log('done')
        return out
    finally:
        dev.close()


def radio_show(r, log=print):
    ap_i, bd_i = int(r['AP'], 16), int(r['BD'], 16)
    log(f"64-bit addr : {r['addr']}  <- put this in the OTHER side's address box")
    if r.get('proto'):
        log(f"protocol    : {r['proto']}   firmware: {r['fw']}")
    log(f"AP (API)    : {ap_i:02X}  ({'OK, API mode' if ap_i == 1 else 'NOT API mode - press FIX'})")
    log(f"BD (baud)   : {bd_i:08X}  ({BD_NAMES.get(bd_i, '?')} baud; 7 = 115200)")
    log(f"ID (network): {int(r['ID'], 16):X}  (must match on both radios)")


def radio_check(port, baud, fix=False, send=False, log=print):
    log(f'port        : {port} @ {baud}')
    if fix:
        r = radio_at_session(port, baud, True)
        r['addr'] = r['SH'].zfill(8) + r['SL'].zfill(8)
        radio_show(r, log)
        if r.get('fix'):
            log('fix        : ' + r['fix'])
        log('now unplug / re-plug the radio, then press CHECK again.')
        return
    try:
        r = radio_api_session(port, baud, send, log)
        radio_show(r, log)
    except Exception as e:
        log(f'API mode open failed ({e}); trying AT command mode ...')
        r = radio_at_session(port, baud, False)
        r['addr'] = r['SH'].zfill(8) + r['SL'].zfill(8)
        radio_show(r, log)


# ==========================================================================
# 4. TX core (logic of tx_node.py without ROS)
# ==========================================================================
class TxCore:
    def __init__(self, link, cfg):
        self.link, self.cfg = link, cfg
        self.vx = self.wz = 0.0
        self.enable = False
        self.joy_time = 0.0
        self.axes, self.buttons = [], []
        self.seq = 0
        self.lock = threading.Lock()
        self.ack = None
        self.ack_count = 0
        self._last = (time.monotonic(), 0)

    def on_joy(self, axes, buttons):
        """axes use the ROS convention: stick up / left = positive."""
        g = lambda seq, i: seq[i] if 0 <= i < len(seq) else 0.0
        c = self.cfg
        self.enable = bool(g(buttons, c['enable_button']))
        if self.enable or not c['require_enable']:
            if c['drive_mode'] == 'tank':
                left, right = g(axes, c['axis_left']), g(axes, c['axis_right'])
                self.vx = c['scale_linear'] * (left + right) / 2.0
                self.wz = c['scale_angular'] * (right - left) / 2.0
            else:
                self.vx = c['scale_linear'] * g(axes, c['axis_linear'])
                self.wz = c['scale_angular'] * g(axes, c['axis_angular'])
        else:
            self.vx = self.wz = 0.0
        self.axes, self.buttons = list(axes), list(buttons)
        self.joy_time = time.monotonic()

    def no_input(self):
        self.vx = self.wz = 0.0
        self.enable = False

    def send_tick(self):
        fresh = time.monotonic() - self.joy_time < self.cfg['cmd_timeout']
        vx, wz = (self.vx, self.wz) if fresh else (0.0, 0.0)
        if self.link.send(pack_cmd(self.seq, vx, wz)):
            self.seq = (self.seq + 1) & 0xFF

    def on_radio(self, data):
        m = unpack(data)
        if m and m[0] == 'ack':
            with self.lock:
                self.ack = (time.monotonic(), m[1], m[2])
                self.ack_count += 1

    def status(self):
        now = time.monotonic()
        t0, n0 = self._last
        rate = (self.link.sent - n0) / max(now - t0, 1e-3)
        self._last = (now, self.link.sent)
        with self.lock:
            ack, ack_count = self.ack, self.ack_count
        if not self.link.connected:             # never show a stale ACK once the radio link is gone
            ack = None
        return dict(link=self.link.connected, port=self.link.port, baud=self.link.baud,
                    local=self.link.local_addr, remote=self.link.remote_addr, err=self.link.err,
                    sent=self.link.sent, rate=round(rate, 1), tx_ok=self.link.tx_ok,
                    tx_fail=self.link.tx_fail, why=self.link.last_fail, vx=self.vx, wz=self.wz, enable=self.enable,
                    ack_count=ack_count,
                    ack=None if ack is None else dict(age=now - ack[0], seq=ack[1], lost=ack[2]))


# ==========================================================================
# 5. GUI
# ==========================================================================
import pygame
from PyQt5.QtCore import Qt, QEvent, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtWidgets import (QApplication, QWidget, QGridLayout, QFormLayout, QGroupBox,
                             QHBoxLayout, QLabel, QVBoxLayout, QComboBox, QLineEdit,
                             QPushButton, QCheckBox, QTabWidget, QTextEdit, QMessageBox)

GREEN, RED, GREY, AMBER = '#2ecc71', '#e74c3c', '#7f8c8d', '#f1c40f'
KEYMAP = {Qt.Key_Up: 'up', Qt.Key_Down: 'down', Qt.Key_Left: 'left', Qt.Key_Right: 'right',
          Qt.Key_W: 'up', Qt.Key_S: 'down', Qt.Key_A: 'left', Qt.Key_D: 'right'}


def keyboard_axes(keys, cfg):
    """Held arrow/WASD keys -> axes list in ROS convention (up / left = +), plus a buttons list."""
    fwd = (1.0 if 'up' in keys else 0.0) - (1.0 if 'down' in keys else 0.0)
    turn = (1.0 if 'left' in keys else 0.0) - (1.0 if 'right' in keys else 0.0)
    n = max(cfg['axis_linear'], cfg['axis_angular'], cfg['axis_left'], cfg['axis_right']) + 1
    axes = [0.0] * n
    if cfg['drive_mode'] == 'tank':
        axes[cfg['axis_left']] = max(-1.0, min(1.0, fwd - turn))
        axes[cfg['axis_right']] = max(-1.0, min(1.0, fwd + turn))
    else:
        axes[cfg['axis_linear']] = fwd
        axes[cfg['axis_angular']] = turn
    buttons = [0] * (cfg['enable_button'] + 1)
    if keys:                                    # any drive key held counts as the deadman
        buttons[cfg['enable_button']] = 1
    return axes, buttons


class Bar(QWidget):
    def __init__(self):
        super().__init__()
        self.v = 0.0
        self.setMinimumSize(150, 16)

    def set(self, v):
        self.v = max(-1.0, min(1.0, v))
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor('#2c3e50'))
        p.fillRect(w // 2, 0, int(self.v * w / 2), h, QColor('#3498db'))
        p.fillRect(w // 2 - 1, 0, 2, h, QColor('#ecf0f1'))


class Panel(QGroupBox):
    def __init__(self, title, rows):
        super().__init__(title)
        form = QFormLayout(self)
        self.led, self.head = QLabel(), QLabel()
        self.led.setFixedSize(16, 16)
        form.addRow(self.led, self.head)
        self.v = {}
        for key, name in rows:
            self.v[key] = QLabel('-')
            self.v[key].setTextInteractionFlags(Qt.TextSelectableByMouse)
            form.addRow(name + ':', self.v[key])
        self.state(GREY, '')

    def state(self, color, text):
        self.led.setStyleSheet(f'background:{color}; border-radius:8px; border:1px solid #222;')
        self.head.setText(text)

    def clear(self):
        for lb in self.v.values():
            lb.setText('-')

    def put(self, **kw):
        for k, t in kw.items():
            if k in self.v:
                self.v[k].setText(str(t))


class RadioWorker(QThread):
    line = pyqtSignal(str)

    def __init__(self, port, baud, fix, send):
        super().__init__()
        self.port, self.baud, self.fix, self.send = port, baud, fix, send

    def run(self):
        try:
            radio_check(self.port, self.baud, self.fix, self.send, log=self.line.emit)
        except Exception as e:
            self.line.emit(f'FAILED: {e}')


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('XBee teleop TX - Windows')
        self.cfg = dict(CONFIG)
        self.link = self.tx = None
        self.auto = False                       # True = connected / keep reconnecting
        self.connecting = False
        self.joy, self.joy_ok = None, False
        self.xpad, self.xin_ok = XInputPad(), False
        self.raw_axes, self.buttons = [], []
        self.keys = set()
        self.keyboard_drive = False

        # pygame MUST be up before the poll timer fires
        pygame.init()
        pygame.joystick.init()

        # ---- connection bar ----
        bar = QHBoxLayout()
        bar.addWidget(QLabel('XBee port:'))
        self.port_box = QComboBox()
        self.port_box.setEditable(True)
        self.refresh_ports()
        bar.addWidget(self.port_box)
        bar.addWidget(QLabel(' Baud:'))
        self.baud_box = QComboBox()
        self.baud_box.setEditable(True)
        self.baud_box.addItems(['115200', '57600', '9600'])
        bar.addWidget(self.baud_box)
        bar.addWidget(QLabel(' Robot radio addr:'))
        self.addr_edit = QLineEdit(ROBOT_RADIO_ADDR)
        self.addr_edit.setMaxLength(18)
        self.addr_edit.setFixedWidth(150)
        bar.addWidget(self.addr_edit)
        self.btn_connect = QPushButton('Connect')
        self.btn_connect.clicked.connect(self.toggle_connect)
        bar.addWidget(self.btn_connect)
        self.kb_check = QCheckBox('Keyboard drive (arrows/WASD)')
        self.kb_check.setFocusPolicy(Qt.NoFocus)
        self.kb_check.toggled.connect(self.on_kb)
        bar.addWidget(self.kb_check)
        bar.addStretch(1)

        # ---- panels ----
        self.joy_p = Panel('Joystick', [('pad', 'Device'), ('seen', 'Devices seen by pygame'),
                                        ('deadman', 'Deadman (hold to drive)'),
                                        ('btn', 'Buttons pressed'), ('raw', 'Raw axes (SDL)')])
        self.sticks = {}
        for i in range(4):
            self.sticks[i] = Bar()
            self.joy_p.layout().addRow(f'Axis {i}:', self.sticks[i])
        self.cmd = Panel('Command', [('vx', 'Linear vx (m/s)'), ('wz', 'Angular wz (rad/s)')])
        self.vx_bar, self.wz_bar = Bar(), Bar()
        self.cmd.layout().addRow('vx:', self.vx_bar)
        self.cmd.layout().addRow('wz:', self.wz_bar)
        self.tx_p = Panel('XBee TX (radio level only)',
                          [('port', 'Port'), ('local', 'This radio'), ('remote', 'Sending to'),
                           ('sent', 'Frames sent'), ('rate', 'Rate'),
                           ('ok', 'Robot RADIO acked / failed'), ('why', 'Last radio failure'), ('err', 'Error')])
        self.rx_p = Panel('Robot ACK (rx_node software)',
                          [('n', 'Last ACK seq'), ('cnt', 'ACKs received (total)'), ('age', 'ACK age'),
                           ('lost', 'Frames lost (robot count)'), ('src', 'Last radio source'),
                           ('ign', 'Ignored frames (wrong source)')])
        grid = QGridLayout()
        for i, w in enumerate((self.joy_p, self.cmd, self.tx_p, self.rx_p)):
            grid.addWidget(w, i // 2, i % 2)

        # ---- radio check tab ----
        radio = QWidget()
        rlay = QVBoxLayout(radio)
        rbar = QHBoxLayout()
        rbar.addWidget(QLabel('Port:'))
        self.r_port = QComboBox()
        self.r_port.setEditable(True)
        rbar.addWidget(self.r_port)
        rbar.addWidget(QLabel('Baud:'))
        self.r_baud = QComboBox()
        self.r_baud.setEditable(True)
        self.r_baud.addItems(['115200', '57600', '9600'])
        rbar.addWidget(self.r_baud)
        self.radio_btns = []
        for text, fix, send in (('Check', False, False), ('Fix (set AP=1 + save)', True, False),
                                ('Send 20 test frames', False, True)):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, f=fix, s=send: self.run_radio(f, s))
            rbar.addWidget(b)
            self.radio_btns.append(b)
        rlay.addLayout(rbar)
        self.radio_log = QTextEdit()
        self.radio_log.setReadOnly(True)
        rlay.addWidget(self.radio_log)
        self.radio_worker = None
        for i in range(self.port_box.count()):
            self.r_port.addItem(self.port_box.itemText(i))

        tabs = QTabWidget()
        main_tab = QWidget()
        mlay = QVBoxLayout(main_tab)
        mlay.addLayout(bar)
        mlay.addLayout(grid)
        tabs.addTab(main_tab, 'Teleop')
        tabs.addTab(radio, 'Radio Check')
        QVBoxLayout(self).addWidget(tabs)

        # ---- timers (started last, after pygame + widgets exist) ----
        self.timers = []
        for ms, fn in ((20, self.poll_inputs), (int(1000 / self.cfg['send_rate_hz']), self.tx_tick),
                       (200, self.refresh),
                       (2000, self.reconnect_tick)):
            t = QTimer(self)
            t.timeout.connect(fn)
            t.start(ms)
            self.timers.append(t)

    # ---------------- helpers ----------------
    def refresh_ports(self):
        self.port_box.clear()
        try:
            from serial.tools import list_ports
            for p in sorted(list_ports.comports(), key=lambda p: p.device):
                self.port_box.addItem(p.device)
        except Exception:
            pass
        if self.port_box.count() == 0:
            self.port_box.addItem(self.cfg['port'])

    def on_kb(self, on):
        self.keyboard_drive = on
        self.keys.clear()

    # ---------------- gamepad / keyboard ----------------
    def drop_gamepad(self):
        self.joy, self.joy_ok = None, False
        self.raw_axes, self.buttons = [], []

    def scan_gamepad(self):
        """Hot-plug aware: no quit()/init() cycling. Relies on pygame.event.pump() being called first."""
        # 1) unplugged?
        for ev in pygame.event.get(pygame.JOYDEVICEREMOVED):
            try:
                if self.joy and ev.instance_id == self.joy.get_instance_id():
                    self.drop_gamepad()
            except Exception:
                self.drop_gamepad()
        # 2) plugged in (or already present at start-up)?
        pygame.event.get(pygame.JOYDEVICEADDED)          # drain; get_count() is already up to date
        if not self.joy_ok:
            n = pygame.joystick.get_count()
            if n:
                try:
                    self.joy = pygame.joystick.Joystick(min(self.cfg['joy_index'], n - 1))
                    self.joy.init()                      # harmless on pygame 2, required on pygame 1.9
                    self.joy_ok = True
                except pygame.error:
                    self.drop_gamepad()

    def poll_inputs(self):
        """Read gamepad (XInput first, pygame fallback) / keyboard; feed TxCore when it exists."""
        axes = buttons = raw = None
        self.xin_ok = False
        xs = self.xpad.read(self.cfg['joy_index'])          # direct Windows XInput (independent of pygame)
        if xs is not None:
            self.xin_ok = True
            raw, buttons = xs
        else:
            try:
                pygame.event.pump()
                self.scan_gamepad()
                if self.joy_ok:
                    raw = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
                    buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
            except pygame.error:
                self.drop_gamepad()
                raw = buttons = None
        if raw is not None:
            self.raw_axes, self.buttons = raw, buttons
            dz = self.cfg['joy_deadzone']
            axes = [0.0 if abs(v) < dz else -v for v in raw]      # SDL up/left = - -> flip to ROS convention
        else:
            self.raw_axes, self.buttons = [], []
        if axes is None and self.keyboard_drive:
            axes, buttons = keyboard_axes(self.keys, self.cfg)
            self.raw_axes, self.buttons = [], buttons
        if not self.tx:
            return
        if axes is None:
            self.tx.no_input()                  # joystick data goes stale -> zeros are sent
        else:
            self.tx.on_joy(axes, buttons)

    def eventFilter(self, obj, ev):
        t = ev.type()
        if t == QEvent.ApplicationDeactivate:
            self.keys.clear()
        elif t in (QEvent.KeyPress, QEvent.KeyRelease) and self.keyboard_drive:
            name = KEYMAP.get(ev.key())
            if name is None or isinstance(QApplication.focusWidget(), QLineEdit):
                return False
            if not ev.isAutoRepeat():
                (self.keys.add if t == QEvent.KeyPress else self.keys.discard)(name)
            return True
        return False

    # ---------------- connection ----------------
    def stop_link(self):
        if self.link:
            self.link.close()

    def do_connect(self):
        if not HAVE_XBEE:
            QMessageBox.critical(self, 'digi-xbee missing', 'Run:  pip install digi-xbee')
            return
        self.stop_link()
        try:
            port, baud = self.port_box.currentText().strip(), int(self.baud_box.currentText())
            remote = norm_addr(self.addr_edit.text())
            XBee64BitAddress.from_hex_string(remote)
        except Exception as e:
            QMessageBox.warning(self, 'Bad settings', f'Check baud / 16-hex-digit radio address:\n{e}')
            return
        self.cfg['port'], self.cfg['baud'] = port, baud
        self.link = XBeeLink(port, baud, remote, self.on_radio)
        self.tx = TxCore(self.link, self.cfg)           # fresh TxCore -> no stale ACK carried over
        self.auto = True
        self.start_connect()

    def start_connect(self):
        if self.connecting or not self.link or self.link.connected:
            return
        self.connecting, link = True, self.link

        def work():
            try:
                link.connect()
                if link is not self.link or not self.auto:      # replaced / disconnected meanwhile
                    link.close()
            finally:
                self.connecting = False
        threading.Thread(target=work, daemon=True).start()

    def toggle_connect(self):
        if self.auto:
            self.auto = False
            self.stop_link()
        else:
            self.do_connect()

    def reconnect_tick(self):
        if self.auto and self.link and not self.link.connected:
            self.start_connect()

    def on_radio(self, data):                   # XBee thread: ACKs from the robot
        if self.tx:
            self.tx.on_radio(data)

    # ---------------- tickers ----------------
    def tx_tick(self):
        if self.tx and self.link and self.link.connected:
            self.tx.send_tick()

    # ---------------- radio tab ----------------
    def run_radio(self, fix, send):
        if self.radio_worker and self.radio_worker.isRunning():
            return
        self.radio_log.clear()
        if self.auto or (self.link and self.link.connected):
            self.auto = False
            self.stop_link()
            self.radio_log.append('(teleop link closed to free the port - press Connect again afterwards)')
        for b in self.radio_btns:
            b.setEnabled(False)
        try:
            baud = int(self.r_baud.currentText())
        except ValueError:
            baud = 115200
        w = self.radio_worker = RadioWorker(self.r_port.currentText().strip(), baud, fix, send)
        w.line.connect(self.radio_log.append)
        w.finished.connect(lambda: [b.setEnabled(True) for b in self.radio_btns])
        w.start()

    # ---------------- refresh ----------------
    def refresh(self):
        link = self.link
        if not self.auto:
            self.btn_connect.setText('Connect')
        else:
            self.btn_connect.setText('Disconnect' if link and link.connected else 'Connecting... (click to stop)')
        tx = self.tx.status() if self.tx else None

        # joystick
        try:
            seen = pygame.joystick.get_count()
        except pygame.error:
            seen = 0
        if self.xin_ok or self.joy_ok:
            if self.xin_ok:
                name, how = f"XInput slot {self.cfg['joy_index']}", 'XInput'
            else:
                how = 'pygame'
                try:
                    name = self.joy.get_name()
                except pygame.error:
                    name = '-'
            self.joy_p.state(GREEN, f'gamepad connected ({how})')
            self.joy_p.put(pad=name, seen=seen,
                           btn=', '.join(str(i) for i, b in enumerate(self.buttons) if b) or '-',
                           raw='  '.join(f'{i}:{a:+.2f}' for i, a in enumerate(self.raw_axes)) or '-')
            ax = self.raw_axes + [0.0] * 4
            for i in range(4):
                self.sticks[i].set(ax[i])
        else:
            self.joy_p.state(AMBER if self.keyboard_drive else RED,
                             'keyboard drive' if self.keyboard_drive else 'no gamepad found (plug it in)')
            self.joy_p.put(pad='-', seen=seen, btn='-', raw='-')
            for b in self.sticks.values():
                b.set(0)
        self.joy_p.put(deadman='-' if tx is None else ('HELD' if tx['enable'] else 'released'))

        # command + XBee TX
        if tx is not None:
            moving = abs(tx['vx']) > 1e-3 or abs(tx['wz']) > 1e-3
            self.cmd.state(GREEN if moving else AMBER, 'driving' if moving else 'zero / stopped')
            self.cmd.put(vx=f"{tx['vx']:+.2f}", wz=f"{tx['wz']:+.2f}")
            self.vx_bar.set(tx['vx'])
            self.wz_bar.set(tx['wz'])
            sending = tx['link'] and tx['rate'] > 0.5
            self.tx_p.state(GREEN if sending else (AMBER if tx['link'] else RED),
                            'radio sending (does NOT prove robot software is running)' if sending
                            else ('radio open, idle' if tx['link'] else 'radio not open'))
            self.tx_p.put(port=f"{tx['port']} @ {tx['baud']}", local=tx['local'] or '-', remote=tx['remote'],
                          sent=tx['sent'], rate=f"{tx['rate']:.1f} Hz", ok=f"{tx['tx_ok']} / {tx['tx_fail']}",
                          why=tx['why'] or '-', err=tx['err'] or '-')
        else:
            self.cmd.state(GREY, 'not connected')
            self.cmd.clear()
            self.vx_bar.set(0)
            self.wz_bar.set(0)
            self.tx_p.state(GREY, 'not connected')
            self.tx_p.clear()

        # ACK from the robot's rx_node (the only real proof the robot software is alive)
        ack = tx['ack'] if tx and tx['ack'] and tx['ack']['age'] < 2.0 else None
        if ack is not None:
            self.rx_p.state(GREEN, 'robot rx_node ACK received')
            self.rx_p.put(n=ack['seq'], cnt=tx['ack_count'], age=f"{ack['age'] * 1000:.0f} ms",
                          lost=ack['lost'], src=link.last_src or '-', ign=link.rx_ignored)
        else:
            waiting = tx is not None and tx['link']
            self.rx_p.state(RED if waiting else GREY,
                            'no ACK from robot rx_node' if waiting else 'no data')
            self.rx_p.clear()
            if link:
                self.rx_p.put(src=link.last_src or '-', ign=link.rx_ignored)

    def closeEvent(self, e):
        self.auto = False
        for t in self.timers:
            t.stop()
        if self.radio_worker and self.radio_worker.isRunning():
            self.radio_worker.wait(3000)
        if self.tx and self.link and self.link.connected:
            self.link.send(pack_cmd(self.tx.seq, 0.0, 0.0))
        self.stop_link()
        pygame.quit()
        e.accept()


def _excepthook(et, ev, tb):
    """Never let a callback error kill the app. Works in a --windowed .exe (no console) too:
    errors are appended to xbee_teleop_error.log next to the exe / script."""
    import traceback
    text = ''.join(traceback.format_exception(et, ev, tb))
    try:
        if sys.stderr:
            sys.stderr.write(text)
        base = os.path.dirname(sys.executable if getattr(sys, 'frozen', False) else os.path.abspath(__file__))
        with open(os.path.join(base, 'xbee_teleop_error.log'), 'a') as f:
            f.write(time.strftime('%Y-%m-%d %H:%M:%S\n') + text + '\n')
    except Exception:
        pass


def main():
    sys.excepthook = _excepthook
    if sys.stdout:
        print('XInput available:', XInputPad().fn is not None, '| pygame', pygame.version.ver, flush=True)
    app = QApplication(sys.argv)
    win = MainWindow()
    app.installEventFilter(win)
    win.resize(900, 650)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
