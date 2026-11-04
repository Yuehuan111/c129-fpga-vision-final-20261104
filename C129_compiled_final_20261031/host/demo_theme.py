"""Shared presentation theme and read-only confirmed-mode emphasis."""
from __future__ import annotations

import math
from tkinter import ttk


BACKGROUND = '#eef2f6'
INK = '#24354a'
MUTED = '#61738a'
BORDER = '#d4deea'
ACCENT = '#087f8c'


def apply_demo_theme(window):
    """Only styling: inherited widget geometry and commands are untouched."""
    style = ttk.Style(window)
    style.theme_use('clam')
    window.configure(background=BACKGROUND)
    style.configure('.', font=('Microsoft YaHei UI', 10), background=BACKGROUND,
                    foreground=INK)
    style.configure('TFrame', background=BACKGROUND)
    style.configure('TLabel', background=BACKGROUND, foreground=INK)
    style.configure('TButton', background='#ffffff', foreground=INK,
                    bordercolor=BORDER, lightcolor='#ffffff', darkcolor='#ffffff',
                    borderwidth=1, relief='flat', focusthickness=1, focuscolor=ACCENT)
    style.map('TButton',
        background=[('disabled', '#e7edf4'), ('pressed', '#d4e9ec'), ('active', '#e8f3f5')],
        foreground=[('disabled', '#8899ad'), ('pressed', '#075d68'), ('active', '#075d68')],
        bordercolor=[('disabled', '#dae3ed'), ('focus', ACCENT), ('active', '#82bcc4')],
        lightcolor=[('disabled', '#e7edf4'), ('pressed', '#d4e9ec'), ('active', '#e8f3f5')],
        darkcolor=[('disabled', '#e7edf4'), ('pressed', '#d4e9ec'), ('active', '#e8f3f5')],
        relief=[('pressed', 'flat'), ('active', 'flat')])
    style.configure('TMenubutton', background='#ffffff', foreground=INK, bordercolor=BORDER,
                    lightcolor='#ffffff', darkcolor='#ffffff', borderwidth=1, relief='flat', padding=(10, 7))
    style.map('TMenubutton', background=[('disabled', '#e7edf4'), ('active', '#e8f3f5')],
              foreground=[('disabled', '#8899ad'), ('active', '#075d68')],
              bordercolor=[('focus', ACCENT), ('active', '#82bcc4')])
    for name in ('Primary.TButton', 'Confirmed.Mode.TButton'):
        style.configure(name, background=ACCENT, foreground='#ffffff',
                        bordercolor=ACCENT, lightcolor=ACCENT, darkcolor=ACCENT)
        style.map(name,
            background=[('disabled', '#e7edf4'), ('pressed', '#05616e'), ('active', '#096f7c')],
            foreground=[('disabled', '#8899ad'), ('!disabled', '#ffffff')],
            bordercolor=[('disabled', '#dae3ed'), ('focus', '#053f4a'), ('!disabled', ACCENT)],
            lightcolor=[('disabled', '#e7edf4'), ('pressed', '#05616e'), ('active', '#096f7c'), ('!disabled', ACCENT)],
            darkcolor=[('disabled', '#e7edf4'), ('pressed', '#05616e'), ('active', '#096f7c'), ('!disabled', ACCENT)])
    style.configure('Mode.TButton', padding=(18, 12), font=('Microsoft YaHei UI', 12, 'bold'))
    style.configure('DemoTitle.TLabel', font=('Microsoft YaHei UI', 18, 'bold'), foreground='#17344f')
    style.configure('Heading.TLabel', font=('Microsoft YaHei UI', 16, 'bold'))
    style.configure('Subtle.TLabel', foreground=MUTED)
    style.configure('Status.TLabel', foreground=ACCENT, font=('Microsoft YaHei UI', 12, 'bold'))
    style.configure('Metric.TLabel', foreground='#17344f', font=('Microsoft YaHei UI', 10, 'bold'))
    style.configure('Header.TFrame', background='#13263d')
    style.configure('Header.TLabel', background='#13263d', foreground='#dbe7f4')
    style.configure('Brand.TLabel', background='#13263d', foreground='#75e0cf', font=('Segoe UI', 19, 'bold'))
    style.configure('TNotebook', background=BACKGROUND, borderwidth=0)
    style.configure('TNotebook.Tab', background='#e6edf5', foreground=MUTED,
                    bordercolor=BORDER, lightcolor='#e6edf5', darkcolor='#e6edf5')
    style.map('TNotebook.Tab', background=[('selected', '#ffffff'), ('active', '#f8fbff')],
              foreground=[('selected', ACCENT), ('active', INK)],
              lightcolor=[('selected', '#ffffff')], darkcolor=[('selected', '#ffffff')])
    style.configure('TLabelframe', background=BACKGROUND, bordercolor=BORDER, relief='solid', borderwidth=1)
    style.configure('TLabelframe.Label', foreground='#35516b', background=BACKGROUND,
                    font=('Microsoft YaHei UI', 10, 'bold'))
    for name in ('TEntry', 'TCombobox', 'TSpinbox'):
        style.configure(name, fieldbackground='#ffffff', foreground=INK, bordercolor=BORDER,
                        lightcolor=BORDER, darkcolor=BORDER, selectbackground=ACCENT,
                        selectforeground='#ffffff', arrowcolor=MUTED)
        style.map(name, fieldbackground=[('disabled', '#e7edf4'), ('readonly', '#ffffff')],
                  foreground=[('disabled', '#8899ad'), ('readonly', INK)],
                  bordercolor=[('focus', ACCENT), ('!focus', BORDER)])
    for name in ('TCheckbutton', 'TRadiobutton'):
        style.configure(name, background=BACKGROUND, foreground=INK, focuscolor=ACCENT)
        style.map(name, foreground=[('disabled', '#8899ad'), ('active', '#075d68')],
                  background=[('active', BACKGROUND)], indicatorbackground=[('selected', ACCENT)])
    style.configure('Vertical.TScrollbar', background='#c6d3e1', troughcolor=BACKGROUND,
                    bordercolor=BACKGROUND, arrowcolor='#637b94', relief='flat')
    style.map('Vertical.TScrollbar', background=[('active', '#9db4c9'), ('pressed', '#7c9db8')])
    style.configure('TSeparator', background=BORDER)
    window.option_add('*TCombobox*Listbox.background', '#ffffff')
    window.option_add('*TCombobox*Listbox.foreground', INK)
    window.option_add('*TCombobox*Listbox.selectBackground', ACCENT)
    window.option_add('*TCombobox*Listbox.selectForeground', '#ffffff')


def confirmed_mode(data, target, *, waiting=False):
    """Style actual fresh output only; never alter control or claim a pass."""
    if not data or waiting or not (data.get('connected') or {}).get('fpga'):
        return None
    if data.get('frozen') or data.get('geometry_recovery'):
        return None
    age = (data.get('fpga') or {}).get('status_age_s')
    if not isinstance(age, (int, float)) or isinstance(age, bool) or not math.isfinite(age) or not 0 <= age <= 2.5:
        return None
    board = data.get('latest_pose_status') or data.get('last_status') or {}
    shown = (data.get('scan') or {}).get('displayed') or {}
    if board.get('config_version') is None or shown.get('version') != board['config_version']:
        return None
    if bool(shown.get('raw')) != bool(board.get('raw_active')) or bool(shown.get('observation')) != bool(board.get('observation_active')):
        return None
    if board.get('raw_active'):
        return 'raw'
    if (data.get('visual_scan') or {}).get('enabled'):
        return 'scan'
    if board.get('committed_modes', 0) & 2:
        return None  # A static advanced scan is not the live-scan mode.
    if data.get('continuous'):
        return 'imu'
    if target in ('normal', 'enhance'):
        return target
    return None  # Retained pose and static comparison have their own status text.
