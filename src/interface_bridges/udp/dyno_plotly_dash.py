#!/usr/bin/env python3
"""
Dyno Plotly Dash Bridge

Receives telemetry JSON from bridge_udp (UDP port 7600) and serves a
live-updating browser dashboard.  Sends commands back on port 7601.

Install dependencies:
    pip install dash plotly

Usage:
    python3 dyno_plotly_dash.py [--telem-port 7600] [--cmd-port 7601] [--web-port 8050]

Then open a browser at: http://localhost:8050
"""

import argparse
import json
import socket
import threading
from collections import deque

import dash
from dash import Input, Output, dcc, html
import plotly.graph_objs as go

TELEM_PORT  = 7600
COMMAND_PORT = 7601
BUFFER_SIZE  = 4096
HISTORY_LEN  = 500
UPDATE_MS    = 100   # dashboard refresh interval


# ── Shared state (written by UDP thread, read by Dash callbacks) ──────────────

_lock    = threading.Lock()
_history = {
    't':          deque(maxlen=HISTORY_LEN),
    'main_vel':   deque(maxlen=HISTORY_LEN),
    'dut_vel':    deque(maxlen=HISTORY_LEN),
    'ch1_torque': deque(maxlen=HISTORY_LEN),
    'ch2_torque': deque(maxlen=HISTORY_LEN),
}
_latest: dict = {}
_t0 = None


def _udp_recv_loop(telem_port: int):
    import time
    global _t0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', telem_port))
    sock.settimeout(1.0)

    while True:
        try:
            data, _ = sock.recvfrom(BUFFER_SIZE)
            t = json.loads(data.decode())
        except (socket.timeout, json.JSONDecodeError):
            continue

        now = time.monotonic()
        if _t0 is None:
            _t0 = now
        elapsed = now - _t0

        with _lock:
            _history['t'].append(elapsed)
            _history['main_vel'].append(t.get('main', {}).get('fb_vel', 0))
            _history['dut_vel'].append(t.get('dut',  {}).get('fb_vel', 0))
            _history['ch1_torque'].append(t.get('ch1_t', 0.0))
            _history['ch2_torque'].append(t.get('ch2_t', 0.0))
            _latest.clear()
            _latest.update(t)


# ── Dash app ──────────────────────────────────────────────────────────────────

app = dash.Dash(__name__)

app.layout = html.Div([
    html.H2('Dyno Testbench Dashboard', style={'fontFamily': 'sans-serif'}),

    # Loop stats bar
    html.Div(id='stats-bar', style={'fontFamily': 'monospace', 'marginBottom': '8px'}),

    # Velocity plot
    dcc.Graph(id='vel-plot', style={'height': '300px'}),

    # Torque plot
    dcc.Graph(id='torque-plot', style={'height': '300px'}),

    # Drive status cards
    html.Div([
        html.Div(id='main-status', style={
            'display': 'inline-block', 'width': '45%',
            'border': '1px solid #ccc', 'padding': '8px',
            'fontFamily': 'monospace', 'marginRight': '2%',
        }),
        html.Div(id='dut-status', style={
            'display': 'inline-block', 'width': '45%',
            'border': '1px solid #ccc', 'padding': '8px',
            'fontFamily': 'monospace',
        }),
    ]),

    dcc.Interval(id='interval', interval=UPDATE_MS, n_intervals=0),
])


@app.callback(
    Output('vel-plot',    'figure'),
    Output('torque-plot', 'figure'),
    Output('stats-bar',   'children'),
    Output('main-status', 'children'),
    Output('dut-status',  'children'),
    Input('interval',     'n_intervals'),
)
def update(_n):
    with _lock:
        ts       = list(_history['t'])
        main_vel = list(_history['main_vel'])
        dut_vel  = list(_history['dut_vel'])
        ch1_t    = list(_history['ch1_torque'])
        ch2_t    = list(_history['ch2_torque'])
        latest   = dict(_latest)

    vel_fig = {
        'data': [
            go.Scatter(x=ts, y=main_vel, mode='lines', name='main_drive', line={'color': 'blue'}),
            go.Scatter(x=ts, y=dut_vel,  mode='lines', name='dut',        line={'color': 'red'}),
        ],
        'layout': go.Layout(
            title='Drive Velocity', xaxis={'title': 'time (s)'},
            yaxis={'title': 'velocity'}, margin={'t': 30},
        ),
    }

    torque_fig = {
        'data': [
            go.Scatter(x=ts, y=ch1_t, mode='lines', name='ch1', line={'color': 'green'}),
            go.Scatter(x=ts, y=ch2_t, mode='lines', name='ch2', line={'color': 'purple'}),
        ],
        'layout': go.Layout(
            title='Torque (ELM3002)', xaxis={'title': 'time (s)'},
            yaxis={'title': 'torque (Nm)'}, margin={'t': 30},
        ),
    }

    stats = (f"cycle={latest.get('cycle','—')}  "
             f"wkc={latest.get('wkc','—')}  "
             f"cycle_us={latest.get('t_us', 0.0):.1f}")

    main = latest.get('main', {})
    main_card = [
        html.B('Main Drive'), html.Br(),
        f"state:  {main.get('state','—')}", html.Br(),
        f"sw:     0x{main.get('sw', 0):04X}", html.Br(),
        f"cmd:    {main.get('cmd_vel', 0)}", html.Br(),
        f"fb:     {main.get('fb_vel',  0)}", html.Br(),
        f"err:    0x{main.get('err', 0):04X}",
    ]

    dut = latest.get('dut', {})
    dut_card = [
        html.B('DUT Drive'), html.Br(),
        f"state:  {dut.get('state','—')}", html.Br(),
        f"sw:     0x{dut.get('sw', 0):04X}", html.Br(),
        f"cmd:    {dut.get('cmd_vel', 0)}", html.Br(),
        f"fb:     {dut.get('fb_vel',  0)}", html.Br(),
        f"err:    0x{dut.get('err', 0):04X}",
    ]

    return vel_fig, torque_fig, stats, main_card, dut_card


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Dyno Plotly Dash dashboard')
    parser.add_argument('--telem-port', type=int, default=TELEM_PORT)
    parser.add_argument('--cmd-port',   type=int, default=COMMAND_PORT)
    parser.add_argument('--web-port',   type=int, default=8050)
    args = parser.parse_args()

    recv_thread = threading.Thread(
        target=_udp_recv_loop, args=(args.telem_port,), daemon=True)
    recv_thread.start()

    print(f'Dashboard at http://localhost:{args.web_port}')
    app.run(host='0.0.0.0', port=args.web_port, debug=False)


if __name__ == '__main__':
    main()
