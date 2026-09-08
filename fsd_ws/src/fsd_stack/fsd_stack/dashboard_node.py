"""FSD MISSION CONTROL — dark-theme telemetry dashboard + run platform.

    ros2 run fsd_stack dashboard         then open http://localhost:8321

Pure stdlib HTTP server (no web frameworks): serves an embedded single-page
UI that polls /api/state at 10 Hz. Panels: live track map (cones, planned
path, car + trail), speed gauge, steering/torque/brake, lateral-g dot,
node health grid, topic rates, lap timer, event log, EBS banner — plus the
bag platform: record the current run or replay a stored one back into the
stack, from the browser or the fsd_bag CLI.

Works identically against the Python sim, the C++ stack, FSDS, and the
real car: it only consumes the locked interface topics.
"""

import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from fsd_msgs.msg import (ConeMap, PathPointArray, VehicleCmd, VehicleStatus,
                          Heartbeat)

from .common import qos_reliable, yaw_from_quaternion

RUNS_DIR = os.path.expanduser('~/fsd_runs')

RECORD_TOPICS = [
    '/camera/left/image_raw', '/camera/right/image_raw',
    '/imu/data', '/wheel_speeds', '/perception/cones',
    '/perception/cone_detections', '/odometry/filtered', '/mapping/track',
    '/planning/path', '/control/cmd', '/vehicle/status',
    '/safety/heartbeat', '/safety/ebs_trigger',
]

REPLAY_STAGES = {
    # raw sensors in -> full stack recomputes everything
    'raw': ['/camera/left/image_raw', '/camera/right/image_raw',
            '/imu/data', '/wheel_speeds'],
    # perception output in -> mapping/planning/control recompute
    'cones': ['/perception/cones', '/imu/data', '/wheel_speeds'],
    # everything back verbatim (pure visualization)
    'all': [],
}

AS_NAMES = {0: 'AS OFF', 1: 'AS READY', 2: 'AS DRIVING',
            3: 'AS FINISHED', 4: 'AS EMERGENCY'}


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.declare_parameter('port', 8321)

        self._lock = threading.Lock()
        self._pose = None
        self._speed = 0.0
        self._wz = 0.0
        self._trail = deque(maxlen=600)
        self._cones = []
        self._path = []
        self._cmd = {'steer': 0.0, 'torque': 0.0, 'brake': 0.0, 'estop': False}
        self._ebs = False
        self._as_state = 0
        self._hb = {}
        self._events = deque(maxlen=200)
        self._rates = {}          # topic -> deque of arrival times
        self._lap = {'count': 0, 'last_s': None, 'best_s': None,
                     'start': None, 'dist': 0.0, 't0': None}
        self._last_pose_for_dist = None
        self._recorder = None
        self._replayer = None
        self._record_dir = None

        sub = self.create_subscription
        sub(Odometry, '/odometry/filtered', self._on_odom, qos_reliable(10))
        sub(ConeMap, '/mapping/track', self._on_map, qos_reliable(5))
        sub(PathPointArray, '/planning/path', self._on_path, qos_reliable(5))
        sub(VehicleCmd, '/control/cmd', self._on_cmd, qos_reliable(1))
        sub(Bool, '/safety/ebs_trigger', self._on_ebs, qos_reliable(10))
        sub(VehicleStatus, '/vehicle/status', self._on_status, qos_reliable(10))
        sub(Heartbeat, '/safety/heartbeat', self._on_hb, qos_reliable(10))

        port = int(self.get_parameter('port').value)
        self._server = ThreadingHTTPServer(('0.0.0.0', port),
                                           self._make_handler())
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self._event(f'dashboard up on :{port}')
        self.get_logger().info(f'FSD MISSION CONTROL: http://localhost:{port}')

    # ------------------------------------------------------------ intake
    def _tick_rate(self, topic):
        d = self._rates.setdefault(topic, deque(maxlen=100))
        d.append(time.time())

    def _event(self, text, level='info'):
        self._events.appendleft(
            {'t': time.strftime('%H:%M:%S'), 'text': text, 'level': level})

    def _on_odom(self, m):
        with self._lock:
            self._tick_rate('/odometry/filtered')
            x = m.pose.pose.position.x
            y = m.pose.pose.position.y
            yaw = yaw_from_quaternion(m.pose.pose.orientation)
            self._pose = (x, y, yaw)
            self._speed = m.twist.twist.linear.x
            self._wz = m.twist.twist.angular.z
            if (not self._trail or
                    math.hypot(x - self._trail[-1][0], y - self._trail[-1][1]) > 0.25):
                self._trail.append((round(x, 2), round(y, 2)))
            self._update_lap(x, y)

    def _update_lap(self, x, y):
        lap = self._lap
        if self._speed > 1.0 and lap['start'] is None:
            lap['start'] = (x, y)
            lap['t0'] = time.time()
            lap['dist'] = 0.0
            self._last_pose_for_dist = (x, y)
            return
        if lap['start'] is None:
            return
        if self._last_pose_for_dist is not None:
            lap['dist'] += math.hypot(x - self._last_pose_for_dist[0],
                                      y - self._last_pose_for_dist[1])
        self._last_pose_for_dist = (x, y)
        if (lap['dist'] > 30.0 and
                math.hypot(x - lap['start'][0], y - lap['start'][1]) < 4.0):
            t = time.time() - lap['t0']
            lap['count'] += 1
            lap['last_s'] = t
            lap['best_s'] = t if lap['best_s'] is None else min(lap['best_s'], t)
            lap['t0'] = time.time()
            lap['dist'] = 0.0
            self._event(f"lap {lap['count']} complete: {t:.2f} s", 'ok')

    def _on_map(self, m):
        with self._lock:
            self._tick_rate('/mapping/track')
            self._cones = [[round(c.x, 2), round(c.y, 2), int(c.color)]
                           for c in m.cones]

    def _on_path(self, m):
        with self._lock:
            self._tick_rate('/planning/path')
            self._path = [[round(p.x, 2), round(p.y, 2)] for p in m.points]

    def _on_cmd(self, m):
        with self._lock:
            self._tick_rate('/control/cmd')
            self._cmd = {'steer': m.steering_angle, 'torque': m.torque_request,
                         'brake': m.brake_cmd, 'estop': m.emergency_stop}

    def _on_ebs(self, m):
        with self._lock:
            if m.data and not self._ebs:
                self._event('EBS TRIGGERED', 'danger')
            self._ebs = self._ebs or m.data

    def _on_status(self, m):
        with self._lock:
            self._tick_rate('/vehicle/status')
            if m.as_state != self._as_state:
                self._event(f'AS state -> {AS_NAMES.get(m.as_state, "?")}',
                            'warn' if m.as_state == 4 else 'info')
            self._as_state = m.as_state

    def _on_hb(self, m):
        with self._lock:
            prev = self._hb.get(m.node_id)
            if prev is not None and prev['status'] != m.status:
                lvl = {0: 'ok', 1: 'warn', 2: 'danger'}[min(m.status, 2)]
                self._event(f'{m.node_id}: '
                            f'{["OK", "DEGRADED", "ERROR"][min(m.status, 2)]}'
                            f'{" — " + m.message if m.message else ""}', lvl)
            self._hb[m.node_id] = {'status': int(m.status), 'msg': m.message,
                                   't': time.time()}

    # ---------------------------------------------------------- snapshot
    def snapshot(self):
        with self._lock:
            now = time.time()
            rates = {}
            for topic, d in self._rates.items():
                recent = [t for t in d if now - t < 2.0]
                rates[topic] = round(len(recent) / 2.0, 1)
            hb = {k: {'status': v['status'], 'msg': v['msg'],
                      'age': round(now - v['t'], 2)}
                  for k, v in self._hb.items()}
            lap = self._lap
            return {
                'pose': self._pose, 'speed': round(self._speed, 2),
                'lat_g': round(self._speed * self._wz / 9.81, 3),
                'trail': list(self._trail), 'cones': self._cones,
                'path': self._path, 'cmd': self._cmd, 'ebs': self._ebs,
                'as_state': self._as_state,
                'as_name': AS_NAMES.get(self._as_state, '?'),
                'heartbeats': hb, 'rates': rates,
                'events': list(self._events),
                'lap': {'count': lap['count'], 'last_s': lap['last_s'],
                        'best_s': lap['best_s'],
                        'current_s': (round(now - lap['t0'], 1)
                                      if lap['t0'] else None)},
                'recording': self._record_dir if self._recorder else None,
                'replaying': self._replayer is not None
                             and self._replayer.poll() is None,
            }

    # -------------------------------------------------------- bag control
    def start_record(self, name):
        if self._recorder:
            return {'ok': False, 'error': 'already recording'}
        os.makedirs(RUNS_DIR, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        safe = ''.join(ch for ch in (name or 'run') if ch.isalnum() or ch in '-_')
        out = os.path.join(RUNS_DIR, f'{stamp}_{safe}')
        self._recorder = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', out] + RECORD_TOPICS)
        self._record_dir = out
        self._event(f'recording -> {os.path.basename(out)}', 'ok')
        return {'ok': True, 'dir': out}

    def stop_record(self):
        if not self._recorder:
            return {'ok': False, 'error': 'not recording'}
        self._recorder.send_signal(signal.SIGINT)
        self._recorder.wait(timeout=10)
        self._recorder = None
        self._event(f'recording saved: {os.path.basename(self._record_dir)}', 'ok')
        out = self._record_dir
        self._record_dir = None
        return {'ok': True, 'dir': out}

    def replay(self, run, stage, rate):
        if self._replayer and self._replayer.poll() is None:
            return {'ok': False, 'error': 'replay already running'}
        path = os.path.join(RUNS_DIR, os.path.basename(run))
        if not os.path.isdir(path):
            return {'ok': False, 'error': f'no such run: {run}'}
        args = ['ros2', 'bag', 'play', path, '--rate', str(rate)]
        topics = REPLAY_STAGES.get(stage, [])
        if topics:
            args += ['--topics'] + topics
        self._replayer = subprocess.Popen(args)
        self._event(f'replaying {os.path.basename(path)} (stage={stage}, '
                    f'rate={rate})', 'ok')
        return {'ok': True}

    @staticmethod
    def list_runs():
        if not os.path.isdir(RUNS_DIR):
            return []
        runs = []
        for d in sorted(os.listdir(RUNS_DIR), reverse=True):
            full = os.path.join(RUNS_DIR, d)
            if not os.path.isdir(full):
                continue
            size = sum(os.path.getsize(os.path.join(r, f))
                       for r, _, fs in os.walk(full) for f in fs)
            runs.append({'name': d, 'size_mb': round(size / 1e6, 1)})
        return runs

    # ------------------------------------------------------- HTTP layer
    def _make_handler(dash):

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):  # silence request spam
                pass

            def _json(self, obj, code=200):
                body = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/' or self.path.startswith('/index'):
                    body = DASHBOARD_HTML.encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == '/api/state':
                    self._json(dash.snapshot())
                elif self.path == '/api/runs':
                    self._json(dash.list_runs())
                else:
                    self._json({'error': 'not found'}, 404)

            def do_POST(self):
                n = int(self.headers.get('Content-Length', 0))
                try:
                    body = json.loads(self.rfile.read(n) or b'{}')
                except json.JSONDecodeError:
                    body = {}
                try:
                    if self.path == '/api/record/start':
                        self._json(dash.start_record(body.get('name', 'run')))
                    elif self.path == '/api/record/stop':
                        self._json(dash.stop_record())
                    elif self.path == '/api/replay':
                        self._json(dash.replay(body.get('run', ''),
                                               body.get('stage', 'raw'),
                                               float(body.get('rate', 1.0))))
                    else:
                        self._json({'error': 'not found'}, 404)
                except Exception as e:  # subprocess errors -> visible in UI
                    self._json({'ok': False, 'error': str(e)}, 500)

        return Handler

    def shutdown(self):
        self._server.shutdown()
        if self._recorder:
            self._recorder.send_signal(signal.SIGINT)


DASHBOARD_HTML = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>FSD MISSION CONTROL</title>
<style>
:root{--bg:#0a0e14;--panel:#10151f;--edge:#1c2433;--fg:#c9d4e3;--dim:#5c6b82;
--cyan:#00e5ff;--ok:#2dd36f;--warn:#ffb020;--danger:#ff2d55;--blue:#3b82f6;--yellow:#eab308;}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--fg);font:13px/1.45 "JetBrains Mono","Cascadia Code",Consolas,monospace;height:100vh;display:flex;flex-direction:column;overflow:hidden}
header{display:flex;align-items:center;gap:16px;padding:8px 14px;background:var(--panel);border-bottom:1px solid var(--edge)}
header h1{font-size:14px;letter-spacing:3px;color:var(--cyan)}
.pill{padding:2px 12px;border-radius:3px;font-weight:bold;letter-spacing:1px;font-size:12px}
.pill.ok{background:#0c2b1a;color:var(--ok);border:1px solid var(--ok)}
.pill.warn{background:#2b230c;color:var(--warn);border:1px solid var(--warn)}
.pill.danger{background:#2b0c14;color:var(--danger);border:1px solid var(--danger);animation:blink .5s infinite alternate}
@keyframes blink{to{opacity:.35}}
#clock{margin-left:auto;color:var(--dim)}
main{flex:1;display:grid;grid-template-columns:1fr 340px;grid-template-rows:1fr 210px;gap:8px;padding:8px;min-height:0}
.panel{background:var(--panel);border:1px solid var(--edge);border-radius:4px;padding:8px;overflow:hidden;display:flex;flex-direction:column}
.panel h2{font-size:10px;letter-spacing:2px;color:var(--dim);margin-bottom:6px;text-transform:uppercase}
#mapwrap{grid-row:1/3}
#map{flex:1;width:100%;height:100%}
#side{display:flex;flex-direction:column;gap:8px;min-height:0}
#gauges{display:grid;grid-template-columns:1fr 1fr;gap:8px}
canvas.g{width:100%;height:110px}
.bar{height:14px;background:#0d1320;border:1px solid var(--edge);border-radius:2px;position:relative;margin:3px 0 8px}
.bar>div{position:absolute;top:0;bottom:0;border-radius:2px}
.lbl{display:flex;justify-content:space-between;color:var(--dim);font-size:11px}
.big{font-size:26px;color:var(--cyan);font-weight:bold}
#bottom{grid-column:1/3;display:grid;grid-template-columns:1.1fr .9fr 1.2fr 1fr;gap:8px;min-height:0}
.tiles{display:flex;flex-wrap:wrap;gap:6px;align-content:flex-start}
.tile{padding:5px 8px;border-radius:3px;font-size:11px;border:1px solid var(--edge);background:#0d1320}
.tile.s0{border-color:var(--ok);color:var(--ok)}
.tile.s1{border-color:var(--warn);color:var(--warn)}
.tile.s2{border-color:var(--danger);color:var(--danger)}
.tile.stale{border-color:var(--danger);color:var(--danger);background:#1b0d12}
#log{overflow-y:auto;flex:1;font-size:11px}
#log div{padding:1px 0;border-bottom:1px solid #131b2a}
#log .ok{color:var(--ok)} #log .warn{color:var(--warn)} #log .danger{color:var(--danger)}
#log .t{color:var(--dim);margin-right:6px}
table{width:100%;border-collapse:collapse;font-size:11px}
td{padding:2px 4px;border-bottom:1px solid #131b2a}
td.num{text-align:right;color:var(--cyan)}
button{background:#0d1928;color:var(--cyan);border:1px solid var(--cyan);border-radius:3px;padding:4px 10px;font:inherit;font-size:11px;cursor:pointer}
button:hover{background:#12283f}
button.rec{color:var(--danger);border-color:var(--danger)}
select,input{background:#0d1320;color:var(--fg);border:1px solid var(--edge);border-radius:3px;padding:3px 6px;font:inherit;font-size:11px}
#runs{overflow-y:auto;flex:1}
.runrow{display:flex;gap:6px;align-items:center;padding:3px 0;border-bottom:1px solid #131b2a;font-size:11px}
.runrow .nm{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.runrow .sz{color:var(--dim)}
#lapbox{display:flex;gap:14px;align-items:baseline;margin-top:4px}
.dim{color:var(--dim)}
</style></head><body>
<header>
  <h1>FSD MISSION CONTROL</h1>
  <span id="as" class="pill ok">AS OFF</span>
  <span id="ebs" class="pill ok">EBS ARMED</span>
  <span id="recpill" class="pill ok" style="display:none">REC</span>
  <span id="clock"></span>
</header>
<main>
  <div class="panel" id="mapwrap"><h2>Track — live map</h2><canvas id="map"></canvas></div>
  <div id="side">
    <div class="panel"><h2>Vehicle</h2>
      <div id="gauges">
        <canvas class="g" id="speedg"></canvas>
        <canvas class="g" id="gdot"></canvas>
      </div>
      <div class="lbl"><span>STEER</span><span id="steerv">0.00 rad</span></div>
      <div class="bar"><div id="steerbar" style="background:var(--cyan)"></div></div>
      <div class="lbl"><span>TORQUE</span><span id="tqv">0.0 Nm</span></div>
      <div class="bar"><div id="tqbar" style="background:var(--ok);left:0"></div></div>
      <div class="lbl"><span>BRAKE</span><span id="brkv">0%</span></div>
      <div class="bar"><div id="brkbar" style="background:var(--danger);left:0"></div></div>
      <div id="lapbox">
        <span class="dim">LAP <span class="big" id="lapn">0</span></span>
        <span>cur <span id="lapc" style="color:var(--cyan)">–</span></span>
        <span>last <span id="lapl" style="color:var(--fg)">–</span></span>
        <span>best <span id="lapb" style="color:var(--ok)">–</span></span>
      </div>
    </div>
    <div class="panel" style="flex:1"><h2>Run platform</h2>
      <div style="display:flex;gap:6px;margin-bottom:6px">
        <input id="runname" placeholder="run name" style="flex:1">
        <button id="recbtn" class="rec" onclick="toggleRec()">● REC</button>
      </div>
      <div style="display:flex;gap:6px;margin-bottom:6px">
        <select id="stage"><option value="raw">replay: raw sensors</option>
          <option value="cones">replay: cones</option>
          <option value="all">replay: everything</option></select>
        <select id="rate"><option>0.5</option><option selected>1.0</option><option>2.0</option></select>
      </div>
      <div id="runs"></div>
    </div>
  </div>
  <div id="bottom">
    <div class="panel"><h2>Node health</h2><div class="tiles" id="nodes"></div></div>
    <div class="panel"><h2>Topic rates</h2><table id="ratest"></table></div>
    <div class="panel"><h2>Event log</h2><div id="log"></div></div>
    <div class="panel"><h2>Telemetry</h2><table id="telet"></table></div>
  </div>
</main>
<script>
"use strict";
let S=null;
const $=id=>document.getElementById(id);

function fmt(v,d=2){return v==null?'–':Number(v).toFixed(d)}

async function poll(){
  try{
    S=await (await fetch('/api/state')).json();
    render();
  }catch(e){/* stack down; keep last frame */}
}
setInterval(poll,100); poll();
async function loadRuns(){
  try{
    const runs=await (await fetch('/api/runs')).json();
    $('runs').innerHTML=runs.map(r=>
      `<div class="runrow"><span class="nm">${r.name}</span>`+
      `<span class="sz">${r.size_mb} MB</span>`+
      `<button onclick="replay('${r.name}')">▶</button></div>`).join('')||'<span style="color:var(--dim)">no stored runs</span>';
  }catch(e){}
}
setInterval(loadRuns,5000); loadRuns();

async function toggleRec(){
  if(S&&S.recording){await fetch('/api/record/stop',{method:'POST'});}
  else{await fetch('/api/record/start',{method:'POST',
    body:JSON.stringify({name:$('runname').value||'run'})});}
  loadRuns();
}
async function replay(run){
  await fetch('/api/replay',{method:'POST',body:JSON.stringify(
    {run:run,stage:$('stage').value,rate:parseFloat($('rate').value)})});
}

function render(){
  $('clock').textContent=new Date().toLocaleTimeString();
  const as=$('as'); as.textContent=S.as_name;
  as.className='pill '+(S.as_state===4?'danger':S.as_state===2?'ok':'warn');
  const ebs=$('ebs');
  ebs.textContent=S.ebs?'EBS FIRED':'EBS ARMED';
  ebs.className='pill '+(S.ebs?'danger':'ok');
  $('recpill').style.display=S.recording?'inline':'none';
  const rb=$('recbtn'); rb.textContent=S.recording?'■ STOP':'● REC';

  // control bars
  const st=S.cmd.steer, sm=0.35;
  const sb=$('steerbar');
  if(st>=0){sb.style.left='50%';sb.style.width=(st/sm*50)+'%';}
  else{sb.style.width=(-st/sm*50)+'%';sb.style.left=(50+st/sm*50)+'%';}
  $('steerv').textContent=fmt(st)+' rad';
  $('tqbar').style.width=Math.min(S.cmd.torque/30*100,100)+'%';
  $('tqv').textContent=fmt(S.cmd.torque,1)+' Nm';
  $('brkbar').style.width=(S.cmd.brake*100)+'%';
  $('brkv').textContent=Math.round(S.cmd.brake*100)+'%';

  // laps
  $('lapn').textContent=S.lap.count;
  $('lapc').textContent=S.lap.current_s!=null?fmt(S.lap.current_s,1)+'s':'–';
  $('lapl').textContent=S.lap.last_s!=null?fmt(S.lap.last_s,2)+'s':'–';
  $('lapb').textContent=S.lap.best_s!=null?fmt(S.lap.best_s,2)+'s':'–';

  // nodes
  $('nodes').innerHTML=Object.entries(S.heartbeats).map(([id,h])=>{
    const stale=h.age>0.5;
    return `<div class="tile ${stale?'stale':'s'+h.status}" title="${h.msg||''}">`+
      `${id}<br><span style="font-size:9px">${stale?'STALE '+fmt(h.age,1)+'s':['OK','DEGRADED','ERROR'][h.status]}</span></div>`;
  }).join('');

  // rates
  $('ratest').innerHTML=Object.entries(S.rates).sort()
    .map(([t,hz])=>`<tr><td>${t}</td><td class="num">${hz} Hz</td></tr>`).join('');

  // telemetry
  const p=S.pose||[0,0,0];
  $('telet').innerHTML=[
    ['speed', fmt(S.speed)+' m/s'],
    ['lateral g', fmt(S.lat_g)+' g'],
    ['pose x', fmt(p[0])+' m'], ['pose y', fmt(p[1])+' m'],
    ['yaw', fmt(p[2])+' rad'],
    ['cones mapped', S.cones.length],
    ['path points', S.path.length],
    ['replaying', S.replaying?'YES':'no'],
  ].map(([k,v])=>`<tr><td>${k}</td><td class="num">${v}</td></tr>`).join('');

  // log
  $('log').innerHTML=S.events.map(e=>
    `<div class="${e.level}"><span class="t">${e.t}</span>${e.text}</div>`).join('');

  drawMap(); drawSpeed(); drawG();
}

function fitCanvas(c){
  const r=c.getBoundingClientRect();
  if(c.width!==r.width*devicePixelRatio){c.width=r.width*devicePixelRatio;c.height=r.height*devicePixelRatio;}
  const ctx=c.getContext('2d');
  ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);
  return [ctx,r.width,r.height];
}

function drawMap(){
  const [ctx,W,H]=fitCanvas($('map'));
  ctx.clearRect(0,0,W,H);
  const pts=[...S.cones.map(c=>[c[0],c[1]]),...S.trail];
  if(S.pose)pts.push([S.pose[0],S.pose[1]]);
  if(!pts.length)return;
  let x0=1e9,x1=-1e9,y0=1e9,y1=-1e9;
  for(const[x,y]of pts){x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,y);y1=Math.max(y1,y);}
  const pad=4, sc=Math.min(W/(x1-x0+2*pad),H/(y1-y0+2*pad));
  const tx=x=> (x-(x0+x1)/2)*sc+W/2;
  const ty=y=> H/2-(y-(y0+y1)/2)*sc;   // y up

  // grid every 5 m
  ctx.strokeStyle='#131b2a';ctx.lineWidth=1;
  for(let gx=Math.floor(x0/5)*5;gx<=x1;gx+=5){ctx.beginPath();ctx.moveTo(tx(gx),0);ctx.lineTo(tx(gx),H);ctx.stroke();}
  for(let gy=Math.floor(y0/5)*5;gy<=y1;gy+=5){ctx.beginPath();ctx.moveTo(0,ty(gy));ctx.lineTo(W,ty(gy));ctx.stroke();}

  // trail
  if(S.trail.length>1){
    ctx.strokeStyle='rgba(0,229,255,.25)';ctx.lineWidth=2;ctx.beginPath();
    S.trail.forEach(([x,y],i)=>i?ctx.lineTo(tx(x),ty(y)):ctx.moveTo(tx(x),ty(y)));
    ctx.stroke();
  }
  // planned path
  if(S.path.length>1){
    ctx.strokeStyle='#00e5ff';ctx.lineWidth=2;ctx.setLineDash([6,4]);ctx.beginPath();
    S.path.forEach(([x,y],i)=>i?ctx.lineTo(tx(x),ty(y)):ctx.moveTo(tx(x),ty(y)));
    ctx.stroke();ctx.setLineDash([]);
  }
  // cones
  const cols={0:'#3b82f6',1:'#eab308',2:'#fb923c',3:'#fb923c'};
  for(const[x,y,c]of S.cones){
    ctx.fillStyle=cols[c]||'#888';
    ctx.beginPath();ctx.arc(tx(x),ty(y),Math.max(2.5,sc*.18),0,7);ctx.fill();
  }
  // car
  if(S.pose){
    const[x,y,yaw]=S.pose,px=tx(x),py=ty(y),L=Math.max(8,sc*.9);
    ctx.save();ctx.translate(px,py);ctx.rotate(-yaw);
    ctx.fillStyle='#ff2d55';
    ctx.beginPath();ctx.moveTo(L,0);ctx.lineTo(-L*.5,L*.4);ctx.lineTo(-L*.5,-L*.4);ctx.closePath();ctx.fill();
    ctx.restore();
  }
}

function drawSpeed(){
  const [ctx,W,H]=fitCanvas($('speedg'));
  ctx.clearRect(0,0,W,H);
  const cx=W/2,cy=H*0.78,R=Math.min(W/2,H*0.7)-6,vmax=12;
  ctx.lineWidth=8;ctx.lineCap='round';
  ctx.strokeStyle='#131b2a';
  ctx.beginPath();ctx.arc(cx,cy,R,Math.PI,2*Math.PI);ctx.stroke();
  const f=Math.min(S.speed/vmax,1);
  ctx.strokeStyle=f>0.8?'#ffb020':'#00e5ff';
  ctx.beginPath();ctx.arc(cx,cy,R,Math.PI,Math.PI*(1+f));ctx.stroke();
  ctx.fillStyle='#c9d4e3';ctx.font='bold 20px monospace';ctx.textAlign='center';
  ctx.fillText(fmt(S.speed,1),cx,cy-6);
  ctx.font='9px monospace';ctx.fillStyle='#5c6b82';
  ctx.fillText('m/s',cx,cy+8);
}

function drawG(){
  const [ctx,W,H]=fitCanvas($('gdot'));
  ctx.clearRect(0,0,W,H);
  const cx=W/2,cy=H/2,R=Math.min(W,H)/2-8;
  ctx.strokeStyle='#131b2a';
  [0.5,1.0].forEach(g=>{ctx.beginPath();ctx.arc(cx,cy,R*g,0,7);ctx.stroke();});
  ctx.beginPath();ctx.moveTo(cx-R,cy);ctx.lineTo(cx+R,cy);ctx.moveTo(cx,cy-R);ctx.lineTo(cx,cy+R);ctx.stroke();
  const gx=Math.max(-1,Math.min(1,S.lat_g));
  ctx.fillStyle='#ff2d55';
  ctx.beginPath();ctx.arc(cx+gx*R,cy,4,0,7);ctx.fill();
  ctx.fillStyle='#5c6b82';ctx.font='9px monospace';ctx.textAlign='center';
  ctx.fillText('lat g',cx,H-2);
}
</script></body></html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
