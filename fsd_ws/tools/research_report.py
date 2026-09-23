#!/usr/bin/env python3
"""Regenerate research figures from recorded telemetry, never simulator control."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def distribution(values):
    a = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    return None if not len(a) else {'n': int(len(a)), 'mean': float(np.mean(a)),
                                   'p50': float(np.percentile(a, 50)),
                                   'p95': float(np.percentile(a, 95)), 'max': float(np.max(a))}


def correlation(x, y):
    if len(x) < 5 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def series(samples, field, nested=None):
    return [s.get(field, {}).get(nested, float('nan')) if nested else s.get(field, float('nan'))
            for s in samples]


def style(ax, x, y):
    ax.set(xlabel=x, ylabel=y)
    ax.grid(alpha=.18)
    ax.spines[['top', 'right']].set_visible(False)


def save(fig, out, name):
    fig.savefig(out / (name + '.png'), dpi=140, bbox_inches='tight')
    plt.close(fig)


def render(manifest, out):
    out.mkdir(parents=True, exist_ok=True)
    spec = read(manifest)
    rows = []
    plt.rcParams.update({'font.size': 10, 'axes.titlesize': 12, 'figure.constrained_layout.use': True})
    for config in spec['runs']:
        path = (manifest.parent / config['evaluation']).resolve()
        data = read(path)
        samples = data['samples']
        paired = [s for s in samples if 'odom' in s and 'reference' in s]
        research = data.get('research', {})
        timing = research.get('callback_timing', [])
        last = next((s['referee'] for s in reversed(samples) if 'referee' in s), {})
        errors = [math.hypot(s['odom']['x'] - s['reference']['x'], s['odom']['y'] - s['reference']['y']) for s in paired]
        info = {'label': config['label'], 'evaluation_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'duration_s': data['duration_s'], 'referee': last,
                'position_error_m': distribution(errors), 'rates_hz': data.get('rates_hz', {}),
                'milestones_s': research.get('milestones_s', {}),
                'first_receive_s': research.get('first_receive_s', {}),
                'callbacks': {}, 'arrival_gaps_ms': {}, 'resources': {}}
        for name in sorted({t['component'] for t in timing}):
            selected = [t for t in timing if t['component'] == name]
            info['callbacks'][name] = {key: distribution([t[key] for t in selected])
                                       for key in ['callback_ms', 'input_age_ms']}
        for name, gaps in research.get('arrival_gaps_s', {}).items():
            info['arrival_gaps_ms'][name] = distribution([g[1] * 1000 for g in gaps])
        milestones = info['milestones_s']
        if all(k in milestones for k in ['first_propulsion_command', 'first_reference_motion_0_2_mps']):
            info['observed_command_to_motion_s'] = milestones['first_reference_motion_0_2_mps'] - milestones['first_propulsion_command']
        tag = config['id']
        rows.append((config, data, info))
        if not research:
            continue

        t = series(samples, 't')
        ebs = milestones.get('first_supervisor_ebs')
        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        axes[0].plot(t, series(samples, 'odom', 'speed'), label='Encoder / wall-time speed')
        axes[0].plot(t, series(samples, 'reference', 'speed'), alpha=.6, label='FSDS physics-time speed')
        axes[0].legend(loc='upper right'); style(axes[0], '', 'Speed (m/s)')
        axes[1].plot(t, np.array(series(samples, 'control', 'torque_request')) / 30, label='Torque / 30 Nm')
        axes[1].plot(t, series(samples, 'control', 'brake_cmd'), label='Brake')
        axes[1].plot(t, np.array(series(samples, 'control', 'steering_angle')) / .35, label='Steering / 0.35 rad')
        axes[1].legend(ncol=3); style(axes[1], '', 'Normalized command')
        axes[2].plot(t, series(samples, 'map_cones'), label='Confirmed map entries')
        axes[2].plot(t, series(samples, 'path_points'), label='Path points')
        axes[2].legend(); style(axes[2], '', 'Count (not accuracy)')
        axes[3].plot([s['t'] for s in paired], errors, color='#8d4c85')
        style(axes[3], 'Elapsed evaluation wall time (s)', 'Position error (m)')
        if ebs is not None:
            for ax in axes: ax.axvline(ebs, color='#b34b36', ls='--', label='Supervisor EBS')
        fig.suptitle(config['label'] + ' — observed closed-loop evolution')
        save(fig, out, tag + '-operation')

        names = list(info['callbacks'])
        if names:
            fig, axes = plt.subplots(1, 2, figsize=(12, 5))
            for ax, key, label in zip(axes, ['callback_ms', 'input_age_ms'], ['Sampled callback execution (ms)', 'Latest input age at entry (ms)']):
                for i, name in enumerate(names):
                    d = info['callbacks'][name][key]
                    if not d: continue
                    ax.plot([d['p50'], d['p95']], [i, i], lw=3, color='#247d88')
                    ax.scatter([d['p50'], d['p95'], d['max']], [i]*3, marker='o', s=20)
                ax.set_yticks(range(len(names)), [n.replace('_', ' ') for n in names])
                ax.set_xscale('symlog', linthresh=.1); style(ax, label, '')
            fig.suptitle(config['label'] + ' — p50 to p95 lines; final point = maximum\nInput age is not causal camera-to-command latency')
            save(fig, out, tag + '-timing')

        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        for ax, key in zip(axes, ['camera', 'imu', 'wheels']):
            gaps = research.get('arrival_gaps_s', {}).get(key, [])
            if gaps:
                ax.plot([g[0] for g in gaps], [1000*g[1] for g in gaps], lw=.8)
            ax.set_yscale('log'); style(ax, '', key + ' arrival gap (ms)')
            if key != 'camera': ax.axhline(300, color='#b34b36', ls=':', label='Estimator receive timeout (different consumer)')
            if ebs is not None: ax.axvline(ebs, color='#b34b36', ls='--')
        axes[-1].set_xlabel('Elapsed evaluation wall time (s)')
        fig.suptitle(config['label'] + ' — observer arrival gaps, not sensor generation intervals')
        save(fig, out, tag + '-gaps')

        epoch = data.get('started_unix_s', 0)
        if config.get('windows'):
            win = read((manifest.parent / config['windows']).resolve())
            from datetime import datetime
            wr = win.get('samples', [])
            wt = [datetime.fromisoformat(r['sample_utc'].replace('Z', '+00:00')).timestamp()-epoch for r in wr]
            fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
            for ax, keys, ylabel in [
                (axes[0], ['host_cpu_percent', 'fsds_cpu_percent_host'], 'CPU (% whole host)'),
                (axes[1], ['fsds_3d_engine_percent'], 'FSDS 3D engine (%)'),
                (axes[2], ['host_used_ram_gib'], 'Host used RAM (GiB)')]:
                for key in keys:
                    vals = [float('nan') if r.get(key) is None else r[key] for r in wr]
                    ax.plot(wt, vals, label=key.replace('_', ' '))
                    info['resources'][key] = distribution(vals)
                style(ax, '', ylabel); ax.legend(fontsize=8)
                if ebs is not None: ax.axvline(ebs, color='#b34b36', ls='--')
            axes[-1].set_xlabel('Elapsed evaluation wall time (s); negative = startup')
            fig.suptitle(config['label'] + ' — host resources; dashed line = EBS when present')
            save(fig, out, tag + '-resources')
            # Match nearest one-second GPU sample to camera gaps, bounded time error.
            gaps = research.get('arrival_gaps_s', {}).get('camera', [])
            gx, gy = [], []
            for at, gap in gaps:
                if not wt: continue
                j = int(np.argmin(np.abs(np.array(wt)-at)))
                gpu = wr[j].get('fsds_3d_engine_percent')
                if gpu is not None and abs(wt[j]-at) <= 1.5:
                    gx.append(gpu); gy.append(gap*1000)
            info['gpu_camera_gap_pearson_r'] = correlation(gx, gy)
            info['gpu_camera_gap_pairs'] = len(gx)
            fig, ax = plt.subplots(figsize=(8, 5))
            ax.scatter(gx, gy, alpha=.35, s=12)
            style(ax, 'FSDS GPU 3D engine (%)', 'Observer camera inter-arrival (ms)')
            ax.set_title('Temporal association, not causation\n' + config['label'])
            save(fig, out, tag + '-correlation')

        if config.get('linux'):
            linux = read((manifest.parent / config['linux']).resolve())
            info['linux_processes'] = linux['summary']
            lt = [x-epoch for x in linux.get('sample_unix_s', [])]
            if len(lt) == len(linux['samples']):
                fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
                for name in linux['summary']:
                    for ax, key in zip(axes, ['cpu_percent_one_core', 'rss_mib']):
                        ax.plot(lt, [s.get(name, {}).get(key, float('nan')) for s in linux['samples']], label=name, lw=.8)
                style(axes[0], '', 'CPU (% one core)'); style(axes[1], 'Elapsed evaluation wall time (s)', 'RSS (MiB; shared pages included)')
                axes[0].legend(ncol=2, fontsize=7, loc='upper left')
                fig.suptitle(config['label'] + ' — runtime versus bridge and diagnostic overhead')
                save(fig, out, tag + '-processes')

        if config.get('startup'):
            phases = read((manifest.parent / config['startup']).resolve())
            origin = phases[0]['unix_s']
            points = [(p['event'], p['unix_s']-origin) for p in phases if p['event'] != 'runner_finished']
            if config.get('bridge_startup'):
                for line in (manifest.parent / config['bridge_startup']).read_text().splitlines():
                    p = json.loads(line); points.append((p['event'], p['unix_s']-origin))
            for name, value in info['first_receive_s'].items():
                if name in ['camera','odom','path','control','map']:
                    points.append(('first '+name, epoch+value-origin))
            for name, value in milestones.items():
                if name != 'first_supervisor_ebs': points.append((name, epoch+value-origin))
            points.sort(key=lambda x: x[1]); info['startup_from_launcher_s'] = dict(points)
            fig, ax = plt.subplots(figsize=(11, 7))
            ax.barh([p[0].replace('_',' ') for p in points], [p[1] for p in points], color='#247d88')
            for i, (_, v) in enumerate(points): ax.text(v+.15,i,f'{v:.2f} s',va='center',fontsize=8)
            ax.invert_yaxis(); ax.set_xlim(0,max(p[1] for p in points)*1.2)
            style(ax,'Seconds after PowerShell launcher entered','')
            ax.set_title(config['label'] + ' — cumulative initialization milestones (not additive durations)')
            save(fig,out,tag+'-startup')

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for config, data, info in rows:
        p = [s for s in data['samples'] if 'odom' in s and 'reference' in s]
        e = [math.hypot(s['odom']['x']-s['reference']['x'],s['odom']['y']-s['reference']['y']) for s in p]
        distance = np.cumsum([0]+[math.hypot(b['reference']['x']-a['reference']['x'],b['reference']['y']-a['reference']['y']) for a,b in zip(p,p[1:])])
        axes[0].plot(distance,e,label=config['label'],lw=1)
        axes[1].plot([s['t'] for s in data['samples']],series(data['samples'],'map_cones'),label=config['label'])
    style(axes[0], 'Reference distance travelled (m)', 'Accumulated position error (m)')
    style(axes[1], 'Elapsed wall time (s)', 'Confirmed map entries (not recall)')
    axes[0].legend(fontsize=7); axes[1].axhline(196,color='gray',ls=':',label='Reference cones')
    fig.suptitle('Historical / current recordings — bundled changes, not controlled ablation')
    save(fig,out,'evolution')

    z=np.linspace(1,18,200); f=302.8; h=.8
    sigma=np.hypot(1.5*z*z/(f*h),.03*z)
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    axes[0].plot(z,sigma,label='Configured ground-foot uncertainty heuristic')
    axes[0].axhline(1.5,color='gray',ls=':',label='Local planner sigma gate')
    style(axes[0],'True forward range (m)','Predicted range sigma (m)'); axes[0].legend(fontsize=8)
    for pitch in [.5,1,2]:
        inferred=h/np.tan(np.arctan(h/z)+math.radians(pitch))
        axes[1].plot(z,inferred-z,label=f'{pitch}° unmodelled downward pitch')
    style(axes[1],'True forward range (m)','Analytic ranging bias (m)'); axes[1].legend(fontsize=8)
    fig.suptitle('Analytic sensitivity — not measured detector accuracy')
    save(fig,out,'monocular-sensitivity')
    result={'schema':1,'runs':[r[2] for r in rows], 'notes':[
        'Callback durations are systematic <=5 Hz samples, not all callbacks.',
        'Input age at entry is not causal camera-to-command latency.',
        'Correlations use repeated nearest GPU samples within 1.5 s; autocorrelation invalidates naive significance tests.',
        'Reference data is offline scoring only; CPU/GPU units differ across Windows and Linux.']}
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'runs':len(rows),'figures':len(list(out.glob('*.png')))},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    render(args.manifest,args.output)
