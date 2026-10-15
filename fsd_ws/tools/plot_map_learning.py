#!/usr/bin/env python3
"""Plot actual mapping/localization transitions and referee lap times."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evaluation', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.evaluation.read_text())
    samples = data['samples']
    t = [s['t'] for s in samples]
    frozen = [s for s in samples if s.get('status', {}).get('map_frozen')]
    race = [s for s in samples if s.get('speed_limit', {}).get('reason') == 1]
    referee = next((s['referee'] for s in reversed(samples) if 'referee' in s), {})
    freeze_time = frozen[0]['t'] if frozen else None
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True, constrained_layout=True)
    axes[0].plot(t, [s.get('map_cones', 0) for s in samples], label='Confirmed landmarks')
    axes[0].plot(t, [s.get('status', {}).get('triangulated_cones', 0) for s in samples], label='Triangulated landmarks (cumulative)')
    axes[0].set_ylabel('Landmark count'); axes[0].legend()
    axes[1].plot(t, [s.get('status', {}).get('matched_cones', 0) for s in samples], label='Unique frame correspondences')
    axes[1].set_ylabel('Matched cones'); axes[1].legend()
    axes[2].plot(t, [s.get('odom', {}).get('speed', float('nan')) for s in samples], label='Encoder / wall-time speed')
    axes[2].plot(t, [s.get('speed_limit', {}).get('v_max_mps', float('nan')) for s in samples], '--', label='Planner speed ceiling')
    axes[2].set_ylabel('Speed (m/s)'); axes[2].legend()
    axes[3].plot(t, [s.get('status', {}).get('localization_rms_m', 0) for s in samples], label='Map-fit RMS (zero also means unavailable)')
    axes[3].plot(t, [int(s.get('ebs', {}).get('data', False)) for s in samples], label='Supervisor EBS flag')
    axes[3].set(ylabel='RMS (m) / EBS flag', xlabel='Elapsed evaluation wall time (s)'); axes[3].legend()
    for ax in axes:
        ax.grid(alpha=.2)
        if freeze_time is not None:
            ax.axvline(freeze_time, color='#a54c32', ls=':', label='Map frozen')
    fig.suptitle('Recorded FSDS learning and map reuse | referee laps: ' +
                 ', '.join(f'{lap:.2f} s' for lap in referee.get('laps', [])) +
                 f" | cone hits: {referee.get('doo_counter', '?')}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=140)
    plt.close(fig)
    summary = {'evaluation_sha256': hashlib.sha256(args.evaluation.read_bytes()).hexdigest(),
               'duration_s': data['duration_s'], 'referee': referee,
               'map_frozen_at_s': freeze_time,
               'race_mode_samples': len(race), 'samples': len(samples),
               'first_race_at_s': race[0]['t'] if race else None,
               'frozen_map_counts': sorted({s.get('map_cones', 0) for s in frozen}),
               'max_triangulated_cones': max(s.get('status', {}).get('triangulated_cones', 0) for s in samples),
               'controller_ebs_samples': sum(bool(s.get('control', {}).get('emergency_stop')) for s in samples),
               'supervisor_ebs_samples': sum(bool(s.get('ebs', {}).get('data')) for s in samples),
               'rates_hz': data.get('rates_hz', {})}
    args.output.with_suffix('.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
