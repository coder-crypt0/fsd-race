#!/usr/bin/env python3
"""Plot recorded FSDS reference/estimated trajectories; no simulator writes."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evaluation', type=Path)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.evaluation.read_text())
    raw = json.loads(args.reference.read_text())['getRefereeState']
    origin = raw['initial_position']
    samples = [s for s in data['samples'] if 'reference' in s and 'odom' in s]
    fig, (track, speed) = plt.subplots(1, 2, figsize=(12, 6), gridspec_kw={'width_ratios':[1,1.35]})
    for color, shade, label in [(1,'#2585d6','Blue boundary'),(0,'#d99d00','Yellow boundary')]:
        cones = [c for c in raw['cones'] if c['color'] == color]
        track.scatter([(c['x']-origin['x'])*.01 for c in cones],
                      [-(c['y']-origin['y'])*.01 for c in cones], s=9, c=shade, label=label)
    for key, shade, label, style in [('reference','#13835d','FSDS vehicle','-'),
                                    ('odom','#9d547d','Estimated position','--')]:
        track.plot([s[key]['x'] for s in samples],[s[key]['y'] for s in samples],
                   style, lw=1.2, c=shade, label=label)
    track.set(aspect='equal', xlabel='Startup-frame x (m)', ylabel='Startup-frame y (m)', title='Recorded trajectory and reference cones')
    track.legend(fontsize=8, loc='best')
    for key, shade, label in [('reference','#13835d','FSDS physics-time speed'),
                              ('odom','#9d547d','Encoder / wall-time speed')]:
        speed.plot([s['t'] for s in samples], [s[key]['speed'] for s in samples],
                   color=shade, lw=.8, alpha=.8, label=label)
    speed.set(xlabel='Wall time (s)', ylabel='Speed (m/s)', title='Different velocity time bases under simulator load')
    speed.legend(fontsize=8, loc='upper right')
    referee = samples[-1].get('referee', {})
    laps = ', '.join(f'{t:.2f} s' for t in referee.get('laps', [])) or 'none'
    fig.suptitle(f"FSDS TrainingMap | completed laps: {laps} | cone hits: {referee.get('doo_counter', '?')}", fontsize=13)
    for ax in (track, speed):
        ax.grid(alpha=.18)
        ax.spines[['top','right']].set_visible(False)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
