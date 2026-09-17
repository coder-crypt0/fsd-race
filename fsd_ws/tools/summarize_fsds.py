#!/usr/bin/env python3
"""Summarize a bounded FSDS evaluation; referee data never enters autonomy."""
import argparse
import json
import math
import statistics
from pathlib import Path


def yaw(pose):
    x, y, z, w = pose['q']
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def summarize(data):
    samples = [s for s in data['samples'] if 'odom' in s and 'reference' in s]
    if not samples:
        raise ValueError('No paired estimated/reference odometry samples')
    radii, moving, errors, headings = [], [], [], []
    ref_distance = 0.0
    for i, sample in enumerate(samples):
        est, ref = sample['odom'], sample['reference']
        errors.append(math.hypot(est['x'] - ref['x'], est['y'] - ref['y']))
        dyaw = yaw(est) - yaw(ref)
        headings.append(abs(math.atan2(math.sin(dyaw), math.cos(dyaw))))
        if i:
            prev = samples[i - 1]['reference']
            ref_distance += math.hypot(ref['x'] - prev['x'], ref['y'] - prev['y'])
        if ref['speed'] > 0.5:
            moving.append(ref['speed'])
        wheel = sample.get('wheels', {})
        if wheel and ref['speed'] > 1.0 and abs(wheel['fl_steering_angle']) < 0.04:
            omega = (wheel['fl_rpm'] + wheel['fr_rpm']) * math.pi / 60
            if omega > 1:
                radii.append(ref['speed'] / omega)
    last = samples[-1]
    return {
        'duration_s': round(data['duration_s'], 2),
        'reference_distance_m': round(ref_distance, 2),
        'referee': last.get('referee'),
        'map_cones': last.get('map_cones'),
        'estimated_distance_m': last.get('status', {}).get('lap_distance_m'),
        'max_position_error_m': round(max(errors), 3),
        'final_position_error_m': round(errors[-1], 3),
        'median_heading_error_deg': round(math.degrees(statistics.median(headings)), 3),
        'median_moving_speed_mps': round(statistics.median(moving), 3) if moving else None,
        'max_speed_mps': max(s['reference']['speed'] for s in samples),
        'median_implied_front_radius_m': statistics.median(radii) if radii else None,
        'radius_samples': len(radii),
        'empty_path_fraction': sum(s.get('path_points', 0) == 0 for s in samples) / len(samples),
        'ebs_sample_count': sum(s.get('control', {}).get('emergency_stop', False) for s in samples),
        'supervisor_ebs_samples': sum(s.get('ebs', {}).get('data', False) for s in samples),
        'rates_hz': data.get('rates_hz', {}),
    }


def corridor_metrics(data, reference):
    """Clearance to the ordered reference cone polylines, not to an inferred map."""
    raw = reference['getRefereeState']
    origin = raw['initial_position']
    sides = []
    # FSDS RPC enum differs from ROS: 0=yellow, 1=blue.
    for color, sign in [(0, 1), (1, -1)]:
        points = [((c['x']-origin['x'])*.01, -(c['y']-origin['y'])*.01)
                  for c in raw['cones'] if c['color'] == color]
        segments = [(a, b) for a, b in zip(points, points[1:]+points[:1])]
        sides.append((sign, segments))

    def clearance(x, y, sign, segments):
        best_d2, result = float('inf'), 0.0
        for (ax, ay), (bx, by) in segments:
            dx, dy = bx-ax, by-ay
            length2 = dx*dx+dy*dy
            if length2 < 1e-6:
                continue
            t = max(0, min(1, ((x-ax)*dx+(y-ay)*dy)/length2))
            d2 = (x-ax-t*dx)**2+(y-ay-t*dy)**2
            if d2 < best_d2:
                best_d2 = d2
                result = sign*(dx*(y-ay)-dy*(x-ax))/math.sqrt(length2)
        return result

    centers, bodies = [], []
    for s in data['samples']:
        if 'reference' not in s:
            continue
        p = s['reference']
        x, y, heading = p['x'], p['y'], yaw(p)
        centers.append(min(clearance(x, y, sign, seg) for sign, seg in sides))
        # FSDS documented collision box: 1.8 m long, 1 m wide.
        corners = [(x+f*math.cos(heading)-l*math.sin(heading),
                    y+f*math.sin(heading)+l*math.cos(heading))
                   for f in [-.9,.9] for l in [-.5,.5]]
        bodies.append(min(clearance(cx, cy, sign, seg)
                          for cx, cy in corners for sign, seg in sides))
    return {'minimum_center_boundary_clearance_m': round(min(centers), 3),
            'minimum_body_boundary_clearance_m': round(min(bodies), 3),
            'center_outside_samples': sum(c < 0 for c in centers),
            'body_outside_samples': sum(c < 0 for c in bodies),
            'note': 'Sampled 5 Hz against straight cone-to-cone reference boundaries; not continuous collision proof.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evaluation', type=Path)
    parser.add_argument('--reference', type=Path)
    parser.add_argument('--require-clean-lap', action='store_true',
                        help='Exit nonzero unless a new lap completed without cone hits or EBS')
    args = parser.parse_args()
    data = json.loads(args.evaluation.read_text(encoding='utf-8'))
    summary = summarize(data)
    print(json.dumps(summary, indent=2))
    corridor = None
    if args.reference:
        corridor = corridor_metrics(data, json.loads(args.reference.read_text()))
        print(json.dumps(corridor, indent=2))
    print('\ntime  est_v  ref_v  position_error  heading_error_deg  cones_down_or_out')
    rows = [s for s in data['samples'] if 'odom' in s and 'reference' in s]
    for s in rows[::max(1, len(rows) // 12)]:
        e, r = s['odom'], s['reference']
        dh = yaw(e) - yaw(r)
        print(f"{s['t']:5.1f} {e['speed']:6.2f} {r['speed']:6.2f} "
              f"{math.hypot(e['x']-r['x'], e['y']-r['y']):14.2f} "
              f"{math.degrees(math.atan2(math.sin(dh), math.cos(dh))):18.2f} "
              f"{s.get('referee', {}).get('doo_counter', '?')}")
    if args.require_clean_lap:
        referees = [s['referee'] for s in data['samples'] if 'referee' in s]
        if not referees or len(referees[-1]['laps']) <= len(referees[0]['laps']):
            raise SystemExit('FAIL: no new referee-confirmed lap completed during this evaluation')
        if referees[-1]['doo_counter'] != referees[0]['doo_counter']:
            raise SystemExit('FAIL: referee cone-hit count increased')
        if summary['ebs_sample_count'] or summary['supervisor_ebs_samples']:
            raise SystemExit('FAIL: an emergency stop occurred')
        if data.get('rates_hz', {}).get('ebs', 0) < 1:
            raise SystemExit('FAIL: supervisor emergency-stop telemetry unavailable')
        if corridor and corridor['body_outside_samples']:
            raise SystemExit('FAIL: sampled vehicle body left the reference corridor')
        print('PASS: referee-confirmed clean lap; no cone hits or emergency stop')


if __name__ == '__main__':
    main()
