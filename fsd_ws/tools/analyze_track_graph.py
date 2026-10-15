#!/usr/bin/env python3
"""Inspect learned Delaunay corridor topology offline; never publish controls."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evaluation', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.evaluation.read_text())
    cones = [c for c in data['final_map']['cones'] if c['color'] <= 1]
    points = np.array([[c['x'], c['y']] for c in cones])
    ids, mids, links = {}, [], []
    for triangle in Delaunay(points).simplices:
        cross_edges = []
        for a, b in zip(triangle, np.roll(triangle, -1)):
            edge = tuple(sorted((int(a), int(b))))
            if cones[a]['color'] == cones[b]['color']:
                continue
            width = np.linalg.norm(points[a] - points[b])
            if not 1.5 <= width <= 6.0:
                continue
            if edge not in ids:
                ids[edge] = len(mids)
                mids.append((points[a] + points[b]) / 2)
                links.append([])
            cross_edges.append(ids[edge])
        if len(cross_edges) == 2:
            a, b = cross_edges
            links[a].append(b); links[b].append(a)
    components, visited = [], set()
    for start in range(len(mids)):
        if start in visited:
            continue
        stack, component = [start], []
        while stack:
            i = stack.pop()
            if i in visited:
                continue
            visited.add(i); component.append(i)
            stack.extend(links[i])
        components.append(component)
    fig, ax = plt.subplots(figsize=(10, 10))
    for color, name, paint in [(0, 'Blue boundary', '#2082d0'), (1, 'Yellow boundary', '#d29b00')]:
        xy = points[[c['color'] == color for c in cones]]
        ax.scatter(xy[:, 0], xy[:, 1], s=14, color=paint, label=name)
    for i, p in enumerate(mids):
        for j in links[i]:
            if j > i:
                ax.plot([p[0], mids[j][0]], [p[1], mids[j][1]], color='#839080', lw=.8)
    trail = [s['odom'] for s in data['samples'] if 'odom' in s and not s.get('status', {}).get('map_frozen')]
    ax.plot([p['x'] for p in trail], [p['y'] for p in trail], color='#9c493c', lw=1, label='Learning odometry')
    ax.set(xlabel='Estimated odom x (m)', ylabel='Estimated odom y (m)',
           title='Learned cone map and triangle-adjacency corridor graph')
    ax.axis('equal'); ax.grid(alpha=.2); ax.legend()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=140); plt.close(fig)
    endpoints = [mids[i].tolist() for i in range(len(mids)) if len(links[i]) == 1]
    print(json.dumps({'boundary_cones': len(cones), 'midpoints': len(mids), 'endpoints': endpoints,
        'components': sorted([{'nodes': len(c), 'closed': all(len(links[i]) == 2 for i in c)}
                              for c in components], key=lambda c: -c['nodes'])}, indent=2))


if __name__ == '__main__':
    main()
