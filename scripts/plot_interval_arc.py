"""Plot measured wrist detours and the automatically sized virtual tube."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from multipath.path_exclusion import compare_path, transport_points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--exclusions', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.exclusions.read_text())
    if len(config['segments']) != 1:
        raise ValueError('Select an exclusion file with one motion interval')
    segment = config['segments'][0]
    start, end = segment['window']
    paths = []
    for directory in (args.baseline, args.candidate):
        trace = json.loads((directory / 'actual_trace.json').read_text())
        paths.append(transport_points(trace, start, end))
    baseline, candidate = paths
    metrics = compare_path(candidate, segment)
    source = segment['sources'][0]
    profile = source['placement_diagnostics']
    fractions = np.asarray(profile['arc_fractions'])
    half_sizes = np.asarray(profile['half_sizes_m'])

    fig, (ax, size_ax) = plt.subplots(1, 2, figsize=(14, 5.5),
                                      gridspec_kw={'width_ratios': [1.3, 1]})
    for box in segment['cuboid'].values():
        center = np.asarray(box['pose'][:3])
        rotation = Rotation.from_quat(np.asarray(box['pose'][3:])[[1, 2, 3, 0]])
        half = np.asarray(box['dims']) / 2
        corners = np.array(np.meshgrid(*[[-v, v] for v in half])).reshape(3, -1).T
        projected = rotation.apply(corners)[:, :2] + center[:2]
        hull = ConvexHull(projected)
        ax.add_patch(Polygon(projected[hull.vertices] * 100, closed=True,
                             facecolor='#cc4c90', edgecolor='none', alpha=.06))
    ax.plot(baseline[:, 0] * 100, baseline[:, 1] * 100, color='#00a6c8',
            lw=2.5, label='Measured baseline wrist')
    ax.plot(candidate[:, 0] * 100, candidate[:, 1] * 100, color='#e83c36',
            lw=3, label='Measured detour wrist')
    ax.scatter(*((baseline[0, :2] * 100).tolist()), color='#208440', s=65, zorder=5)
    ax.scatter(*((baseline[-1, :2] * 100).tolist()), color='#d87711', s=65, zorder=5)
    ax.set(aspect='equal', xlabel='World X (cm)', ylabel='World Y (cm)',
           title=f'Measured PhysX wrist XY, source frames {start}–{end - 1}')
    ax.legend(loc='best', fontsize=9)
    ax.grid(alpha=.25)
    comparison = metrics['comparisons'][0]
    fig.text(.02, .012,
             f"Measured max separation {comparison['max_nearest_path_distance_m'] * 100:.1f} cm  |  "
             f"middle median {comparison['middle_median_path_distance_m'] * 100:.1f} cm  |  "
             f"required arc {metrics['minimum_arc_distance_m'] * 100:.1f} cm",
             fontsize=10)

    size_ax.plot(fractions, half_sizes * 100, color='#bf3878', lw=3)
    size_ax.fill_between(fractions, half_sizes * 100, color='#cc4c90', alpha=.18)
    size_ax.set(xlabel='Normalized distance along baseline interval',
                ylabel='Virtual obstacle transverse half-width (cm)',
                title='Automatically sized: small → large → small')
    size_ax.grid(alpha=.25)
    size_ax.text(.02, .98,
                 f"Peak {max(half_sizes) * 100:.1f} cm\n"
                 f"Measured clearance {metrics['actual_wrist_min_exclusion_clearance_m'] * 1000:.1f} mm",
                 transform=size_ax.transAxes, fontsize=10, va='top',
                 bbox={'facecolor': 'white', 'alpha': .9, 'edgecolor': '.8'})
    report=json.loads((args.candidate/'pipeline_result.json').read_text())
    status='accepted' if report.get('diverse_solution_success') else 'diagnostic, task not accepted'
    fig.suptitle(f'Measured interval arc: {status}', fontsize=14)
    fig.tight_layout(rect=[0, .055, 1, .94])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    print(json.dumps({'plot': str(args.output.resolve()), 'metrics': metrics}))


if __name__ == '__main__':
    main()
