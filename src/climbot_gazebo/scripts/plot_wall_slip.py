#!/usr/bin/env python3
# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Plot the recorded wall-slip trajectory so the drift can be judged by eye."""

import argparse
from collections import defaultdict

from climbot_gazebo.trajectory_io import read_trajectory as read_rows

import matplotlib
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SOFT = '#52514e'
GRID = '#dedcd6'
# First three slots of the reference categorical palette, in fixed order.
SERIES = ('#2a78d6', '#eb6834', '#1baf7a')

ROBOT_LENGTH_M = 0.52
ROBOT_WIDTH_M = 0.44


def read_trajectory(path):
    """Group the recorded samples by phase label."""
    phases = defaultdict(list)
    for row in read_rows(path):
        phases[row['phase']].append((
            float(row['time_s']), float(row['forward_m']),
            float(row['up_m']), float(row['yaw_deg'])))
    return phases


def style_axes(axes, title, xlabel, ylabel):
    """Apply the recessive grid and ink treatment shared by every panel."""
    axes.set_facecolor(SURFACE)
    axes.set_title(title, color=INK, fontsize=11, pad=10, loc='left')
    axes.set_xlabel(xlabel, color=INK_SOFT, fontsize=9)
    axes.set_ylabel(ylabel, color=INK_SOFT, fontsize=9)
    axes.grid(True, color=GRID, linewidth=0.6)
    axes.set_axisbelow(True)
    axes.tick_params(colors=INK_SOFT, labelsize=8)
    for spine in ('top', 'right'):
        axes.spines[spine].set_visible(False)
    for spine in ('left', 'bottom'):
        axes.spines[spine].set_color(GRID)


def plot_wall_plane(axes, phases):
    """Draw the true path at equal aspect, with the robot drawn to scale."""
    runs = sorted(name for name in phases if name.startswith('horizontal_'))
    for index, name in enumerate(runs):
        samples = phases[name]
        forward = [row[1] for row in samples]
        up = [row[2] for row in samples]
        axes.plot(forward, up, color=SERIES[index % len(SERIES)], linewidth=2.0,
                  label='run %s' % name.split('_')[-1], zorder=3)
    if runs:
        samples = phases[runs[0]]
        for position, edge in ((samples[0], INK_SOFT), (samples[-1], INK_SOFT)):
            axes.add_patch(Rectangle(
                (position[1] - ROBOT_LENGTH_M / 2.0,
                 position[2] - ROBOT_WIDTH_M / 2.0),
                ROBOT_LENGTH_M, ROBOT_WIDTH_M, facecolor='none',
                edgecolor=edge, linewidth=1.2, linestyle='--', zorder=2))
        drop = samples[0][2] - samples[-1][2]
        axes.annotate(
            '%.0f mm drop over %.2f m' % (drop * 1000.0, samples[-1][1]),
            xy=(samples[-1][1], samples[-1][2]),
            xytext=(samples[-1][1] - 0.62, samples[-1][2] + 0.20),
            color=INK, fontsize=10,
            arrowprops={'arrowstyle': '->', 'color': INK_SOFT, 'linewidth': 1.0})
    # Equal aspect is the whole point: any other scaling would exaggerate or
    # hide the slope. 'box' shrinks the frame instead of padding the data.
    axes.set_aspect('equal', adjustable='box')
    style_axes(
        axes,
        'True path in the wall plane, equal aspect (dashed: robot footprint)',
        'along-wall horizontal travel (m)', 'along-wall vertical travel (m)')
    # Laid out above the frame: the plot area is occupied by the footprint
    # outlines and the annotation, with no corner free for a stacked legend.
    axes.legend(frameon=False, fontsize=9, labelcolor=INK_SOFT, ncol=3,
                loc='lower right', bbox_to_anchor=(1.0, 1.0))


def plot_descent_ratio(axes, phases):
    """Show descent against forward travel, where the ratio is the slope."""
    runs = sorted(name for name in phases if name.startswith('horizontal_'))
    for index, name in enumerate(runs):
        samples = phases[name]
        forward = [row[1] for row in samples]
        descent = [-row[2] * 1000.0 for row in samples]
        axes.plot(forward, descent, color=SERIES[index % len(SERIES)],
                  linewidth=2.0, label='run %s' % name.split('_')[-1])
    style_axes(axes, 'Descent against forward travel',
               'along-wall horizontal travel (m)', 'descent (mm)')
    axes.legend(frameon=False, fontsize=9, labelcolor=INK_SOFT, loc='upper left')


def plot_vertical(axes, phases):
    """Compare climbing and descending distance over the same duration."""
    for index, prefix in enumerate(('up', 'down')):
        runs = sorted(name for name in phases if name.startswith(prefix + '_'))
        for order, name in enumerate(runs):
            samples = phases[name]
            times = [row[0] for row in samples]
            travel = [abs(row[2]) for row in samples]
            axes.plot(times, travel, color=SERIES[index], linewidth=2.0,
                      alpha=1.0 if order == 0 else 0.45,
                      label=('climbing' if prefix == 'up' else 'descending')
                      if order == 0 else None)
    style_axes(axes, 'Climbing versus descending travel',
               'time (s)', 'along-wall vertical travel (m)')
    axes.legend(frameon=False, fontsize=9, labelcolor=INK_SOFT, loc='upper left')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory_csv')
    parser.add_argument('--output', default='results/wall_slip.png')
    parser.add_argument(
        '--title', default='Gravity-driven wall slip, Gazebo ground truth')
    arguments = parser.parse_args()

    # Selected here rather than at import time so the module stays importable
    # without a display and the import block keeps its checked ordering.
    matplotlib.use('Agg')
    plt.rcParams['axes.unicode_minus'] = False

    phases = read_trajectory(arguments.trajectory_csv)
    # The wall-plane path is wide and short at equal aspect, so it gets the
    # full width and the two analysis panels share the row below it.
    figure = plt.figure(figsize=(12, 8.5), facecolor=SURFACE)
    grid = figure.add_gridspec(2, 2, height_ratios=[1, 1.15], hspace=0.32,
                               wspace=0.24)
    plot_wall_plane(figure.add_subplot(grid[0, :]), phases)
    plot_descent_ratio(figure.add_subplot(grid[1, 0]), phases)
    plot_vertical(figure.add_subplot(grid[1, 1]), phases)
    figure.suptitle(arguments.title, color=INK, fontsize=13, x=0.02, ha='left')
    figure.savefig(arguments.output, dpi=160, facecolor=SURFACE,
                   bbox_inches='tight')
    print('Wrote %s' % arguments.output)


if __name__ == '__main__':
    main()
