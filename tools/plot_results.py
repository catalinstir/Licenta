#!/usr/bin/env python3
"""
Plot NXP CUP telemetry CSVs for thesis figures.

Generates six figures per run (or overlaid comparisons across runs):
  1. Lateral error  (e_lat)  vs time
  2. Heading error  (theta_e) vs time
  3. Steering command          vs time
  4. Vehicle speed             vs time
  5. Phase portrait: e_lat vs theta_e
  6. Lateral error histogram

Usage:
    # Single run
    python tools/plot_results.py tools/data/lqr_20260624_120000.csv

    # Compare multiple runs — labels derived from filenames automatically
    python tools/plot_results.py tools/data/pid_*.csv tools/data/lqr_*.csv

    # Override labels
    python tools/plot_results.py pid.csv lqr.csv --labels "PID" "LQR"

    # Save PNGs instead of showing interactively
    python tools/plot_results.py *.csv --save tools/data/plots/
"""

import argparse
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
except ImportError as e:
    print(f'Missing dependency: {e}')
    print('Run: pip install matplotlib numpy pandas')
    sys.exit(1)

COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']
FIG_W, FIG_H = 10, 4


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    numeric = ['time_s', 'steer', 'motor_speed', 'rpm', 'speed_kmh',
               'distance', 'vectors', 'e_lat', 'theta_e', 'pid_error', 'pid_control']
    for col in numeric:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    return df


def label_from_path(path: Path) -> str:
    # "pid_20260624_120000.csv"  →  "pid"
    parts = path.stem.split('_')
    return parts[0] if len(parts) > 1 else path.stem


def steer_from(df: pd.DataFrame) -> pd.DataFrame:
    """Return a DataFrame with (time_s, steer) drawn from the most informative source."""
    ctrl = df[df['record_type'].isin(['control_lqr', 'control_pid'])].dropna(subset=['steer'])
    if not ctrl.empty:
        return ctrl[['time_s', 'steer']]
    telem = df[df['record_type'] == 'telemetry'].dropna(subset=['steer'])
    return telem[['time_s', 'steer']]


def event_times(df: pd.DataFrame, label: str):
    evts = df[(df['record_type'] == 'event') & (df['event'] == label)]
    return evts['time_s'].tolist()


# ---------------------------------------------------------------------------
# Individual plots
# ---------------------------------------------------------------------------

def _finish(fig, ax, xlabel, ylabel, title, save_dir, fname):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    if save_dir:
        p = Path(save_dir) / fname
        fig.savefig(p, dpi=150)
        print(f'  Saved: {p}')
        plt.close(fig)
    else:
        plt.show()


def plot_e_lat(datasets, labels, colors, save_dir):
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    # Track whether legend entries for intersection markers have been added yet
    _legend_real = _legend_fail = False
    for df, label, color in zip(datasets, labels, colors):
        d = df[df['record_type'] == 'control_lqr'].dropna(subset=['e_lat'])
        if d.empty:
            continue
        ax.plot(d['time_s'], d['e_lat'] * 100, label=label, color=color, lw=1.5)
        for t in event_times(df, 'intersection_real'):
            ax.axvline(t, color='gray', lw=1.0, ls='--', alpha=0.7,
                       label='intersection' if not _legend_real else '_')
            _legend_real = True
        for evt in ('intersection_no_vectors', 'intersection_bad_vectors'):
            for t in event_times(df, evt):
                ax.axvline(t, color='red', lw=1.0, ls=':', alpha=0.7,
                           label='tracking failure' if not _legend_fail else '_')
                _legend_fail = True
    ax.axhline(0, color='k', lw=0.8, ls='--', alpha=0.5)
    _finish(fig, ax, 'Time [s]', 'Lateral error [cm]', 'Lateral tracking error', save_dir, 'e_lat.png')


def plot_theta_e(datasets, labels, colors, save_dir):
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    for df, label, color in zip(datasets, labels, colors):
        d = df[df['record_type'] == 'control_lqr'].dropna(subset=['theta_e'])
        if d.empty:
            continue
        ax.plot(d['time_s'], np.degrees(d['theta_e']), label=label, color=color, lw=1.5)
    ax.axhline(0, color='k', lw=0.8, ls='--', alpha=0.5)
    _finish(fig, ax, 'Time [s]', 'Heading error [°]', 'Heading error', save_dir, 'theta_e.png')


def plot_steer(datasets, labels, colors, save_dir):
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    for df, label, color in zip(datasets, labels, colors):
        d = steer_from(df)
        if d.empty:
            continue
        ax.step(d['time_s'], d['steer'], label=label, color=color, lw=1.2, where='post')
    ax.set_yticks([500, 600, 700, 800, 900])
    ax.set_yticklabels(['500\n(full L)', '600', '700\n(neutral)', '800', '900\n(full R)'])
    _finish(fig, ax, 'Time [s]', 'Steering duty × 100', 'Steering command', save_dir, 'steer.png')


def plot_speed(datasets, labels, colors, save_dir):
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    for df, label, color in zip(datasets, labels, colors):
        d = df[df['record_type'] == 'telemetry'].dropna(subset=['speed_kmh'])
        if d.empty:
            continue
        ax.plot(d['time_s'], d['speed_kmh'], label=label, color=color, lw=1.5)
    _finish(fig, ax, 'Time [s]', 'Speed [km/h]', 'Vehicle speed', save_dir, 'speed.png')


def plot_phase(datasets, labels, colors, save_dir):
    fig, ax = plt.subplots(figsize=(6, 6))
    for df, label, color in zip(datasets, labels, colors):
        d = df[df['record_type'] == 'control_lqr'].dropna(subset=['e_lat', 'theta_e'])
        if d.empty:
            continue
        ax.scatter(d['e_lat'] * 100, np.degrees(d['theta_e']),
                   label=label, color=color, s=10, alpha=0.5)
    ax.axhline(0, color='k', lw=0.6, alpha=0.4)
    ax.axvline(0, color='k', lw=0.6, alpha=0.4)
    ax.set_xlabel('Lateral error [cm]')
    ax.set_ylabel('Heading error [°]')
    ax.set_title('State-space phase portrait')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    if save_dir:
        p = Path(save_dir) / 'phase.png'
        fig.savefig(p, dpi=150)
        print(f'  Saved: {p}')
        plt.close(fig)
    else:
        plt.show()


def plot_e_lat_hist(datasets, labels, colors, save_dir):
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    for df, label, color in zip(datasets, labels, colors):
        d = df[df['record_type'] == 'control_lqr'].dropna(subset=['e_lat'])
        if d.empty:
            continue
        ax.hist(d['e_lat'] * 100, bins=40, label=label, color=color,
                alpha=0.55, density=True, edgecolor='none')
    ax.axvline(0, color='k', lw=0.8, ls='--', alpha=0.5)
    _finish(fig, ax, 'Lateral error [cm]', 'Density',
            'Lateral error distribution', save_dir, 'e_lat_hist.png')


# ---------------------------------------------------------------------------
# Summary statistics table
# ---------------------------------------------------------------------------

def print_stats(datasets, labels):
    cols = ['e_lat mean [cm]', 'e_lat std [cm]', 'e_lat |max| [cm]',
            'theta_e std [°]', 'n control', 'intersections', 'track fails']
    header = f'{"Run":<28}' + ''.join(f'{c:>16}' for c in cols)
    print('\n' + header)
    print('-' * len(header))
    for df, label in zip(datasets, labels):
        d = df[df['record_type'] == 'control_lqr'].dropna(subset=['e_lat', 'theta_e'])
        evts = df[df['record_type'] == 'event']
        n_real = (evts['event'] == 'intersection_real').sum()
        n_fail = evts['event'].isin(['intersection_no_vectors', 'intersection_bad_vectors']).sum()
        if d.empty:
            vals = ['n/a'] * 5 + [str(n_real), str(n_fail)]
            print(f'{label:<28}' + ''.join(f'{v:>16}' for v in vals))
            continue
        e = d['e_lat'] * 100
        th = np.degrees(d['theta_e'])
        vals = [f'{e.mean():+.2f}', f'{e.std():.2f}', f'{e.abs().max():.2f}',
                f'{th.std():.2f}', str(len(d)), str(n_real), str(n_fail)]
        print(f'{label:<28}' + ''.join(f'{v:>16}' for v in vals))
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description='Plot NXP CUP telemetry CSVs.')
    p.add_argument('files', nargs='+', help='CSV files produced by data_logger.py')
    p.add_argument('--labels', nargs='*',
                   help='Override run labels (one per file, default: from filename)')
    p.add_argument('--save', metavar='DIR', nargs='?', const='tools/data/plots',
                   help='Save figures as PNG in DIR (default: tools/data/plots/) instead of showing')
    args = p.parse_args()

    paths = [Path(f) for f in args.files]
    labels = args.labels if args.labels else [label_from_path(p) for p in paths]

    if len(labels) != len(paths):
        print(f'Error: {len(paths)} files but {len(labels)} labels.', file=sys.stderr)
        sys.exit(1)

    datasets = [load(p) for p in paths]
    colors = [COLORS[i % len(COLORS)] for i in range(len(datasets))]

    if args.save:
        Path(args.save).mkdir(parents=True, exist_ok=True)
        print(f'Saving figures to: {args.save}')

    print_stats(datasets, labels)

    plot_e_lat(datasets, labels, colors, args.save)
    plot_theta_e(datasets, labels, colors, args.save)
    plot_steer(datasets, labels, colors, args.save)
    plot_speed(datasets, labels, colors, args.save)
    plot_phase(datasets, labels, colors, args.save)
    plot_e_lat_hist(datasets, labels, colors, args.save)


if __name__ == '__main__':
    main()
