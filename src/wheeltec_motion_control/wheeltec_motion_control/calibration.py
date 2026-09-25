"""Offline fixed-centre calibration. Never creates a ROS node or publishes motion."""
import argparse
import csv
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np


@dataclass
class Options:
    min_samples: int = 30
    min_angle_deg: float = 180.0
    min_rate: float = 0.03
    max_gap: float = 0.5
    max_jump: float = 0.3
    max_condition: float = 20.0
    max_rms: float = 0.03
    max_p95: float = 0.06
    max_direction_difference: float = 0.03
    max_axis_variation_deg: float = 5.0
    level_tolerance_deg: float = 2.0
    body_z: float = None

    def validate(self):
        for name, value in asdict(self).items():
            if name == 'body_z':
                if value is not None and not math.isfinite(value):
                    raise ValueError('body_z must be finite')
            elif not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.min_samples < 3 or self.min_angle_deg > 360:
            raise ValueError('min_samples >= 3 and min_angle_deg <= 360 required')


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or abs(norm-1) > 0.1:
        raise ValueError('invalid orientation quaternion')
    x, y, z, w = q/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


COLUMNS = ['stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']


def read_csv(path):
    with Path(path).open() as stream:
        reader = csv.DictReader(stream)
        if not set(COLUMNS).issubset(reader.fieldnames or []):
            raise ValueError('CSV needs columns: '+','.join(COLUMNS))
        return np.array([[float(row[key]) for key in COLUMNS] for row in reader])


def read_bag(path, topic, world_frame, body_frame, storage_id):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if types.get(topic) != 'nav_msgs/msg/Odometry':
        raise ValueError(f'{topic} missing or not nav_msgs/msg/Odometry')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        _, serialized, _ = reader.read_next()
        msg = deserialize_message(serialized, Odometry)
        if msg.header.frame_id != world_frame or msg.child_frame_id != body_frame:
            raise ValueError('odometry frame mismatch; check --world-frame and --body-frame')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        rows.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                     p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    return np.asarray(rows)


def fit_segment(p, rotations, z, options):
    # Each segment has its own unknown centre: remove its mean analytically.
    centered = rotations-rotations.mean(axis=0)
    A = centered[:, :, :2].reshape(-1, 2)
    b = -(p-p.mean(axis=0)+centered[:, :, 2]*z).reshape(-1)
    xy, _, rank, singular = np.linalg.lstsq(A, b, rcond=None)
    condition = float(singular[0]/singular[-1]) if singular[-1] > 1e-12 else math.inf
    if rank < 2 or condition > options.max_condition:
        raise ValueError('horizontal translation is ill-conditioned for the supplied body Z constraint')
    t = np.array([*xy, z])
    corrected = p+np.einsum('nij,j->ni', rotations, t)
    centre = corrected.mean(axis=0)
    errors = np.linalg.norm(corrected-centre, axis=1)
    full_singular = np.linalg.svd(centered.reshape(-1, 3), compute_uv=False)
    return dict(translation=t.tolist(), centre=centre.tolist(),
                rms=float(np.sqrt(np.mean(errors**2))), p95=float(np.percentile(errors, 95)),
                maximum=float(errors.max()), condition=condition,
                unconstrained_singular_values=full_singular.tolist())


def analyze(rows, options):
    options.validate()
    if rows.ndim != 2 or rows.shape[1] != 8 or len(rows) < options.min_samples:
        raise ValueError('not enough pose samples')
    if not np.all(np.isfinite(rows)):
        raise ValueError('non-finite pose data')
    dt = np.diff(rows[:, 0])
    if np.any(dt <= 0):
        raise ValueError('source timestamps repeat or go backwards; use a single continuous recording')
    p = rows[:, 1:4]
    R = np.array([rotation(q) for q in rows[:, 4:8]])
    if np.any(np.linalg.norm(np.diff(p, axis=0), axis=1) > options.max_jump):
        raise ValueError('position jump detected; select a continuous, stable recording')
    yaw = np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0]))
    rate = np.diff(yaw)/dt
    direction = np.where(np.abs(rate) >= options.min_rate, np.sign(rate), 0)
    direction[dt > options.max_gap] = 0
    runs, start = [], 0
    # Maximal contiguous left/right runs. Stops/gaps separate independent centres.
    while start < len(direction):
        sign = direction[start]
        end = start+1
        while end < len(direction) and direction[end] == sign:
            end += 1
        if sign and end-start+1 >= options.min_samples:
            angle = abs(math.degrees(yaw[end]-yaw[start]))
            if angle >= options.min_angle_deg:
                runs.append((start, end+1, int(sign), angle))
        start = end
    if not runs:
        raise ValueError('no usable rotation segment; record slow continuous turns with sufficient angular coverage')
    used = np.unique(np.concatenate([np.arange(a, b) for a, b, _, _ in runs]))
    up_vectors = R[used, 2, :]
    axis = up_vectors.mean(axis=0)
    axis /= np.linalg.norm(axis)
    tilt = math.degrees(math.acos(np.clip(abs(axis[2]), 0, 1)))
    variation = float(np.max(np.degrees(np.arccos(np.clip(up_vectors @ axis, -1, 1)))))
    z = options.body_z if options.body_z is not None else 0.0
    segments = []
    for a, b, sign, angle in runs:
        fit = fit_segment(p[a:b], R[a:b], z, options)
        fit.update(start_index=a, end_index=b, samples=b-a, direction='left' if sign > 0 else 'right',
                   angle_deg=angle, start_sec=float(rows[a, 0]-rows[0, 0]),
                   end_sec=float(rows[b-1, 0]-rows[0, 0]))
        segments.append(fit)
    # Equal weight per segment prevents a slower/longer turn dominating the result.
    translations = np.array([s['translation'] for s in segments])
    t = translations.mean(axis=0)
    spread = float(np.max(np.linalg.norm(translations[:, :2]-t[:2], axis=1)))
    groups = {d: [s for s in segments if s['direction'] == d] for d in ('left', 'right')}
    difference = None
    if all(groups.values()):
        difference = float(np.linalg.norm(
            np.mean([s['translation'] for s in groups['left']], axis=0)[:2]-
            np.mean([s['translation'] for s in groups['right']], axis=0)[:2]))
    reasons = []
    if difference is None:
        reasons.append('Both left and right rotations are required for cross-checking.')
    elif difference > options.max_direction_difference:
        reasons.append('Left/right translation estimates disagree.')
    if spread > options.max_direction_difference:
        reasons.append('Repeated segment estimates disagree.')
    if variation > options.max_axis_variation_deg:
        reasons.append('Rotation axis changes excessively; chassis tilt or localization instability.')
    if tilt > options.level_tolerance_deg and options.body_z is None:
        reasons.append('IMU is tilted: X/Y depend on the unknown body Z value; supply --body-z.')
    # Evaluate the common estimate too, not just each individual fitted lever arm.
    for s in segments:
        a, b = s['start_index'], s['end_index']
        centres = p[a:b]+np.einsum('nij,j->ni', R[a:b], t)
        errors = np.linalg.norm(centres-centres.mean(axis=0), axis=1)
        s['common_rms'] = float(np.sqrt(np.mean(errors**2)))
        s['common_p95'] = float(np.percentile(errors, 95))
        if max(s['rms'], s['common_rms']) > options.max_rms or max(s['p95'], s['common_p95']) > options.max_p95:
            reasons.append(f"Segment at {s['start_sec']:.2f}s has excessive centre drift/residual.")
    return dict(schema_version=1, samples=len(rows), used_samples=len(used),
                options=asdict(options), translation_conditioned_on_body_z=t.tolist(),
                body_z_measured=options.body_z is not None,
                rotation_axis_in_body=axis.tolist(), imu_z_axis_tilt_deg=tilt,
                axis_variation_deg=variation, left_right_difference_m=difference,
                segment_spread_m=spread, quality_passed=not reasons, reasons=reasons,
                segments=segments,
                limitations=['Z is externally supplied or fixed to zero, never estimated.',
                             'Rotation RPY is not calibrated by this tool.',
                             'A fixed effective rotation centre is assumed; low residual does not prove ground truth.',
                             'With unknown Z, X/Y are conditional on Z=0; tilted mounting couples all components.'])


def write_outputs(output, rows, report, plot=True):
    # A new directory is mandatory: never overwrite an earlier calibration.
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    np.savetxt(output/'poses.csv', rows, delimiter=',', header=','.join(COLUMNS), comments='', fmt='%.17g')
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    t = report['translation_conditioned_on_body_z']
    lines = ['# Offline rotation calibration', '',
             f"Quality checks passed: {report['quality_passed']}",
             f'Body translation conditioned on supplied Z (or Z=0): {t} m',
             f"Z externally measured: {report['body_z_measured']}",
             f"Left/right difference: {report['left_right_difference_m']} m", '',
             '## Limitations', *['- '+x for x in report['limitations']], '',
             '## Quality issues', *['- '+x for x in report['reasons']]]
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    if report['quality_passed'] and report['body_z_measured']:
        (output/'translation_suggestion.yaml').write_text(
            '# Fragment only. Review the chosen base reference point and rotation RPY separately.\n'
            '# This file is NOT automatically applied.\n'
            'wheeltec_motion_controller:\n  ros__parameters:\n'
            f'    body_from_base_translation: {t}\n'
            '    extrinsics_calibrated: false\n    control_enabled: false\n')
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(11, 5))
        for number, s in enumerate(report['segments'], 1):
            selected = rows[s['start_index']:s['end_index']]
            p = selected[:, 1:4]
            R = np.array([rotation(q) for q in selected[:, 4:8]])
            centres = p+np.einsum('nij,j->ni', R, t)
            label = f"{number}: {s['direction']}"
            axes[0].plot(p[:, 0], p[:, 1], label=label+' IMU')
            axes[0].plot(centres[:, 0], centres[:, 1], '--', label=label+' centre')
            error = np.linalg.norm(centres-centres.mean(axis=0), axis=1)
            axes[1].plot(selected[:, 0]-rows[0, 0], error, label=label)
        axes[0].set(xlabel='World X (m)', ylabel='World Y (m)', title='IMU and corrected centre')
        axes[0].set_aspect('equal', adjustable='datalim')
        axes[1].set(xlabel='Time (s)', ylabel='Distance from segment mean (m)', title='Centre residual')
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(output/'trajectory.png', dpi=150)
        plt.close(fig)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--bag', type=Path)
    inputs.add_argument('--csv', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='new output directory')
    parser.add_argument('--topic', default='/Odometry')
    parser.add_argument('--world-frame', default='camera_init')
    parser.add_argument('--body-frame', default='body')
    parser.add_argument('--storage-id', default='sqlite3')
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--start-sec', type=float, default=0.0)
    parser.add_argument('--end-sec', type=float)
    for name, value in asdict(Options()).items():
        parser.add_argument('--'+name.replace('_', '-'), type=int if name == 'min_samples' else float, default=value)
    args = parser.parse_args(args)
    try:
        if args.output.exists():
            raise ValueError('output directory already exists; choose a new name')
        if not math.isfinite(args.start_sec) or args.start_sec < 0 or (
                args.end_sec is not None and (not math.isfinite(args.end_sec) or args.end_sec <= args.start_sec)):
            raise ValueError('invalid time range')
        if not args.no_plot:
            import matplotlib  # Fail before creating output if plotting dependency is absent.
        rows = read_csv(args.csv) if args.csv else read_bag(
            args.bag, args.topic, args.world_frame, args.body_frame, args.storage_id)
        if not len(rows):
            raise ValueError('no odometry samples')
        relative = rows[:, 0]-rows[0, 0]
        rows = rows[(relative >= args.start_sec) & (relative <= (args.end_sec if args.end_sec is not None else math.inf))]
        report = analyze(rows, Options(**{name: getattr(args, name) for name in asdict(Options())}))
        report['input'] = str((args.csv or args.bag).resolve())
        report['pose_data_sha256'] = hashlib.sha256(rows.astype('<f8').tobytes()).hexdigest()
        report['command'] = sys.argv if args is None else vars(args).__repr__()
        write_outputs(args.output, rows, report, not args.no_plot)
        print(json.dumps({k: report[k] for k in ('quality_passed', 'translation_conditioned_on_body_z',
                                               'body_z_measured', 'left_right_difference_m', 'reasons')}, indent=2))
        print(f'Report: {args.output.resolve()}/report.md')
        return 0 if report['quality_passed'] else 2
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(f'Calibration failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
