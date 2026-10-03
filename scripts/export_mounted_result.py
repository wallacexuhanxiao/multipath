"""Export an honest compact report and reuse the measured-state video renderer.

No simulator launch, reference animation, task re-scoring, or source-clock changes.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys


FIELDS = (
    'status', 'anchor', 'pbs_job_id', 'm2_success', 'success', 'full_pipeline_success',
    'task_success', 'prefix_success', 'development_success', 'source_fps',
    'source_start_s', 'source_duration_s', 'actual_duration_s', 'core_interval',
    'synthesis_interval', 'core_replay_success', 'padded_rollout_complete',
    'handoff_verified', 'state_resets_between_segments',
    'state_resets_within_padded_rollout', 'state_resets_within_core_replay',
    'development_snapshot_restores', 'measured_velocity_ratio',
    'measured_acceleration_ratio', 'measured_acceleration_policy',
    'collision_validation_note', 'error', 'failures', 'source_provenance',
)


def read_json(path):
    return json.loads(path.read_text())


def summarize(folder, render, python):
    result_path = folder / 'pipeline_result.json'
    result = read_json(result_path)
    item = {'directory': str(folder), 'evidence': str(result_path),
            **{k: result[k] for k in FIELDS if k in result}}
    if 'handoff' in result:
        handoff = result['handoff']
        item['handoff'] = {k: handoff[k] for k in
                           ('success', 'arm_q_error_rad', 'object_boundary_error') if k in handoff}
        item['handoff']['contact_mode'] = handoff.get('actual_contact', {}).get('mode')
    gate = folder / 'local_gate_result.json'
    if gate.exists():
        value = read_json(gate)
        item['exported_core_gate'] = {k: value[k] for k in FIELDS if k in value}
    trace_path = folder / 'actual_trace.json'
    item['video'] = {'status': 'not_requested' if not render else 'unavailable'}
    if not trace_path.exists():
        item['recording'] = 'No actual-state trace; no video can be exported.'
        return item
    trace = read_json(trace_path)
    item['actual_samples'] = len(trace)
    if not trace:
        item['recording'] = 'Empty actual-state trace; no video can be exported.'
        return item
    item['recorded_source_interval_s'] = [trace[0]['time_s'], trace[-1]['time_s']]
    item['last_actual_sample'] = {k: trace[-1][k] for k in ('time_s', 'source_frame', 'mode') if k in trace[-1]}
    if not render:
        return item
    required = ('visual_meshes.npz', 'visual_actual.npz', 'visual_mesh_paths.json')
    missing = [name for name in required if not (folder / name).exists()]
    if missing:
        item['video']['reason'] = 'Missing measured mesh data: ' + ', '.join(missing)
        return item
    try:
        fps = float(result['source_fps'])
        duration = float(result['actual_duration_s'])
        if fps <= 0 or duration <= 0 or not math.isfinite(fps + duration):
            raise ValueError('Non-positive or non-finite source clock')
        # Existing renderer places one recorded sample per source frame. Reject
        # irregular sampling rather than silently changing playback speed.
        n = min(len(trace), max(1, math.ceil(duration * fps - 1e-8))) if result.get('source') == 'oracle' else len(trace)
        origin = float(trace[0]['time_s'])
        for i, sample in enumerate(trace[:n]):
            time = float(sample['time_s'])
            if not math.isfinite(time) or abs(time - origin - i / fps) > 1e-6:
                raise ValueError('Recorded sample times are not on the source clock; rendering refused')
        if 'table_z' not in result:
            raise ValueError('Recorded table geometry missing')
        log = folder / 'result_export_render.log'
        with log.open('w') as output:
            proc = subprocess.run([python, str(Path(__file__).with_name('render_mounted_actual.py')),
                                   '--run-dir', str(folder), '--stride', '1'],
                                  stdout=output, stderr=subprocess.STDOUT, timeout=180)
        if proc.returncode:
            raise RuntimeError(f'Renderer exited {proc.returncode}; see {log}')
        video = folder / 'mounted_xhand_bc_curobo_actual.mp4'
        if not video.exists():
            raise RuntimeError('Renderer produced no video')
        item['video'] = {'status': 'exported', 'path': str(video), 'fps': fps,
                         'expected_duration_s': n / fps,
                         'source': 'measured PhysX mesh transforms; visualization only',
                         'timing_note': 'Native source fps, no stretching. A partial final interval is quantized to one video frame.'}
    except (KeyError, ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as error:
        item['video'] = {'status': 'failed', 'reason': str(error)}
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--report-only', action='store_true', help='Standard-library-only report; do not render')
    parser.add_argument('--render-python', default=sys.executable, help='Python with numpy/mujoco/opencv/trimesh/scipy')
    args = parser.parse_args()
    root = args.run_dir.resolve()
    if not root.is_dir():
        parser.error(f'Run directory does not exist: {root}')
    report = {'run_directory': str(root), 'scope': 'Recorded evidence only; no new physics validation',
              'physical_success': 'unverified', 'stages': []}
    for name, key in [('development_result.json', 'development'), ('startup_result.json', 'startup')]:
        path = root / name
        if path.exists():
            try:
                value = read_json(path)
                report[key] = value if key == 'startup' else {k: value[k] for k in FIELDS if k in value}
            except (ValueError, OSError) as error:
                report[key] = {'read_error': str(error)}
    # Ignore MPC candidate trials: replay and exported-core results are the gates.
    paths = [root / 'pipeline_result.json']
    for child in sorted(root.iterdir()):
        if child.is_dir() and child.name not in ('implementation', 'mpc_search', 'anchors'):
            paths.append(child / 'pipeline_result.json')
            if child.name.startswith('anchor_'):
                paths.extend(child / stage / 'pipeline_result.json' for stage in
                             ('replay', 'replay_core', 'mpc_replay', 'mpc_core'))
    for path in paths:
        if path.exists():
            try:
                report['stages'].append(summarize(path.parent, not args.report_only, args.render_python))
            except (ValueError, OSError, KeyError, TypeError) as error:
                report['stages'].append({'directory': str(path.parent), 'read_error': str(error)})
    if any(s.get('directory') == str(root) and s.get('full_pipeline_success') is True for s in report['stages']):
        report['physical_success'] = 'full_pipeline_passed_as_reported'
    elif report.get('development', {}).get('m2_success') is True:
        report['physical_success'] = 'local_anchor_passed_as_reported; full_pipeline_not_proven'
    elif report['stages']:
        report['physical_success'] = 'not_established; inspect_stage_results'
    if not report['stages']:
        report['note'] = 'No completed replay result. Initialization/queued/running/failed status cannot be inferred from missing artifacts alone. No video fabricated.'
    search = root / 'mpc_search' / 'search.json'
    if search.exists():
        try:
            trials = read_json(search)
            report['mpc_search'] = [
                {'begin_tick': tick, 'candidates': len(group),
                 'nominal_precheck_passed': sum(t.get('feasible_controls') is True for t in group),
                 'physical_score_recorded': sum(t.get('score') is not None for t in group)}
                for tick in sorted({t['begin_tick'] for t in trials})
                for group in [[t for t in trials if t['begin_tick'] == tick]]]
        except (ValueError, OSError, KeyError, TypeError) as error:
            report['mpc_search'] = {'read_error': str(error)}
    log = root / 'run.log'
    if log.exists():
        # Keep the saved diagnostic small and avoid loading multi-GB logs.
        with log.open('rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 12000))
            report['log_tail'] = stream.read().decode('utf-8', errors='replace').splitlines()[-20:]
    json_path, md_path = root / 'result_export.json', root / 'RESULT.md'
    json_path.write_text(json.dumps(report, indent=2) + '\n')
    lines = ['# Recorded experiment result', '', f"- Physical result: **{report['physical_success']}**",
             '- Videos show actual PhysX states. Rendering does not establish interaction or task success.',
             '- Missing fields mean unreported; acceleration ratio is diagnostic, not a new acceptance gate.', '']
    for key in ('development', 'startup', 'mpc_search'):
        if key in report:
            lines += [f'## {key.capitalize()}', '', '```json', json.dumps(report[key], indent=2), '```', '']
    for stage in report['stages']:
        lines += [f"## {Path(stage['directory']).relative_to(root)}", '', '```json', json.dumps(stage, indent=2), '```', '']
        if stage.get('video', {}).get('status') == 'exported':
            lines += [f"[Actual-state video](<{stage['video']['path']}>)", '']
    if report.get('note'):
        lines += [report['note'], '']
    if report.get('log_tail'):
        lines += ['## Last log lines', '', '```text', *report['log_tail'], '```', '']
    md_path.write_text('\n'.join(lines))
    print(json.dumps({'report': str(md_path), 'json': str(json_path), 'physical_success': report['physical_success']}))
    # An exported failure report is a successful export, not a successful rollout.
    if any(s.get('video', {}).get('status') == 'failed' or 'read_error' in s for s in report['stages']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
