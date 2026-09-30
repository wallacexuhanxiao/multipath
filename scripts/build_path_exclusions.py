"""Generate cumulative exclusion boxes from full successful actual rollouts."""
import argparse
import json
from pathlib import Path
from multipath.path_exclusion import build_exclusions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--accepted-run', type=Path, action='append', required=True)
    parser.add_argument('--window-start', type=int, help='Optional explicit interval; otherwise infer from the schedule')
    parser.add_argument('--window-end', type=int)
    parser.add_argument('--half-size', type=float, default=.025)
    parser.add_argument('--size-profile',choices=['uniform','tapered'],default='tapered')
    parser.add_argument('--exclusion-scope',choices=['robot','wrist'],default='wrist')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = build_exclusions(args.accepted_run, args.window_start, args.window_end, args.half_size,
                              adaptive_size=args.size_profile=='tapered',exclusion_scope=args.exclusion_scope)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'output': str(args.output), 'prior_solutions': len(result['sources']),
                      'obstacles': sum(len(s['cuboid']) for s in result['segments']),
                      'windows': [s['window'] for s in result['segments']]}))


if __name__ == '__main__':
    main()
