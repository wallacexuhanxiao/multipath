"""Inspect a VLM schedule without implying physical execution success."""
from __future__ import annotations

import argparse
import json

from multipath.schedule import Schedule


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("schedule")
    args = parser.parse_args()
    print(json.dumps(Schedule.load(args.schedule).bootstrap_manifest(), indent=2))


if __name__ == "__main__":
    main()
