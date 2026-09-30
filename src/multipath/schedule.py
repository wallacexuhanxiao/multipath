"""Validate a VLM selective-BC schedule for the BC/cuRobo pipeline.

The imported schedule identifies *where* BC is needed. It does not imply that
robot BC demonstrations, a trained controller, or physical anchors exist.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class Window:
    start: int
    end: int
    controller: str


@dataclass(frozen=True)
class Schedule:
    instruction: str
    num_frames: int
    fps: float
    windows: tuple[Window, ...]

    @property
    def bc_windows(self) -> tuple[Window, ...]:
        return tuple(w for w in self.windows if w.controller == "BC")

    @property
    def curobo_windows(self) -> tuple[Window, ...]:
        return tuple(w for w in self.windows if w.controller == "CUROBO")

    @classmethod
    def load(cls, path: str | Path) -> "Schedule":
        raw = json.loads(Path(path).read_text())
        n = int(raw["num_frames"])
        fps = float(raw["fps"])
        if n <= 0 or fps <= 0 or not str(raw.get("instruction", "")).strip():
            raise ValueError("invalid schedule duration or instruction")
        windows = []
        cursor = 0
        for item in raw["schedule"]:
            start, end = int(item["start"]), int(item["end"])
            mode = {"BC": "BC", "RELAXED": "CUROBO"}.get(item["mode"])
            if mode is None or start != cursor or end <= start or end > n:
                raise ValueError(f"invalid schedule window [{start},{end})")
            windows.append(Window(start, end, mode))
            cursor = end
        if cursor != n or not windows or not any(w.controller == "BC" for w in windows):
            raise ValueError("schedule must cover video and contain BC interaction")
        bc = [(w.start, w.end) for w in windows if w.controller == "BC"]
        declared = [(int(x["start"]), int(x["end"])) for x in raw["bc_intervals"]]
        if bc != declared:
            raise ValueError("BC windows disagree with declared intervals")
        return cls(str(raw["instruction"]), n, fps, tuple(windows))

    def bootstrap_manifest(self) -> dict:
        """The work required before a schedule can execute on a robot."""
        return {
            "instruction": self.instruction,
            "frames": self.num_frames,
            "fps": self.fps,
            "windows": [
                {
                    "start": w.start,
                    "end": w.end,
                    "controller": w.controller,
                    **({"bc_demonstration": "required_or_synthesize_with_egoengine",
                        "bc_physical_validation": "pending"}
                       if w.controller == "BC" else
                       {"start_end_states": "from_adjacent_verified_BC_or_initial_terminal",
                        "physical_rollout": "pending"}),
                }
                for w in self.windows
            ],
        }
