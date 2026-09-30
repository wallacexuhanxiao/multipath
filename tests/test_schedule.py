import json
from pathlib import Path

import pytest

from multipath.schedule import Schedule


def test_put_in_schedule_has_bc_and_curobo_windows():
    root = Path(__file__).resolve().parents[1]
    schedule = Schedule.load(root / "configs/put_in_bc_curobo_schedule.json")
    assert [(w.start, w.end) for w in schedule.bc_windows] == [(54, 65), (86, 122)]
    assert [(w.start, w.end) for w in schedule.curobo_windows] == [
        (0, 54), (65, 86), (122, 144)]
    assert all(x["bc_physical_validation"] == "pending" for x in
               schedule.bootstrap_manifest()["windows"] if x["controller"] == "BC")


def test_contact_checked_schedule_keeps_grasp_contact_in_bc():
    root = Path(__file__).resolve().parents[1]
    schedule = Schedule.load(root / "configs/put_in_bc_curobo_contact_checked.json")
    assert [(w.start, w.end) for w in schedule.bc_windows] == [(43, 65), (86, 122)]
    assert schedule.windows[0].end == 43


def test_rejects_schedule_gap(tmp_path):
    root = Path(__file__).resolve().parents[1]
    raw = json.loads((root / "configs/put_in_bc_curobo_schedule.json").read_text())
    raw["schedule"][2]["start"] += 1
    path = tmp_path / "gap.json"
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="invalid schedule window"):
        Schedule.load(path)
