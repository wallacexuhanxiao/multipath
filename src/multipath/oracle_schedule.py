"""Static, explicitly reviewed segmentation. No discovery or VLM dependency."""
from dataclasses import dataclass
import json
from pathlib import Path
from .anchors.schema import InteractionAnchor


@dataclass
class OracleSchedule:
    fps: float
    num_frames: int
    windows: list
    anchors: dict
    metadata: dict

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        if data['source'] != 'oracle':
            raise ValueError('V0 requires oracle segmentation')
        anchors = {a['id']: InteractionAnchor(**a) for a in data['anchors']}
        cursor = 0
        previous_mode = 'free'
        used = []
        for w in data['windows']:
            if w['start'] != cursor or w['end'] <= cursor:
                raise ValueError('Schedule must cover the source once, with no gaps')
            cursor = w['end']
            if w['mode'] == 'ANCHOR':
                a = anchors[w['anchor']]
                if [a.core_start, a.core_end] != [w['start'], w['end']]:
                    raise ValueError('Runtime Anchor window must be exactly its core')
                if a.entry_contact != previous_mode:
                    raise ValueError('Contact change requires an interaction Anchor')
                previous_mode = a.exit_contact
                used.append(a.id)
            elif w['mode'] == 'RELAXED':
                if w['contact'] != previous_mode:
                    raise ValueError('cuRobo cannot change contact mode')
            else:
                raise ValueError('Unknown V0 mode')
        if cursor != data['num_frames'] or set(used) != set(anchors) or len(used) != len(anchors):
            raise ValueError('Incomplete or repeated schedule')
        if any(a.synth_end > cursor for a in anchors.values()):
            raise ValueError('Synthesis context extends outside source')
        return cls(data['fps'], cursor, data['windows'], anchors, data)
