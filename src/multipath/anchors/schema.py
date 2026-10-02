from dataclasses import dataclass


@dataclass(frozen=True)
class InteractionAnchor:
    id: str
    core_start: int
    core_end: int
    synth_start: int
    synth_end: int
    entry_contact: str
    exit_contact: str

    def __post_init__(self):
        if any(not isinstance(v,int) for v in (self.core_start,self.core_end,self.synth_start,self.synth_end)):
            raise ValueError('Anchor frames must be integers')
        if not 0 <= self.synth_start <= self.core_start < self.core_end <= self.synth_end:
            raise ValueError('Invalid core/context intervals')
        if {self.entry_contact, self.exit_contact} - {'free', 'grasped'}:
            raise ValueError('V0 contact modes are free/grasped')
