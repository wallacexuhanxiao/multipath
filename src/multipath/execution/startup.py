"""Bound native CUDA/Isaac startup; no physics or source-clock changes."""
import faulthandler
import json
import os
import time
from pathlib import Path


def start_simulation(output_dir, timeout_s=180):
    status_path=Path(output_dir)/'startup_result.json'
    started=time.monotonic()
    def mark(stage, **extra):
        status_path.write_text(json.dumps(dict(stage=stage,elapsed_s=time.monotonic()-started,
            pbs_job_id=os.environ.get('PBS_JOBID'),**extra),indent=2))
        print(stage,extra,flush=True)
    faulthandler.enable()
    faulthandler.dump_traceback_later(timeout_s,exit=True)
    try:
        mark('TORCH_IMPORT_BEGIN')
        import torch
        mark('CUDA_PROBE_BEGIN',visible_devices=os.environ.get('CUDA_VISIBLE_DEVICES'))
        probe=torch.ones(1,device='cuda');torch.cuda.synchronize()
        mark('CUDA_PROBE_READY',device=torch.cuda.get_device_name(0),value=float(probe.sum()))
        del probe
        from isaacsim import SimulationApp
        mark('SIMULATION_APP_BEGIN')
        app=SimulationApp({'headless':True})
        mark('SIMULATION_APP_READY')
        return app
    except Exception as error:
        mark('STARTUP_FAILED',error=f'{type(error).__name__}: {error}')
        raise
    finally:
        faulthandler.cancel_dump_traceback_later()
