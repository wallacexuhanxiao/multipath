"""Bound native CUDA/Isaac startup; no physics or source-clock changes."""
import faulthandler
import json
import os
import time
from pathlib import Path


def watch_dependency_imports(output_dir,timeout_s=120):
    """Cover the scientific imports that occur before SimulationApp startup."""
    faulthandler.enable()
    faulthandler.dump_traceback_later(timeout_s,exit=True)
    Path(output_dir,'startup_result.json').write_text(json.dumps({'stage':'PYTHON_DEPENDENCIES_BEGIN',
        'pbs_job_id':os.environ.get('PBS_JOBID'),'timeout_s':timeout_s},indent=2))
    print('PYTHON_DEPENDENCIES_BEGIN',flush=True)


def start_simulation(output_dir, timeout_s=180):
    status_path=Path(output_dir)/'startup_result.json'
    started=time.monotonic()
    def mark(stage, **extra):
        if stage.endswith('_BEGIN'):
            # Budget each native stage separately; import time is not Isaac time.
            faulthandler.dump_traceback_later(timeout_s,exit=True)
        status_path.write_text(json.dumps(dict(stage=stage,elapsed_s=time.monotonic()-started,
            pbs_job_id=os.environ.get('PBS_JOBID'),**extra),indent=2))
        print(stage,extra,flush=True)
    faulthandler.enable()
    try:
        mark('TORCH_IMPORT_BEGIN')
        import torch
        mark('CUDA_PROBE_BEGIN',visible_devices=os.environ.get('CUDA_VISIBLE_DEVICES'),
             torch_file=torch.__file__,torch_version=torch.__version__)
        probe=torch.ones(1,device='cuda');torch.cuda.synchronize()
        mark('CUDA_PROBE_READY',device=torch.cuda.get_device_name(0),value=float(probe.sum()))
        del probe
        mark('ISAAC_IMPORT_BEGIN')
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
