"""Execute the development, selection and evaluation stages in order."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[1]
OUT = BASE / 'aligned'
OUT.mkdir(exist_ok=True)


def status(stage, state, code=None):
    value = dict(stage=stage, state=state, exit_code=code, pid=os.getpid(),
                 updated_at=datetime.now(timezone.utc).isoformat())
    temp = OUT/'status.tmp'
    temp.write_text(json.dumps(value, indent=2)+'\n')
    temp.replace(OUT/'status.json')


with (OUT/'.pipeline.lock').open('w') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    for stage in ('development', 'freeze', 'evaluation'):
        destination = OUT/('development' if stage == 'freeze' else stage)
        cmd = [sys.executable, '-u', str(ROOT/'py/run_orca_experiment.py'), stage,
               '--output', str(destination), '--workers', '4']
        if stage == 'evaluation':
            cmd += ['--selection', str(OUT/'development/selection.json')]
        status(stage, 'running')
        with (OUT/f'{stage}.log').open('a', buffering=1) as log:
            code = subprocess.call(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        if code:
            status(stage, 'failed', code)
            sys.exit(code)
        status(stage, 'complete', 0)
    status('all', 'complete', 0)
