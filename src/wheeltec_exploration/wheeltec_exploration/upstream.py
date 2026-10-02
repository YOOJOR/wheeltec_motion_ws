"""Load only the locked official FMM module; never import Go2/Agent/VLM entry points."""
import importlib.util
from pathlib import Path
import subprocess

LOCKED_COMMIT = '891ba09959278b666ef01a7f8b6c64f8fe8436cc'


def load_fmm(root):
    root = Path(root).resolve()
    revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != LOCKED_COMMIT:
        raise ValueError('UPSTREAM_VERSION_MISMATCH')
    relative = 'utils/fmm_planner.py'
    if subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain', '--', relative], text=True):
        raise ValueError('UPSTREAM_FMM_MODIFIED')
    spec = importlib.util.spec_from_file_location('wheeltec_locked_fmm', root/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.FMMPlanner
