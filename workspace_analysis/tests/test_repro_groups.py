"""Missing artifacts must fail explicitly; relocated saved Viewer tests retained."""
from hashlib import sha256
from pathlib import Path
import shutil
import subprocess
import sys
from test_resources import ARTIFACTS

ROOT=Path(__file__).resolve().parents[1]
VIEWERS=['task_approach/p22_20261104_03/figures/q0_approach.html',
         'task_approach/p22_20261104_03/figures/q1_approach.html',
         'task_planning/p22_20261104_02/figures/q0_linear.html']


def invoke(path):
    return subprocess.run([sys.executable,'-B','-m','pytest','-q','-rs','-p','no:cacheprovider','-m','artifact',
        '--artifact-root',str(path),'tests/integration/test_saved_viewer_artifacts.py'],cwd=ROOT,capture_output=True,text=True,timeout=40)


def test_explicit_missing_artifacts_and_lfs_pointers_block_with_nonzero_exit(tmp_path):
    empty=tmp_path/'missing';empty.mkdir();r=invoke(empty)
    assert r.returncode==4 and 'BLOCKED / MISSING_ARTIFACT' in r.stderr
    assert all(name in r.stderr for name in VIEWERS) and 'skipped' not in r.stdout
    pointers=tmp_path/'pointers'
    for name in VIEWERS:
        p=pointers/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('version https://git-lfs.github.com/spec/v1\noid sha256:'+'0'*64+'\nsize 123\n')
    r=invoke(pointers)
    assert r.returncode==4 and 'actual artifact missing' in r.stderr


def test_saved_viewers_can_use_an_independent_external_artifact_root(tmp_path):
    before={name:sha256((ARTIFACTS/name).read_bytes()).hexdigest() for name in VIEWERS}
    for name in VIEWERS:
        p=tmp_path/name;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ARTIFACTS/name,p)
    r=invoke(tmp_path)
    assert r.returncode==0,r.stderr
    assert '3 passed' in r.stdout
    assert before=={name:sha256((ARTIFACTS/name).read_bytes()).hexdigest() for name in VIEWERS}
