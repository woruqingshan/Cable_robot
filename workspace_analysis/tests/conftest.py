"""Resolve local sources from this file, independent of the shell CWD."""

from pathlib import Path
import sys
import os
import json
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))
_active_level=None


def _audit_history_access(event,args):
    if event!='open' or _active_level is None or not args or not isinstance(args[0],(str,bytes)):return
    from test_resources import ARTIFACTS
    path=Path(os.fsdecode(args[0])).resolve()
    if _active_level=='core' and 'jetrover_description' in path.parts:
        raise AssertionError('Undeclared official-model access from Core: '+str(path))
    roots=[ROOT/'results',ARTIFACTS]+[Path(p) for p in os.environ.get('R2_FORBIDDEN_RESULTS','').split(os.pathsep) if p]
    if any(path==p.resolve() or p.resolve() in path.parents for p in roots):
        native=os.environ.get('C1_BACKEND')
        if _active_level!='core' and native and path==Path(native).resolve():return
        if _active_level=='artifact' and (path==ARTIFACTS or ARTIFACTS in path.parents):return
        raise AssertionError('Implicit historical artifact read outside selected test contract: '+str(path))


sys.addaudithook(_audit_history_access)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_protocol(item,nextitem):
    global _active_level
    _active_level=item._repro_dependencies['level']
    try:yield
    finally:_active_level=None


def pytest_addoption(parser):
    parser.addoption('--artifact-root',default=None,help='Explicit historical results directory; missing inputs BLOCK with exit 4')
    parser.addoption('--native-backend',default=None,help='Explicit collision_backend executable')
    parser.addoption('--system-source',default=None,help='Official vendor source tree, separate from clone layout')


def pytest_configure(config):
    if config.getoption('--artifact-root'):os.environ['WORKSPACE_ARTIFACT_ROOT']=str(Path(config.getoption('--artifact-root')).resolve())
    if config.getoption('--native-backend'):os.environ['C1_BACKEND']=str(Path(config.getoption('--native-backend')).resolve())
    if config.getoption('--system-source'):os.environ['JETROVER_SYSTEM_SOURCE']=str(Path(config.getoption('--system-source')).resolve())


@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config,items):
    declared=json.loads((ROOT/'tests/dependencies.json').read_text())['tests']
    for item in items:
        name=item.originalname or item.name.split('[')[0]
        key=Path(str(item.fspath)).relative_to(ROOT/'tests').as_posix()+'::'+name
        row=declared.get(key)
        if row is None:raise pytest.UsageError('Undeclared test dependency: '+key)
        item.add_marker(row['level']);item._repro_dependencies=row


def pytest_collection_finish(session):
    from test_resources import ARTIFACTS,SYSTEM_SOURCE
    missing=set();requires=set();bindings=set()
    for item in session.items:
        row=item._repro_dependencies;requires.update(row['requires'])
        for name in row['artifacts']:
            p=ARTIFACTS/name
            if not p.exists():missing.add(str(p));continue
            if p.is_file():
                with p.open('rb') as stream:prefix=stream.read(128)
                if prefix.startswith(b'version https://git-lfs.github.com/spec/v1'):
                    missing.add(str(p)+' (Git LFS pointer; actual artifact missing)')
            # Manifests bind every output byte. Missing saved-run components are
            # reported before executing any test, never substituted or skipped.
            if p.is_dir() and (p/'manifest.json').is_file():
                try:data=json.loads((p/'manifest.json').read_text())
                except ValueError as exc:raise pytest.UsageError('BLOCKED / INVALID_ARTIFACT_MANIFEST: '+str(p/'manifest.json')+': '+str(exc))
                for rel in data.get('files',{}):
                    if not (p/rel).is_file():missing.add(str(p/rel))
                for field in ['input_prefix','seed_prefix','ik_run','path_run']:
                    value=data.get(field)
                    if isinstance(value,str) and Path(value).is_absolute() and not Path(value).resolve().is_relative_to(ARTIFACTS):
                        bindings.add(str(p/'manifest.json')+' -> '+field+'='+value)
    if missing:raise pytest.UsageError('BLOCKED / MISSING_ARTIFACT:\n'+'\n'.join(sorted(missing)))
    if bindings:raise pytest.UsageError('BLOCKED / ARTIFACT_PATH_BINDING: historical absolute references need compatible restoration; no silent rebinding\n'+'\n'.join(sorted(bindings)))
    if 'native' in requires:
        backend=Path(os.environ.get('C1_BACKEND','/not/configured'))
        if not backend.is_file() or not os.access(backend,os.X_OK):raise pytest.UsageError('BLOCKED / MISSING_NATIVE_BACKEND: set --native-backend or C1_BACKEND')
    if 'model' in requires:
        try:
            from ament_index_python.packages import get_package_share_directory
            share=Path(get_package_share_directory('jetrover_description'))
            if not (share/'urdf/jetrover.xacro').is_file():raise FileNotFoundError(str(share))
        except Exception as exc:raise pytest.UsageError('BLOCKED / MISSING_MODEL: '+str(exc))
    if 'vendor' in requires and not SYSTEM_SOURCE.is_dir():raise pytest.UsageError('BLOCKED / MISSING_VENDOR_SOURCE: '+str(SYSTEM_SOURCE))
