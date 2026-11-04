"""Explicit source dependency, not a copied research package or implicit CWD path."""
from pathlib import Path
import sys
from .errors import ReplayBlocked, ReplayError


def use_workspace(workspace_root: str | Path) -> Path:
    """Bind imports to the caller-declared repository and reject another checkout."""
    root = Path(workspace_root).expanduser().resolve(strict=True)
    if not (root/'src/workspace_analysis/robot/model.py').is_file():
        raise ReplayBlocked('MISSING_RESEARCH_SOURCE', str(root))
    loaded = sys.modules.get('workspace_analysis')
    if loaded is not None and not Path(loaded.__file__).resolve().is_relative_to(root/'src'):
        raise ReplayError('SOURCE_IDENTITY', 'workspace_analysis imported from another checkout')
    for path in [str(root), str(root/'src')]:
        if path not in sys.path: sys.path.insert(0, path)
    return root
