"""Explicit external resources; configuration is set before test collection."""
import os
from pathlib import Path

PROJECT_ROOT=Path(__file__).resolve().parents[1]
ARTIFACTS=Path(os.environ.get('WORKSPACE_ARTIFACT_ROOT',PROJECT_ROOT/'results')).expanduser().resolve()
SYSTEM_SOURCE=Path(os.environ.get('JETROVER_SYSTEM_SOURCE',PROJECT_ROOT.parents[1]/'system_source')).expanduser().resolve()
