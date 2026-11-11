"""Read the existing P2 model without importing protocol/HTTP/GPU libraries.

ROS launch normally runs under the system ROS interpreter; its model-description
loading must not require the separate protocol environment's Pydantic install.
"""
from pathlib import Path
import sys


def load_description_model(workspace_root, config_path=None):
    root = Path(workspace_root).expanduser().resolve()
    if not (root / "src/workspace_analysis/robot/model.py").is_file():
        raise ValueError("explicit existing workspace_analysis source root required")
    source = str(root / "src")
    if source not in sys.path:
        sys.path.insert(0, source)
    from workspace_analysis.robot.model import load_model
    import workspace_analysis
    if Path(workspace_analysis.__file__).resolve().parent != root / "src/workspace_analysis":
        raise ValueError("another P2 source tree is already imported")
    return load_model(config_path or root / "config/workspace.yaml")
