"""Bind fresh baseline compilation to its frozen headers and core only."""
import hashlib
from pathlib import Path


def snapshot(root):
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted((root / 'navigation2d').rglob('*'))
            if path.is_file() and path.suffix in {'.h', '.hpp'}}


def prepare(source, include_root):
    root = Path(include_root).resolve() if include_root else (
        Path(source).resolve().parent.parent.parent if source else None)
    if root is None or not (root / 'navigation2d/application/navigation_system.h').is_file():
        raise ValueError('fresh baseline requires its complete frozen --baseline-include-root')
    return root, snapshot(root)


def baseline_link(command, node_object, core):
    # The original frozen node is one translation unit. Candidate-only ROS
    # adapters must never be linked against the old core or its private ABI.
    result = [arg for arg in command if not (arg.endswith('.o') and not arg.startswith('-'))
              or arg == str(node_object)]
    indices = [index for index, arg in enumerate(result) if arg.endswith('libnavigation2d_core.a')]
    if len(indices) != 1:
        raise ValueError('unexpected baseline core link contract')
    result[indices[0]] = str(core)
    return result
