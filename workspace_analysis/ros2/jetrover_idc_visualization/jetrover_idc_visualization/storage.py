"""Declared local input roots, strict paths/JSON and no-follow artifact access."""
from hashlib import sha256
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
from .errors import ReplayError, ReplayBlocked


def json_object(raw: bytes | str) -> dict:
    def pairs(items):
        out = {}
        for key, value in items:
            if key in out: raise ReplayError('INVALID_JSON', 'duplicate key: '+key)
            out[key] = value
        return out
    def reject(value): raise ReplayError('INVALID_JSON', 'nonfinite constant: '+value)
    try:
        value = json.loads(raw, object_pairs_hook=pairs, parse_constant=reject)
    except (ValueError, UnicodeError) as exc:
        if isinstance(exc, ReplayError): raise
        raise ReplayError('INVALID_JSON', str(exc)) from exc
    if not isinstance(value, dict): raise ReplayError('INVALID_JSON', 'object required')
    return value


def relative_name(name: str) -> str:
    if not isinstance(name, str) or not name or '\\' in name or '\0' in name:
        raise ReplayError('UNTRUSTED_PATH', repr(name))
    p = PurePosixPath(name)
    if p.is_absolute() or name != p.as_posix() or any(x in ('', '.', '..') for x in name.split('/')):
        raise ReplayError('UNTRUSTED_PATH', name)
    return name


def sha(raw: bytes) -> str:
    return sha256(raw).hexdigest()


class ReadOnlyStore:
    """No symlinks in roots/components; openat/O_NOFOLLOW prevents redirection."""
    def __init__(self, root: str | Path):
        root = Path(root).expanduser().absolute()
        for part in [*reversed(root.parents), root]:
            if part.is_symlink(): raise ReplayError('UNTRUSTED_PATH', 'symlink root: '+str(part))
        if not root.is_dir(): raise ReplayBlocked('MISSING_ARTIFACT', str(root))
        self.root = root

    def path(self, name: str) -> Path:
        relative_name(name)
        path = self.root
        for part in name.split('/'):
            path = path/part
            if path.is_symlink(): raise ReplayError('UNTRUSTED_PATH', 'symlink: '+str(path))
        if not path.is_file(): raise ReplayBlocked('MISSING_ARTIFACT', str(path))
        return path

    def read(self, name: str, *, digest: str | None = None, limit: int = 256*1024*1024) -> bytes:
        relative_name(name)
        fd = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            parts = name.split('/')
            for component in parts[:-1]:
                next_fd = os.open(component, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=fd)
                os.close(fd); fd = next_fd
            file_fd = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=fd)
            with os.fdopen(file_fd, 'rb') as stream:
                before = os.fstat(stream.fileno())
                if not stat.S_ISREG(before.st_mode) or before.st_size > limit:
                    raise ReplayError('INVALID_ARTIFACT', 'not a bounded regular file: '+name)
                raw = stream.read(limit+1)
                after = os.fstat(stream.fileno())
                if len(raw) > limit or (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                    raise ReplayError('ARTIFACT_CHANGED', name)
        except FileNotFoundError as exc:
            raise ReplayBlocked('MISSING_ARTIFACT', str(self.root/name)) from exc
        except OSError as exc:
            raise ReplayError('UNTRUSTED_PATH', name+': '+str(exc)) from exc
        finally: os.close(fd)
        if raw.startswith(b'version https://git-lfs.github.com/spec/v1'):
            raise ReplayBlocked('MISSING_ARTIFACT', name+' is a Git LFS pointer')
        if digest is not None and (not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest) or sha(raw) != digest):
            raise ReplayError('ARTIFACT_HASH', name)
        return raw

    def object(self, name: str, *, digest: str | None = None) -> dict:
        return json_object(self.read(name, digest=digest))
