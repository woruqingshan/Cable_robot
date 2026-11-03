"""Streaming file identity helper; standard library only."""

from hashlib import sha256
from pathlib import Path


def file_sha256(path):
    digest = sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


