"""Explicitly download and record the checkpoints used by preview-sam."""

import hashlib
import json
from pathlib import Path
from tempfile import NamedTemporaryFile

import requests
from clip.clip import _MODELS


ROOT = Path('/sim/runtime/models')
RELEASE = 'v8.3.0'
ASSETS = f'https://github.com/ultralytics/assets/releases/download/{RELEASE}'


def prepare(path, url, version, size, expected_sha=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file():
        with path.open('rb') as source:
            digest = hashlib.sha256()
            for block in iter(lambda: source.read(1024 * 1024), b''):
                digest.update(block)
        sha = digest.hexdigest()
        if path.stat().st_size == size and (expected_sha is None or sha == expected_sha):
            return dict(path=str(path), url=url, version=version, bytes=size, sha256=sha)
        raise ValueError(f'Invalid checkpoint {path}. Remove it and rerun prepare-sam.')

    print(f'Downloading {path.name} from {url}', flush=True)
    digest = hashlib.sha256()
    with NamedTemporaryFile(dir=path.parent, prefix=path.name + '.', suffix='.part', delete=False) as output:
        temporary = Path(output.name)
        try:
            with requests.get(url, stream=True, timeout=(15, 90)) as response:
                response.raise_for_status()
                for block in response.iter_content(1024 * 1024):
                    output.write(block)
                    digest.update(block)
            output.flush()
            sha = digest.hexdigest()
            if temporary.stat().st_size != size or (expected_sha is not None and sha != expected_sha):
                raise ValueError(f'Size or SHA-256 mismatch for {url}')
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
    return dict(path=str(path), url=url, version=version, bytes=size, sha256=sha)


def main():
    clip_url = _MODELS['ViT-B/32']
    clip_lock = json.loads(Path('/sim/agent/deployment.lock.json').read_text())['clip']
    resources = [
        prepare(ROOT / 'yolov8l-world.pt', f'{ASSETS}/yolov8l-world.pt', RELEASE, 95667042),
        prepare(ROOT / 'mobile_sam.pt', f'{ASSETS}/mobile_sam.pt', RELEASE, 40728226),
        prepare(Path.home() / '.cache/clip/ViT-B-32.pt', clip_url, 'ViT-B/32',
                353976522, clip_url.split('/')[-2]),
    ]
    manifest = ROOT / 'manifest.json'
    manifest.write_text(json.dumps(dict(clip_package=clip_lock, checkpoints=resources), indent=2) + '\n')
    print(manifest)


if __name__ == '__main__':
    main()
