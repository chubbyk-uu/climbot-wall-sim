"""Keep committed project material free of machine-specific home directories."""

from pathlib import Path
import subprocess


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
FORBIDDEN_PATH_MARKERS = (
    b'/' + b'home/',
    b'/' + b'Users/',
    b'\\' + b'Users\\',
)


def test_tracked_files_do_not_embed_private_home_paths():
    tracked = subprocess.check_output(
        ['git', '-C', str(REPOSITORY_ROOT), 'ls-files', '-z'])
    offenders = []
    for raw_path in tracked.split(b'\0'):
        if not raw_path:
            continue
        path = REPOSITORY_ROOT / raw_path.decode('utf-8')
        if not path.is_file():
            continue
        contents = path.read_bytes()
        if any(marker in contents for marker in FORBIDDEN_PATH_MARKERS):
            offenders.append(str(raw_path, 'utf-8'))
    assert not offenders, f'machine-specific home paths found in: {offenders}'
