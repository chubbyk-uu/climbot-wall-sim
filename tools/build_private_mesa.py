#!/usr/bin/env python3
"""Build pinned Ubuntu Mesa in a fresh private directory; never install system packages."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
from urllib.parse import unquote
from urllib.request import urlopen

REPO = Path(__file__).resolve().parents[1]


def verify(path, expected):
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError(f'SHA256 mismatch: {path.name}')


def build(args):
    release = platform.freedesktop_os_release()
    if (release.get('ID'), release.get('VERSION_ID'), platform.machine()) != ('ubuntu', '24.04', 'x86_64'):
        raise ValueError('This recipe supports Ubuntu 24.04 x86_64 only (WSL D3D12 runtime)')
    root = args.root.resolve()
    if any(c.isspace() or c == ':' for c in str(root)):
        raise ValueError('Build root cannot contain whitespace or colon')
    if root.exists():
        raise ValueError('Build root already exists; choose a fresh --root (no automatic deletion)')
    if args.jobs < 1:
        raise ValueError('--jobs must be positive')
    for command in ('g++', 'ninja', 'pkg-config', 'dpkg-source', 'dpkg-deb', 'apt-get', 'patch'):
        if not shutil.which(command):
            raise ValueError(f'Missing prerequisite: {command}; see docs/MESA_SETUP.md')
    lock = json.loads((REPO / 'tools/patches/mesa-build-lock.json').read_text())
    patch = REPO / lock['project_patch']
    verify(patch, lock['project_patch_sha256'])
    root.mkdir(parents=True)
    # Meson's cmake probes leave two directories both named MesonTemp under
    # build/meson-private. colcon discovers them as packages and refuses the
    # whole workspace with a duplicate-name error, so a plain "colcon test"
    # never starts. Mark the tree before anything is written into it.
    (root / 'COLCON_IGNORE').touch()
    for folder in ('downloads', 'deps', 'sysroot'):
        (root / folder).mkdir()
    print(f'Building private Mesa; complete log: {root / "build.log"}', flush=True)
    with (root / 'build.log').open('w') as log:
        def run(command, cwd=root, env=None):
            subprocess.run(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)

        def cached(folder, name, digest):
            target = root / folder / name
            source = args.cache / folder / name if args.cache else None
            if source and source.is_file():
                shutil.copyfile(source, target)
                verify(target, digest)
            return target

        for name, digest in lock['downloads'].items():
            target = cached('downloads', name, digest)
            if not target.exists():
                remote = f"mesa_{lock['ubuntu_source_version']}.dsc" if name == 'mesa.dsc' else name
                # urllib honors standard proxy environment variables; never persist their values.
                with urlopen(lock['source_base_url'] + remote, timeout=120) as response, target.open('wb') as out:
                    shutil.copyfileobj(response, out)
            verify(target, digest)
        for name, digest in lock['dependency_packages'].items():
            target = cached('deps', name, digest)
            if not target.exists():
                package, version, _ = unquote(name).rsplit('_', 2)
                run(['apt-get', 'download', f'{package}={version}'], cwd=root / 'deps')
            verify(target, digest)
            run(['dpkg-deb', '-x', str(target), str(root / 'sysroot')])
        # Dev debs contain relative .so links, but runtime libs belong to the host.
        # Resolve these explicitly so Meson cannot silently choose static archives.
        for link in (root / 'sysroot/usr/lib/x86_64-linux-gnu').glob('*.so'):
            if link.is_symlink() and not link.exists():
                host = Path('/usr/lib/x86_64-linux-gnu') / os.readlink(link)
                if not host.is_file():
                    # Some unpacked dev packages are optional for this backend.
                    # Meson will diagnose missing required dependencies.
                    continue
                link.unlink()
                link.symlink_to(host)
        for pc in (root / 'sysroot').rglob('*.pc'):
            pc.write_text(pc.read_text().replace('prefix=/usr', f'prefix={root}/sysroot/usr'))
        run(['dpkg-source', '--no-check', '-x', str(root / 'downloads/mesa.dsc'), str(root / 'source')])
        run(['patch', '--batch', '--fuzz=0', '-p1', '-i', str(patch)], cwd=root / 'source')
        env = dict(os.environ)
        private = root / 'sysroot/usr'
        env.update(PATH=f'{private}/bin:' + env['PATH'],
                   PYTHONPATH=f'{private}/lib/python3/dist-packages:{private}/lib/python3.12/site-packages',
                   PKG_CONFIG_PATH=f'{private}/lib/x86_64-linux-gnu/pkgconfig:{private}/share/pkgconfig',
                   CFLAGS=f'-I{private}/include', CXXFLAGS=f'-I{private}/include',
                   BISON_PKGDATADIR=f'{private}/share/bison')
        run([str(private / 'bin/meson'), 'setup', str(root / 'build'), str(root / 'source'),
             f'--prefix={root}/install', *lock['configuration']], env=env)
        run(['ninja', '-C', str(root / 'build'), f'-j{args.jobs}'], env=env)
        run([str(private / 'bin/meson'), 'install', '-C', str(root / 'build'), '--no-rebuild'], env=env)
        libraries = list((root / 'install/lib').glob('*.so*'))
        if not libraries:
            raise ValueError('No installed shared libraries')
        check_env = dict(os.environ)
        check_env['LD_LIBRARY_PATH'] = str(root / 'install/lib')
        for library in libraries:
            result = subprocess.run(['ldd', str(library)], capture_output=True, text=True, env=check_env)
            log.write(result.stdout + result.stderr)
            if result.returncode or 'not found' in result.stdout:
                raise ValueError(f'Unresolved library dependency: {library.name}')
        (root / 'build-result.json').write_text(json.dumps({
            'source_version': lock['ubuntu_source_version'],
            'libraries_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in libraries},
            'runtime_tested': False}, indent=2) + '\n')
    print(f'Build and dependency checks passed. Prefix: {root / "install"}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    # Outside the data root: this shared graphics runtime is not disposable data.
    parser.add_argument('--root', type=Path, default=Path.home() / 'opt' / 'agv-mesa-25.2.8',
                        help='Fresh build directory (default ~/opt/agv-mesa-25.2.8); prefix is <root>/install')
    parser.add_argument('--cache', type=Path, help='Optional prior downloads/ and deps/ directory; hashes always checked')
    parser.add_argument('--jobs', type=int, default=8)
    args = parser.parse_args()
    try:
        build(args)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'Build failed: {error}; inspect build.log if created', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
