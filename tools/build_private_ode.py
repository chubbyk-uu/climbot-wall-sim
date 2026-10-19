#!/usr/bin/env python3
"""Build a pinned, local ODE with the cross-zero hash-space correction.

No system library is replaced. Full build output stays in the prefix's log.
The upstream variant exists solely for same-build A/B diagnosis.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import urllib.request

URL = 'https://codeload.github.com/odedevs/ode/tar.gz/refs/tags/0.16.2'
SOURCE_SHA256 = '28b547ac9c3f2c8ba4c23a9cbe579667447616876c61b507a184f8607143050e'
PATCH = Path(__file__).resolve().parent / 'patches/ode-0.16.2-hashspace-overflow.patch'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--archive', type=Path)
    parser.add_argument('--variant', choices=['hashfix', 'upstream'], default='hashfix')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    prefix = args.prefix.expanduser().resolve()
    if prefix.exists():
        parser.error('prefix exists; preserve previous build and manifest')
    if not 1 <= args.jobs <= 16:
        parser.error('jobs must be in 1..16')
    archive = (args.archive.expanduser().resolve() if args.archive else
               Path.home() / '.cache/climbot/ode-0.16.2.tar.gz')
    if not archive.exists():
        archive.parent.mkdir(parents=True, exist_ok=True)
        temporary = archive.with_suffix('.download')
        with urllib.request.urlopen(URL, timeout=60) as response, temporary.open('wb') as output:
            shutil.copyfileobj(response, output)
        if digest(temporary) != SOURCE_SHA256:
            raise ValueError('ODE source download checksum mismatch')
        temporary.rename(archive)
    if digest(archive) != SOURCE_SHA256:
        raise ValueError('ODE source archive checksum mismatch')
    prefix.mkdir(parents=True)
    source, build, install = [prefix / name for name in ['source', 'build', 'install']]
    source.mkdir()
    with tarfile.open(archive) as stream:
        stream.extractall(source, filter='data')
    source = source / 'ode-0.16.2'
    log = prefix / 'build.log'
    options = ['-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_INSTALL_LIBDIR=lib',
               '-DODE_DOUBLE_PRECISION=ON', '-DODE_WITH_DEMOS=OFF', '-DODE_WITH_TESTS=OFF',
               '-DODE_WITH_LIBCCD=ON', '-DODE_WITH_LIBCCD_SYSTEM=ON',
               '-DODE_WITH_LIBCCD_BOX_CYL=OFF',
               '-DBUILD_SHARED_LIBS=ON',
               '-DCMAKE_INSTALL_PREFIX=' + str(install)]
    with log.open('w') as output:
        def run(command):
            subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=True)
        if args.variant == 'hashfix':
            run(['patch', '-p1', '--directory', str(source), '--input', str(PATCH)])
        # Upstream 0.16.2 CMake has a lowercase target typo in the system-ccd branch.
        cmake_file = source / 'CMakeLists.txt'
        cmake_file.write_text(cmake_file.read_text().replace(
            'target_compile_definitions(ode PRIVATE', 'target_compile_definitions(ODE PRIVATE')
            .replace('find_package(ccd)', 'find_package(ccd REQUIRED)')
            .replace('target_link_libraries(ODE ccd::ccd)', 'target_link_libraries(ODE ccd)'))
        # Match Ubuntu's SONAME; both A/B builds receive the same packaging setting.
        with (source / 'CMakeLists.txt').open('a') as cmake:
            cmake.write('\nset_target_properties(ODE PROPERTIES SOVERSION 8 VERSION 8.0.2)\n'
                        'target_compile_options(ODE PRIVATE -UdNODEBUG)\n'
                        'target_include_directories(ODE PRIVATE '
                        '${CMAKE_CURRENT_SOURCE_DIR}/libccd/src/custom)\n')
        run(['cmake', '-S', str(source), '-B', str(build), *options])
        run(['cmake', '--build', str(build), '--parallel', str(args.jobs)])
        run(['cmake', '--install', str(build)])
    library = install / 'lib/libode.so.8.0.2'
    probe_source = Path(__file__).with_name('ode_hash_probe.cpp')
    probe = prefix / 'ode_hash_probe'
    regression = prefix / 'hash_regression.json'
    with log.open('a') as output:
        subprocess.run(['g++', '-std=c++17', str(probe_source),
                        '-I' + str(install / 'include'), '-L' + str(install / 'lib'),
                        '-Wl,-rpath,' + str(install / 'lib'), '-lode', '-o', str(probe)],
                       stdout=output, stderr=subprocess.STDOUT, check=True)
    with regression.open('w') as output:
        subprocess.run([str(probe), *(['--diagnostic'] if args.variant == 'upstream' else [])],
                       stdout=output, check=True)
    regression_result = json.loads(regression.read_text())
    manifest = {'schema_version': 1, 'version': '0.16.2', 'variant': args.variant,
                'source_url': URL, 'source_sha256': SOURCE_SHA256,
                'patch_sha256': digest(PATCH) if args.variant == 'hashfix' else None,
                'cmake_options': options, 'library_sha256': digest(library),
                'build_profile': 'ubuntu-noble-system-ccd-assertions',
                'packaging_compile_options': ['-UdNODEBUG'],
                'packaging_cmake_fixes': ['system-libccd-target-case', 'libccd-custom-header-path',
                                          'ubuntu-libccd-exported-target'],
                'soname': 'libode.so.8', 'install': str(install), 'log': str(log),
                'regression_passed': regression_result['passed'],
                'regression_source_sha256': digest(probe_source),
                'regression_sha256': digest(regression)}
    dependencies = subprocess.check_output(['ldd', str(library)], text=True)
    if 'libccd.so.2 =>' not in dependencies:
        raise RuntimeError('private ODE must link system libccd.so.2')
    config = subprocess.check_output([
        'python3', '-c', 'import ctypes,sys; l=ctypes.CDLL(sys.argv[1]); '
        'l.dGetConfiguration.restype=ctypes.c_char_p; print(l.dGetConfiguration().decode())',
        str(library)], text=True).strip()
    manifest['ode_configuration'] = config
    manifest['linked_dependencies'] = dependencies.splitlines()
    (prefix / 'build-result.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'variant': args.variant, 'install': str(install), 'log': str(log)}))


if __name__ == '__main__':
    main()
