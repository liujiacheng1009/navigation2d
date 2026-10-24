"""Build a single-source contract driver against current and frozen core APIs."""
import hashlib
from pathlib import Path
import shlex
import subprocess
from replay_baseline_headers import prepare, snapshot, baseline_link
from replay_object_cache import compile_cached


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_probe(build, target_name, source, headers, baseline_core, output):
    build, source = Path(build).resolve(strict=True), Path(source).resolve(strict=True)
    baseline_core, output = Path(baseline_core).resolve(strict=True), Path(output).resolve()
    current_core = build / 'libnavigation2d_core.a'
    if current_core.exists() and baseline_core.samefile(current_core):
        raise ValueError('baseline core must be independently frozen')
    header_root, header_hashes = prepare(None, headers)
    output.mkdir(parents=True, exist_ok=True)
    project_build = next((p for p in [build, *build.parents]
                          if (p / 'CMakeCache.txt').is_file()), None)
    if project_build is None:
        raise ValueError('build directory is not in a configured CMake tree')
    subprocess.run(['cmake', '--build', str(project_build), '--target', target_name,
                    '--parallel', '2'], check=True)
    target = build / ('test/CMakeFiles/' + target_name + '.dir')
    flags = []
    for line in (target / 'flags.make').read_text().splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            flags.extend(shlex.split(line.split('=', 1)[1]))
    template = shlex.split((target / 'link.txt').read_text())
    obj, binary = output / 'baseline.o', output / 'baseline'
    compilation = compile_cached(template[0], ['-I' + str(header_root), *flags], source, obj)
    objects = [i for i, token in enumerate(template) if token.endswith('.o')]
    if len(objects) != 1:
        raise ValueError('contract driver must have exactly one source object')
    template[objects[0]] = str(obj)
    template[template.index('-o') + 1] = str(binary)
    link = baseline_link(template, obj, baseline_core)
    subprocess.run(link, cwd=build / 'test', check=True)
    if snapshot(header_root) != header_hashes:
        raise RuntimeError('frozen headers changed while compiling')
    candidate = build / 'test' / target_name
    provenance = {'baseline_core_sha256': digest(baseline_core),
                  'candidate_core_sha256': digest(current_core),
                  'baseline_binary_sha256': digest(binary),
                  'candidate_binary_sha256': digest(candidate),
                  'source_sha256': digest(source), 'baseline_headers': header_hashes,
                  'baseline_compilation': compilation, 'baseline_link': link}
    return binary, candidate, provenance
