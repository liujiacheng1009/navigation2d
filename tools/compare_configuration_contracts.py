#!/usr/bin/env python3
"""Compare every legacy config field across all built-in selection pairs."""
import argparse
import json
from pathlib import Path
import re
import subprocess
from frozen_core_probe import build_probe, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', required=True, type=Path)
    parser.add_argument('--baseline-headers', required=True, type=Path)
    parser.add_argument('--baseline-core', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    baseline, candidate, provenance = build_probe(
        args.build, 'navigation2d_config_replay', root / 'test/navigation_config_contract.cc',
        args.baseline_headers, args.baseline_core, output)
    original = (root / 'config/navigation2d.yaml').read_text()
    results = []
    for global_id in ['dijkstra', 'astar', 'theta_star', 'state_lattice']:
        for local_id in ['rpp', 'dwa', 'mppi', 'mpc']:
            yaml, replaced_global = re.subn(r'(?m)^  planner: .*$',
                                            '  planner: ' + global_id, original)
            yaml, replaced_local = re.subn(r'(?m)^  controller: .*$',
                                           '  controller: ' + local_id, yaml)
            if replaced_global != 1 or replaced_local != 1:
                raise ValueError('unsupported canonical selection layout')
            case = global_id + '-' + local_id
            path = output / (case + '.yaml')
            path.write_text(yaml)
            captured = {}
            for label, binary in [('baseline', baseline), ('candidate', candidate)]:
                text = subprocess.check_output([str(binary), str(path)], text=True)
                (output / (case + '-' + label + '.json')).write_text(text)
                captured[label] = text
                fields = json.loads(text)
                if len(fields) != 110 or fields['planner'] != global_id or fields['controller'] != local_id:
                    raise ValueError('configuration field/scenario capture is incomplete')
            results.append({'case': case, 'equal': captured['baseline'] == captured['candidate'],
                            'input_sha256': digest(path)})
    equal = all(row['equal'] for row in results)
    (output / 'comparison.json').write_text(json.dumps(
        {'equal': equal, 'cases': results, **provenance}, indent=2) + '\n')
    print('16 selections, 109 scalar fields + footprint:', 'equal' if equal else 'DIFFERENT')
    if not equal:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
