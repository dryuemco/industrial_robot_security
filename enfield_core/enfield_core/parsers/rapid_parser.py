# Copyright 2026 Yunus Emre Cogurcu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ABB RAPID parser — Lark-based PoC for speed violation detection (A1)."""

from lark import Lark

RAPID_GRAMMAR = """
    start: command+
    command: "MoveL" target "," speed "," zone "," tool ";"

    target: CNAME
    speed: "v" NUMBER
    zone: CNAME
    tool: CNAME

    %import common.CNAME
    %import common.NUMBER
    %import common.WS
    %ignore WS
"""

parser = Lark(RAPID_GRAMMAR, start='start')


def check_safety(code_snippet, max_speed=1000):
    """Parse RAPID code and check for A1 speed violations."""
    violations = []
    tree = parser.parse(code_snippet)

    for speed_node in tree.find_data('speed'):
        speed_value = int(speed_node.children[0].value)
        if speed_value > max_speed:
            severity = (speed_value - max_speed) / max_speed
            violations.append({
                'attack_type': 'A1',
                'iso_clause': '5.6',
                'speed': speed_value,
                'max_speed': max_speed,
                'severity': round(severity, 2),
                'description': (
                    f'Speed {speed_value} mm/s exceeds '
                    f'limit {max_speed} mm/s (severity: {severity:.2f})'
                ),
            })
    return violations


if __name__ == '__main__':
    safe_code = 'MoveL pPick, v500, fine, tool0;'
    unsafe_code = 'MoveL pPlace, v5000, fine, tool0;'

    print('--- Safe code ---')
    result = check_safety(safe_code)
    print(f'  Violations: {len(result)}')

    print('--- Unsafe code ---')
    result = check_safety(unsafe_code)
    for v in result:
        print(f'  [{v["attack_type"]}] {v["description"]}')
