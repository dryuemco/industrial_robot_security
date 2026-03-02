"""ABB RAPID parser — Lark-based PoC for speed violation detection (A1)."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
# Original: scripts/rapid_parser.py

from lark import Lark

rapid_grammar = """
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

parser = Lark(rapid_grammar, start='start')


def check_safety(code_snippet: str, max_speed: int = 1000) -> list[dict]:
    """Parse RAPID code and check for A1 speed violations.

    Args:
        code_snippet: RAPID code string containing MoveL commands.
        max_speed: Maximum allowed TCP speed in mm/s.

    Returns:
        List of violation dicts with keys: line, speed, max_speed, severity.
    """
    violations: list[dict] = []
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
