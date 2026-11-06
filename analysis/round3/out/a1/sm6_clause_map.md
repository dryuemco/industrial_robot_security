### Code (enfield_watchdog_static/rules/sm*.py): iso_clause and cwe per rule
| rule file | line | text |
|---|---|---|
| sm1_input_validation.py | 51 | `iso_clause="5.1.16",` |
| sm1_input_validation.py | 60 | `"cwe": "CWE-20",` |
| sm1_input_validation.py | 62 | `"missing": "speed",` |
| sm1_input_validation.py | 70 | `iso_clause="5.1.16",` |
| sm1_input_validation.py | 79 | `"cwe": "CWE-20",` |
| sm1_input_validation.py | 81 | `"missing": "acceleration",` |
| sm2_unchecked_return.py | 51 | `iso_clause="none",` |
| sm2_unchecked_return.py | 59 | `metadata={"cwe": "CWE-252"},` |
| sm2_unchecked_return.py | 81 | `iso_clause="none",` |
| sm2_unchecked_return.py | 90 | `"cwe": "CWE-252",` |
| sm3_protection_failure.py | 46 | `iso_clause="5.1.16",` |
| sm3_protection_failure.py | 52 | `"cwe": "CWE-693",` |
| sm4_unusual_conditions.py | 52 | `iso_clause="none",` |
| sm4_unusual_conditions.py | 60 | `metadata={"cwe": "CWE-754"},` |
| sm5_hardcoded_values.py | 56 | `iso_clause="5.1.16",` |
| sm5_hardcoded_values.py | 68 | `"cwe": "CWE-798",` |
| sm5_hardcoded_values.py | 81 | `iso_clause="5.1.16",` |
| sm5_hardcoded_values.py | 92 | `"cwe": "CWE-798",` |
| sm6_missing_preamble.py | 59 | `iso_clause="5.1.14",` |
| sm6_missing_preamble.py | 68 | `"cwe": "robot-specific",` |
| sm6_missing_preamble.py | 69 | `"missing": "set_tcp",` |
| sm6_missing_preamble.py | 77 | `iso_clause="5.1.15",` |
| sm6_missing_preamble.py | 87 | `"cwe": "robot-specific",` |
| sm6_missing_preamble.py | 88 | `"missing": "set_payload",` |
| sm7_prompt_marker.py | 56 | `iso_clause="5.1.16",` |
| sm7_prompt_marker.py | 62 | `"cwe": "robot-specific",` |

### Table 18 (paper/main_access.tex, tab:iso)
| line | text |
|---|---|
| 1915 | `DM-1 & Speed value-range & 5.5.3 \\` |
| 1916 | `DM-1 & Payload mass and centre of gravity & 5.1.15 \\` |
| 1917 | `DM-2 & Safe-zone halfspace test & 5.7.4 \\` |
| 1918 | `DM-3 & Orientation-cone test & 5.7.4 \\` |
| 1919 | `DM-4 & Protective-stop presence & 5.4.2 \\` |
| 1920 | `DM-4 & Safety-logic completeness & 5.4 \\` |
| 1921 | `DM-5 & Frame consistency & 5.7.4 \\` |
| 1922 | `DM-5 & Cross-frame deviation & 5.7.4 \\` |
| 1923 | `DM-6 & Tool identity & 5.1.14 \\` |
| 1924 | `DM-6 & Tool activation mode & 5.1.14 \\` |
| 1925 | `DM-7 & Configuration integrity & 5.1.16 \\` |
| 1926 | `SM-1 & Input-validation proxy (rel. CWE-20) & 5.1.16 \\` |
| 1927 | `SM-2 & Error-handling proxy (rel. CWE-252) & none \\` |
| 1928 | `SM-3 & Protection mechanism (CWE-693) & 5.1.16 \\` |
| 1929 | `SM-4 & Unusual conditions (CWE-754) & none \\` |
| 1930 | `SM-5 & Hardcoded value (CWE-798) & 5.1.16 \\` |
| 1931 | `SM-6 & Tool-centre-point preamble & 5.1.14 \\` |
| 1932 | `SM-6 & Payload preamble & 5.1.15 \\` |
| 1933 | `SM-7 & Prompt-injection marker & 5.1.16 \\` |

### docs/iso_10218_traceability.csv
| line | text |
|---|---|
| 1 | `rule_id,rule_type,attack_id,short_name,iso_clause,clause_title,source_file,test_file` |
| 2 | `DM-1,safety,A1,Speed value-range check,5.5.3,Speed limit(s) monitoring,enfield_watchdog_static/rules/a1_speed.py,enfield_watchdog_static/test/test_a1_speed.py` |
| 3 | `DM-1,safety,A4,Payload mass/CoG bounds,5.1.15,Payload setting,enfield_watchdog_static/rules/a4_payload.py,enfield_watchdog_static/test/test_a4_payload.py` |
| 4 | `DM-2,safety,A2,Safe-zone halfspace test,5.7.4,Software-based limiting,enfield_watchdog_static/rules/a2_zone.py,enfield_watchdog_static/test/test_a2_zone.py` |
| 5 | `DM-3,safety,A3,Orientation cone test,5.7.4,Software-based limiting,enfield_watchdog_static/rules/a3_orientation.py,enfield_watchdog_static/test/test_a3_orientation.py` |
| 6 | `DM-4,safety,A5,E-Stop command presence in motion sequence,5.4.2,Emergency stop,enfield_watchdog_static/rules/a5_logic.py,enfield_watchdog_static/test/test_a5_logic.py` |
| 7 | `DM-4,safety,A5,Safety-logic structural completeness,5.4,Stopping functions,enfield_watchdog_static/rules/a5_logic.py,enfield_watchdog_static/test/test_a5_logic.py` |
| 8 | `DM-5,safety,A6,Frame consistency check,5.7.4,Software-based limiting,enfield_watchdog_static/rules/a6_frame.py,enfield_watchdog_static/test/test_a6_frame.py` |
| 9 | `DM-5,safety,A6,Frame cross-frame deviation,5.7.4,Software-based limiting,enfield_watchdog_static/rules/a6_frame.py,enfield_watchdog_static/test/test_a6_frame.py` |
| 10 | `DM-6,safety,A7,Tool identity check,5.1.14,TCP setting,enfield_watchdog_static/rules/a7_tool.py,enfield_watchdog_static/test/test_a7_tool.py` |
| 11 | `DM-6,safety,A7,Tool activation mode,5.1.14,TCP setting,enfield_watchdog_static/rules/a7_tool.py,enfield_watchdog_static/test/test_a7_tool.py` |
| 12 | `DM-7,safety,A8,Prompt-security config integrity,5.1.16,Cybersecurity,enfield_watchdog_static/rules/a8_prompt.py,enfield_watchdog_static/test/test_a8_prompt.py` |
| 13 | `SM-1,security (CWE-20),—,Improper input validation — missing v=/a= on motion calls,5.1.16,Cybersecurity,enfield_watchdog_static/rules/sm1_input_validation.py,enfield_watchdog_static/test/test_sm1_input_validation.py` |
| 14 | `SM-2,security (CWE-252),—,Unchecked return — no error handling near critical ops,—,(CWE-only; no ISO anchor),enfield_watchdog_static/rules/sm2_unchecked_return.py,enfield_watchdog_static/test/test_sm2_unchecked_return.py` |
| 15 | `SM-3,security (CWE-693),—,Protection mechanism failure — interlock bypass,5.1.16,Cybersecurity,enfield_watchdog_static/rules/sm3_protection_failure.py,enfield_watchdog_static/test/test_sm3_protection_failure.py` |
| 16 | `SM-4,security (CWE-754),—,Improper check for unusual conditions,—,(CWE-only; no ISO anchor),enfield_watchdog_static/rules/sm4_unusual_conditions.py,enfield_watchdog_static/test/test_sm4_unusual_conditions.py` |
| 17 | `SM-5,security (CWE-798),—,Hardcoded unsafe parameters (speed/accel),5.1.16,Cybersecurity,enfield_watchdog_static/rules/sm5_hardcoded_values.py,enfield_watchdog_static/test/test_sm5_hardcoded_values.py` |
| 18 | `SM-6a,security,—,Missing set_tcp preamble before motion,5.1.14,TCP setting,enfield_watchdog_static/rules/sm6_missing_preamble.py,enfield_watchdog_static/test/test_sm6_missing_preamble.py` |
| 19 | `SM-6b,security,—,Missing set_payload preamble before motion,5.1.15,Payload setting,enfield_watchdog_static/rules/sm6_missing_preamble.py,enfield_watchdog_static/test/test_sm6_missing_preamble.py` |
| 20 | `SM-7,security,—,Prompt injection marker leaked into generated code,5.1.16,Cybersecurity,enfield_watchdog_static/rules/sm7_prompt_marker.py,enfield_watchdog_static/test/test_sm7_prompt_marker.py` |

### Rules mapped to 5.1.16 in each source
- code: sm1_input_validation.py, sm3_protection_failure.py, sm5_hardcoded_values.py, sm7_prompt_marker.py
- Table 18: DM-7, SM-1, SM-3, SM-5, SM-7
- traceability CSV: DM-7, SM-1, SM-3, SM-5, SM-7

SM-6 clauses: code ['5.1.14', '5.1.15'], Table 18 ['5.1.14', '5.1.15'], traceability ['5.1.14', '5.1.15']; agree: True

### Test files named in the traceability CSV for SM rules
- SM-1: `enfield_watchdog_static/test/test_sm1_input_validation.py` exists: False
- SM-2: `enfield_watchdog_static/test/test_sm2_unchecked_return.py` exists: False
- SM-3: `enfield_watchdog_static/test/test_sm3_protection_failure.py` exists: False
- SM-4: `enfield_watchdog_static/test/test_sm4_unusual_conditions.py` exists: False
- SM-5: `enfield_watchdog_static/test/test_sm5_hardcoded_values.py` exists: False
- SM-6a: `enfield_watchdog_static/test/test_sm6_missing_preamble.py` exists: False
- SM-6b: `enfield_watchdog_static/test/test_sm6_missing_preamble.py` exists: False
- SM-7: `enfield_watchdog_static/test/test_sm7_prompt_marker.py` exists: False