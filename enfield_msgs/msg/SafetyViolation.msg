# SafetyViolation.msg
# Published by watchdog nodes when a safety invariant is breached.

std_msgs/Header header

# Attack type identifier (A1-A8)
string attack_type

# ISO 10218 clause reference (e.g., "5.6", "5.12.3")
string iso_clause

# Human-readable description of the violation
string description

# Severity metric (attack-type-specific, dimensionless or physical units)
float64 severity

# Violation source: "ast" (static) or "runtime" (monitor)
string source

# Detection mechanism name (DM-1 through DM-7)
string detection_mechanism

# TCP position at time of violation (if applicable)
geometry_msgs/Point tcp_position

# TCP speed at time of violation in mm/s (if applicable)
float64 tcp_speed

# Scenario/task identifier
string scenario_id
