#!/usr/bin/env bash
# =============================================================================
# ENFIELD — Run All Tests
# Single command to verify entire test suite (740+ tests)
# Usage: ./scripts/run_tests.sh [--fast] [--suite SUITE_NAME]
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_DIR"

GREEN='\033[0;32m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

TOTAL_PASSED=0
TOTAL_FAILED=0
ALL_PASSED=true
SUMMARY=""

FAST_MODE=false
FILTER_SUITE=""
VERBOSE="-v"

while [[ $# -gt 0 ]]; do
    case $1 in
        --fast) FAST_MODE=true; VERBOSE="-q"; shift ;;
        --suite) FILTER_SUITE="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: ./scripts/run_tests.sh [--fast] [--suite NAME]"
            echo "Suites: tasks, attacks, translators, watchdog, llm, experiment"
            exit 0 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

run_suite() {
    local label="$1"
    local pypath="$2"
    local test_path="$3"

    if [[ -n "$FILTER_SUITE" && "$label" != *"$FILTER_SUITE"* ]]; then
        return 0
    fi

    echo -e "\n${CYAN}━━━ ${BOLD}${label}${NC}${CYAN} ━━━${NC}"

    local output exit_code=0
    output=$(PYTHONPATH="$pypath" python3 -m pytest "$test_path" $VERBOSE --tb=short 2>&1) || exit_code=$?

    if [[ "$FAST_MODE" == "false" ]]; then
        echo "$output"
    fi

    local last_line passed=0 failed=0
    last_line=$(echo "$output" | tail -1)
    passed=$(echo "$last_line" | grep -oP '\d+(?= passed)' || echo 0)
    failed=$(echo "$last_line" | grep -oP '\d+(?= failed)' || echo 0)

    TOTAL_PASSED=$((TOTAL_PASSED + passed))
    TOTAL_FAILED=$((TOTAL_FAILED + failed))

    if [[ $exit_code -eq 0 ]]; then
        SUMMARY="${SUMMARY}\n${GREEN}  ✓ ${label}: ${passed} passed${NC}"
    else
        SUMMARY="${SUMMARY}\n${RED}  ✗ ${label}: ${passed} passed, ${failed} failed${NC}"
        ALL_PASSED=false
    fi
}

echo -e "${BOLD}ENFIELD — Running All Tests${NC}"
echo "Date: $(date '+%Y-%m-%d %H:%M:%S')"

START_TIME=$(date +%s)

run_suite "enfield_tasks"           "enfield_tasks"                         "enfield_tasks/test/"
run_suite "enfield_attacks"         "enfield_attacks:enfield_tasks"         "enfield_attacks/test/"
run_suite "enfield_translators"     "enfield_translators:enfield_tasks"     "enfield_translators/test/"
run_suite "enfield_watchdog_static" "enfield_watchdog_static:enfield_tasks" "enfield_watchdog_static/test/"
run_suite "enfield_llm"             "enfield_llm:enfield_tasks"             "enfield_llm/test/"
run_suite "experiment_runner"       "enfield_watchdog_static:enfield_tasks" "tests/"

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo ""
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BOLD}  SUMMARY${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "$SUMMARY"
echo ""

TOTAL=$((TOTAL_PASSED + TOTAL_FAILED))
echo -e "  Total: ${BOLD}${TOTAL}${NC} tests in ${DURATION}s"
echo -e "  Passed: ${GREEN}${TOTAL_PASSED}${NC}"

if [[ $TOTAL_FAILED -gt 0 ]]; then
    echo -e "  Failed: ${RED}${TOTAL_FAILED}${NC}"
fi

echo ""
if [[ "$ALL_PASSED" == "true" ]]; then
    echo -e "${GREEN}${BOLD}  ✓ ALL TESTS PASSED${NC}"
    exit 0
else
    echo -e "${RED}${BOLD}  ✗ FAILURES DETECTED${NC}"
    exit 1
fi
