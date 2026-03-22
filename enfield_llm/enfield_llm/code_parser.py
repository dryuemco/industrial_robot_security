"""Code parser — extracts URScript from LLM response text.

LLMs return code in various formats: markdown code blocks,
raw code, mixed prose+code. This parser extracts the URScript
portion and performs basic structural validation.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)

# URScript keywords that indicate actual robot code
URSCRIPT_INDICATORS = [
    "def ",
    "movej",
    "movel",
    "movec",
    "set_tcp",
    "set_payload",
    "textmsg",
    "popup",
    "speedl",
    "speedj",
    "servoj",
    "stopl",
    "stopj",
    "sleep",
    "get_actual_tcp_pose",
    "get_actual_joint_positions",
    "socket_open",
    "write_output_boolean_register",
    "set_digital_out",
    "set_standard_analog_out",
    "freedrive_mode",
    "end_freedrive_mode",
    "force_mode",
    "end_force_mode",
    "set_safety_mode_transition_hardness",
    "power_on",
    "power_off",
]

# Pattern for fenced code blocks
CODE_BLOCK_PATTERN = re.compile(
    r"```(?:urscript|python|ur|URScript|text)?\s*\n"
    r"(.*?)"
    r"\n\s*```",
    re.DOTALL,
)

# Pattern for URScript function definition
URSCRIPT_FUNC_PATTERN = re.compile(
    r"(def\s+\w+\s*\(.*?\).*?end)",
    re.DOTALL,
)


@dataclass
class ParseResult:
    """Result of code extraction from LLM response.

    Attributes:
        code: Extracted URScript code, or None if extraction failed.
        extraction_method: How the code was found ('code_block',
            'function_pattern', 'indicator_heuristic', or 'failed').
        confidence: Rough confidence score (0.0 to 1.0).
        line_count: Number of non-empty lines in extracted code.
        has_motion_command: Whether any motion command was found.
        has_safety_check: Whether any safety-related call was found.
    """

    code: Optional[str]
    extraction_method: str
    confidence: float
    line_count: int = 0
    has_motion_command: bool = False
    has_safety_check: bool = False


class CodeParser:
    """Extracts and validates URScript from LLM responses.

    Uses a cascade of extraction strategies:
    1. Fenced code blocks (```urscript ... ```)
    2. URScript function pattern (def ... end)
    3. Indicator-based heuristic (lines with URScript keywords)

    Usage:
        parser = CodeParser()
        result = parser.extract(llm_response_text)
        if result.code:
            # Feed to watchdog
            pass
    """

    def extract(self, text: str) -> ParseResult:
        """Extract URScript code from LLM response text.

        Tries extraction strategies in order of reliability.
        Returns the first successful extraction.
        """
        if not text or not text.strip():
            return ParseResult(
                code=None,
                extraction_method="failed",
                confidence=0.0,
            )

        # Strategy 1: Fenced code blocks
        result = self._extract_from_code_block(text)
        if result.code:
            return result

        # Strategy 2: URScript function pattern
        result = self._extract_from_function_pattern(text)
        if result.code:
            return result

        # Strategy 3: Indicator-based heuristic
        result = self._extract_from_indicators(text)
        if result.code:
            return result

        return ParseResult(
            code=None,
            extraction_method="failed",
            confidence=0.0,
        )

    def _extract_from_code_block(self, text: str) -> ParseResult:
        """Extract code from markdown fenced code blocks."""
        matches = CODE_BLOCK_PATTERN.findall(text)
        if not matches:
            return ParseResult(code=None, extraction_method="code_block",
                               confidence=0.0)

        # If multiple code blocks, pick the one most likely to be URScript
        best_code = None
        best_score = -1

        for match in matches:
            code = match.strip()
            score = self._urscript_score(code)
            if score > best_score:
                best_score = score
                best_code = code

        if best_code and best_score > 0:
            return self._build_result(best_code, "code_block", 0.9)

        return ParseResult(code=None, extraction_method="code_block",
                           confidence=0.0)

    def _extract_from_function_pattern(self, text: str) -> ParseResult:
        """Extract URScript function definitions (def ... end)."""
        matches = URSCRIPT_FUNC_PATTERN.findall(text)
        if not matches:
            return ParseResult(code=None, extraction_method="function_pattern",
                               confidence=0.0)

        # Join all function definitions found
        code = "\n\n".join(m.strip() for m in matches)
        return self._build_result(code, "function_pattern", 0.7)

    def _extract_from_indicators(self, text: str) -> ParseResult:
        """Extract lines containing URScript indicators."""
        lines = text.split("\n")
        code_lines: list[str] = []
        in_code = False

        for line in lines:
            stripped = line.strip()
            if not stripped:
                if in_code:
                    code_lines.append("")
                continue

            is_code_line = any(
                ind.lower() in stripped.lower() for ind in URSCRIPT_INDICATORS
            )
            # Also include lines that look like assignments or comments
            # when we're inside a code section
            looks_like_code = (
                stripped.startswith("#")
                or stripped.startswith("//")
                or "=" in stripped
                or stripped.startswith("end")
                or stripped.startswith("if ")
                or stripped.startswith("while ")
                or stripped.startswith("local ")
                or stripped.startswith("global ")
            )

            if is_code_line:
                in_code = True
                code_lines.append(stripped)
            elif in_code and looks_like_code:
                code_lines.append(stripped)
            elif in_code and not stripped.startswith(("*", "-", ">")):
                # Stop collecting if we hit prose-like content
                # (but keep going for blank lines)
                in_code = False

        if not code_lines:
            return ParseResult(code=None, extraction_method="indicator_heuristic",
                               confidence=0.0)

        code = "\n".join(code_lines)
        return self._build_result(code, "indicator_heuristic", 0.4)

    def _urscript_score(self, code: str) -> int:
        """Score how likely a code block is to be URScript."""
        code_lower = code.lower()
        return sum(
            1 for ind in URSCRIPT_INDICATORS
            if ind.lower() in code_lower
        )

    def _build_result(
        self, code: str, method: str, confidence: float
    ) -> ParseResult:
        """Build a ParseResult with analysis metadata."""
        code_lower = code.lower()
        non_empty = [l for l in code.split("\n") if l.strip()]

        motion_commands = ["movej", "movel", "movec", "speedl", "speedj",
                           "servoj", "servoc"]
        safety_keywords = ["safety", "stop", "halt", "popup", "protect",
                           "check", "limit", "zone", "emergency",
                           "set_safety"]

        return ParseResult(
            code=code,
            extraction_method=method,
            confidence=confidence,
            line_count=len(non_empty),
            has_motion_command=any(mc in code_lower for mc in motion_commands),
            has_safety_check=any(sk in code_lower for sk in safety_keywords),
        )
