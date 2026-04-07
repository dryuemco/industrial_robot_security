"""Tests for enfield_llm.code_parser."""

import pytest

from enfield_llm.code_parser import CodeParser


@pytest.fixture
def parser():
    return CodeParser()


class TestCodeBlockExtraction:
    """Test extraction from markdown fenced code blocks."""

    def test_urscript_block(self, parser):
        text = '''Here is the code:

```urscript
def pick_place():
  movej([0.1, -1.5, 1.2, -1.2, -1.57, 0], a=1.4, v=1.05)
  movel(p[0.4, 0.2, 0.05, 3.14, 0, 0], a=0.5, v=0.25)
end
```

This will perform the pick operation.'''

        result = parser.extract(text)
        assert result.code is not None
        assert result.extraction_method == "code_block"
        assert result.confidence == 0.9
        assert "movej" in result.code
        assert "movel" in result.code
        assert result.has_motion_command is True

    def test_python_block_with_urscript(self, parser):
        text = '''```python
def pick_place():
  movej([0.1, -1.5, 1.2, -1.2, -1.57, 0], a=1.4, v=1.05)
end
```'''
        result = parser.extract(text)
        assert result.code is not None
        assert "movej" in result.code

    def test_generic_block(self, parser):
        text = '''```
def main():
  set_tcp(p[0, 0, 0.15, 0, 0, 0])
  movel(p[0.4, 0.2, 0.15, 3.14, 0, 0], v=0.25)
end
```'''
        result = parser.extract(text)
        assert result.code is not None

    def test_multiple_blocks_picks_best(self, parser):
        text = '''```text
This is just a description.
```

```urscript
def task():
  movej([0.1, -1.5, 1.2, -1.2, -1.57, 0], a=1.4, v=1.05)
  movel(p[0.4, 0.2, 0.05, 3.14, 0, 0], a=0.5, v=0.25)
  set_digital_out(0, True)
end
```'''
        result = parser.extract(text)
        assert result.code is not None
        assert "movej" in result.code
        assert "set_digital_out" in result.code


class TestFunctionPatternExtraction:
    """Test extraction from URScript function patterns (no code block)."""

    def test_def_end_pattern(self, parser):
        text = '''Sure, here is the URScript code:

def pick_and_place():
  movej([0.1, -1.5, 1.2, -1.2, -1.57, 0], a=1.4, v=1.05)
  movel(p[0.4, 0.2, 0.05, 3.14, 0, 0], a=0.5, v=0.25)
end

This moves the robot to pick up the object.'''

        result = parser.extract(text)
        assert result.code is not None
        assert result.extraction_method == "function_pattern"
        assert "movej" in result.code


class TestIndicatorExtraction:
    """Test heuristic extraction from mixed prose+code."""

    def test_mixed_content(self, parser):
        text = '''To perform this task, you would use:

movej([0.1, -1.5, 1.2, -1.2, -1.57, 0], a=1.4, v=1.05)
movel(p[0.4, 0.2, 0.05, 3.14, 0, 0], a=0.5, v=0.25)
set_digital_out(0, True)

And that completes the pick operation.'''

        result = parser.extract(text)
        assert result.code is not None
        assert result.extraction_method == "indicator_heuristic"
        assert result.confidence < 0.9  # Lower confidence


class TestEdgeCases:
    """Test edge cases and failure modes."""

    def test_empty_string(self, parser):
        result = parser.extract("")
        assert result.code is None
        assert result.extraction_method == "failed"

    def test_none_like_empty(self, parser):
        result = parser.extract("   \n  \n  ")
        assert result.code is None

    def test_pure_prose(self, parser):
        text = "I cannot generate robot code because of safety concerns."
        result = parser.extract(text)
        assert result.code is None

    def test_non_urscript_code_block(self, parser):
        text = '''```python
print("hello world")
x = 42
```'''
        result = parser.extract(text)
        # Should fail — no URScript indicators
        assert result.code is None


class TestParseResultMetadata:
    """Test that metadata fields are correctly populated."""

    def test_line_count(self, parser):
        text = '''```urscript
def task():
  movej([0, 0, 0, 0, 0, 0])
  movel(p[0.4, 0.2, 0.05, 3.14, 0, 0])
  sleep(0.5)
end
```'''
        result = parser.extract(text)
        assert result.line_count == 5

    def test_has_motion_command(self, parser):
        text = '''```urscript
def task():
  set_tcp(p[0, 0, 0.15, 0, 0, 0])
end
```'''
        result = parser.extract(text)
        assert result.has_motion_command is False

    def test_has_safety_check(self, parser):
        text = '''```urscript
def task():
  popup("safety check", "Warning")
  movej([0, 0, 0, 0, 0, 0])
end
```'''
        result = parser.extract(text)
        assert result.has_safety_check is True



class TestURScriptValidityGate:
    """Validity gate: distinguish real URScript from pseudo-code.

    The substring-based has_motion_command check is insufficient for
    confirmatory experiments because a response like
    "I would call movej to position the arm" matches it. The validity
    gate (is_valid_urscript) requires at least one URScript function
    *call* (keyword + opening parenthesis) and is the pre-specified
    sensitivity gate for H4-H6 (see paper §IV.C, §V.E).
    """

    def test_real_urscript_with_movej_call(self, parser):
        text = """```urscript
def task():
  movej([0, 0, 0, 0, 0, 0], a=1.4, v=1.05)
  sleep(0.5)
end
``````"""
        result = parser.extract(text)
        assert result.is_valid_urscript is True

    def test_pseudo_code_keywords_without_calls(self, parser):
        # CodeLlama-style: keywords appear in prose, no call syntax
        text = """```
# Plan:
# 1. First, use movej to position the arm above the part
# 2. Then call movel to descend slowly
# 3. Engage the gripper, then retreat with movej
`````"""
        result = parser.extract(text)
        # has_motion_command may be True (substring), but the gate must reject
        assert result.is_valid_urscript is False

    def test_set_tcp_call_alone_is_valid(self, parser):
        text = """```urscript
set_tcp(p[0, 0, 0.15, 0, 0, 0])
````"""
        result = parser.extract(text)
        assert result.is_valid_urscript is True

    def test_empty_def_block_is_invalid(self, parser):
        text = """```urscript
def task():
  # TODO: implement motion
end
```"""
        result = parser.extract(text)
        # Has def...end structure but no actual calls -> not committable
        assert result.is_valid_urscript is False

    def test_invalid_when_extraction_failed(self, parser):
        result = parser.extract("I cannot help with that request.")
        assert result.code is None
        assert result.is_valid_urscript is False
