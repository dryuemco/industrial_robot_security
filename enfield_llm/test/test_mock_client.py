"""Unit tests for MockLLMClient.

These tests pin down the public contract of the mock client so that
the runner smoke test (see tests/test_runner_mock_smoke.py) can rely
on deterministic, reproducible responses without a live Ollama server.
"""

from __future__ import annotations

import pytest

from enfield_llm.base_client import LLMResponse, ResponseStatus
from enfield_llm.code_parser import CodeParser
from enfield_llm.mock_client import (
    MockLLMClient,
    MockTemplate,
    _TEMPLATE_ORDER,
)


# ---------------------------------------------------------------------------
# Basic contract
# ---------------------------------------------------------------------------


class TestMockClientContract:
    def test_provider_name(self):
        client = MockLLMClient()
        assert client.provider_name == "mock"

    def test_default_model(self):
        client = MockLLMClient()
        assert client.model == "mock-model-v1"

    def test_generate_returns_llm_response(self):
        client = MockLLMClient()
        resp = client.generate("sys", "user")
        assert isinstance(resp, LLMResponse)
        assert resp.model == "mock-model-v1"
        assert resp.timestamp != ""
        assert resp.latency_ms > 0.0

    def test_call_count_increments(self):
        client = MockLLMClient()
        assert client.call_count == 0
        client.generate("s", "u")
        assert client.call_count == 1
        client.generate("s", "u")
        assert client.call_count == 2


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------


class TestDeterminism:
    def test_same_seed_same_sequence(self):
        """Two clients with the same seed produce identical template
        sequences, independent of prompt content."""
        a = MockLLMClient(seed=42)
        b = MockLLMClient(seed=42)
        seq_a = [a.generate("x", "y").metadata["template"] for _ in range(14)]
        seq_b = [
            b.generate("different", "prompts").metadata["template"]
            for _ in range(14)
        ]
        assert seq_a == seq_b

    def test_different_seed_different_sequence(self):
        a = MockLLMClient(seed=0)
        b = MockLLMClient(seed=1)
        seq_a = [a.generate("s", "u").metadata["template"] for _ in range(7)]
        seq_b = [b.generate("s", "u").metadata["template"] for _ in range(7)]
        assert seq_a != seq_b

    def test_rotation_covers_all_templates(self):
        """A full rotation of length len(_TEMPLATE_ORDER) visits every
        template exactly once, regardless of the starting offset."""
        client = MockLLMClient(seed=3)
        n = len(_TEMPLATE_ORDER)
        seen = {
            client.generate("s", "u").metadata["template"] for _ in range(n)
        }
        assert seen == {t.value for t in _TEMPLATE_ORDER}


# ---------------------------------------------------------------------------
# Forced template mode
# ---------------------------------------------------------------------------


class TestForcedTemplate:
    @pytest.mark.parametrize("template", list(MockTemplate))
    def test_forced_template_is_returned(self, template):
        client = MockLLMClient(forced_template=template)
        resp = client.generate("s", "u")
        assert resp.metadata["template"] == template.value


# ---------------------------------------------------------------------------
# Downstream parser / refusal interaction
# ---------------------------------------------------------------------------


class TestDownstreamBehavior:
    """These assertions are what the runner smoke test relies on:
    each template must drive a specific downstream code path."""

    def _response(self, template: MockTemplate) -> LLMResponse:
        client = MockLLMClient(forced_template=template)
        return client.generate("sys", "user")

    def test_refusal_template_is_classified_as_refusal(self):
        resp = self._response(MockTemplate.REFUSAL)
        assert resp.status == ResponseStatus.REFUSAL
        assert resp.generated_code is None

    def test_empty_template_leaves_no_extractable_code(self):
        resp = self._response(MockTemplate.EMPTY)
        assert resp.status == ResponseStatus.SUCCESS
        assert resp.raw_response == ""
        parser = CodeParser()
        result = parser.extract(resp.raw_response)
        assert result.code is None
        assert result.extraction_method == "failed"

    def test_clean_template_is_valid_urscript(self):
        resp = self._response(MockTemplate.CLEAN)
        assert resp.status == ResponseStatus.SUCCESS
        parser = CodeParser()
        result = parser.extract(resp.raw_response)
        assert result.code is not None
        assert result.is_valid_urscript is True
        assert result.has_motion_command is True

    def test_sm1_template_is_valid_urscript_but_missing_speed_params(self):
        resp = self._response(MockTemplate.SM1_VIOLATION)
        parser = CodeParser()
        result = parser.extract(resp.raw_response)
        assert result.code is not None
        assert result.is_valid_urscript is True
        # The whole point of this template: no v= / a= parameters on
        # the motion calls, which is what SM-1 fires on.
        assert "v=" not in result.code
        assert "a=" not in result.code

    def test_sm5_template_contains_hardcoded_high_speed(self):
        resp = self._response(MockTemplate.SM5_VIOLATION)
        parser = CodeParser()
        result = parser.extract(resp.raw_response)
        assert result.code is not None
        assert result.is_valid_urscript is True
        assert "v=2.5" in result.code  # well above 0.25 m/s collaborative

    def test_sm6_template_has_no_set_tcp_preamble(self):
        resp = self._response(MockTemplate.SM6_VIOLATION)
        parser = CodeParser()
        result = parser.extract(resp.raw_response)
        assert result.code is not None
        assert result.is_valid_urscript is True
        assert "set_tcp" not in result.code
        assert "set_payload" not in result.code

    def test_pseudocode_template_fails_validity_gate(self):
        """PSEUDOCODE is the whole reason the validity gate exists.
        The response mentions URScript keywords in prose but contains
        no function-call pattern, so is_valid_urscript must be False.
        Either the parser returns code=None, or it returns code with
        is_valid_urscript==False."""
        resp = self._response(MockTemplate.PSEUDOCODE)
        # Base class refusal detector checks for "movej" as a code
        # indicator and skips refusal, so status stays SUCCESS.
        assert resp.status == ResponseStatus.SUCCESS
        parser = CodeParser()
        result = parser.extract(resp.raw_response)
        if result.code is not None:
            assert result.is_valid_urscript is False
