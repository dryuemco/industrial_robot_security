"""Tests for enfield_llm.base_client (refusal detection, response model)."""

import json

import pytest

from enfield_llm.base_client import (
    LLMResponse,
    ResponseStatus,
    LLMClient,
    REFUSAL_INDICATORS,
)


class TestLLMResponse:
    """Test the response data model."""

    def test_to_dict(self):
        resp = LLMResponse(
            model="test-model",
            status=ResponseStatus.SUCCESS,
            raw_response="def task():\n  movej([0,0,0,0,0,0])\nend",
            generated_code="def task():\n  movej([0,0,0,0,0,0])\nend",
            prompt_tokens=100,
            completion_tokens=50,
        )
        d = resp.to_dict()
        assert d["status"] == "success"
        assert d["model"] == "test-model"
        assert d["prompt_tokens"] == 100

    def test_to_json_roundtrip(self):
        resp = LLMResponse(
            model="test",
            status=ResponseStatus.REFUSAL,
            refusal_reason="safety concern",
        )
        j = resp.to_json()
        parsed = json.loads(j)
        assert parsed["status"] == "refusal"
        assert parsed["refusal_reason"] == "safety concern"


class TestRefusalDetection:
    """Test refusal detection logic from the base client.

    Uses a concrete stub to access the protected methods.
    """

    class StubClient(LLMClient):
        """Minimal concrete client for testing base class methods."""

        @property
        def provider_name(self) -> str:
            return "stub"

        def _call_api(self, system_prompt, user_prompt):
            return LLMResponse(
                model="stub",
                status=ResponseStatus.SUCCESS,
                raw_response="",
            )

    @pytest.fixture
    def client(self):
        return self.StubClient(api_key="test", model="stub")

    def test_detects_refusal_without_code(self, client):
        text = "I cannot generate this code because it could cause harm."
        assert client._detect_refusal(text) is True

    def test_no_refusal_with_code(self, client):
        text = (
            "I cannot guarantee safety, but here is the code:\n"
            "```\ndef task():\n  movej([0,0,0,0,0,0])\nend\n```"
        )
        assert client._detect_refusal(text) is False

    def test_no_refusal_on_normal_response(self, client):
        text = "def task():\n  movej([0,0,0,0,0,0])\nend"
        assert client._detect_refusal(text) is False

    def test_extracts_refusal_reason(self, client):
        text = "I must decline this request due to safety concerns."
        reason = client._extract_refusal_reason(text)
        assert reason == "I must decline"

    def test_all_indicators_detected(self, client):
        """Each indicator should trigger refusal when no code present."""
        for indicator in REFUSAL_INDICATORS:
            text = f"Response: {indicator} for safety reasons."
            assert client._detect_refusal(text) is True, (
                f"Indicator not detected: {indicator}"
            )
