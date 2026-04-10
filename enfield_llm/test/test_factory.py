"""Unit tests for the LLM client factory.

These tests pin down the ``create_client`` contract for the providers
that can be tested without network access or API keys: ``mock`` and
the error paths. Live providers (``ollama``, ``anthropic``, ``openai``,
``xai``) are exercised by integration suites elsewhere.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from enfield_llm.base_client import LLMClient
from enfield_llm.factory import create_client
from enfield_llm.mock_client import MockLLMClient, MockTemplate


# ---------------------------------------------------------------------------
# Mock provider
# ---------------------------------------------------------------------------


class TestMockProvider:
    def test_mock_provider_returns_mock_client(self):
        client = create_client("mock")
        assert isinstance(client, MockLLMClient)
        assert isinstance(client, LLMClient)
        assert client.provider_name == "mock"

    def test_mock_default_model(self):
        client = create_client("mock")
        assert client.model == "mock-model-v1"

    def test_mock_custom_model_override(self):
        client = create_client("mock", model="mock-alt")
        assert client.model == "mock-alt"

    def test_mock_forwards_temperature_and_max_tokens(self):
        client = create_client(
            "mock", temperature=0.7, max_tokens=256
        )
        assert client.temperature == 0.7
        assert client.max_tokens == 256

    def test_mock_forwards_log_dir(self, tmp_path: Path):
        client = create_client("mock", log_dir=tmp_path)
        assert client.log_dir == tmp_path
        assert tmp_path.exists()

    def test_mock_provider_is_case_insensitive(self):
        client = create_client("MOCK")
        assert isinstance(client, MockLLMClient)

    def test_mock_client_produces_deterministic_sequence(self):
        """Factory-created mock client is still a regular MockLLMClient:
        same seed gives the same template sequence."""
        a = create_client("mock")
        b = create_client("mock")
        seq_a = [a.generate("s", "u").metadata["template"] for _ in range(7)]
        seq_b = [b.generate("s", "u").metadata["template"] for _ in range(7)]
        assert seq_a == seq_b
        # And the rotation visits every template.
        assert set(seq_a) == {t.value for t in MockTemplate}


# ---------------------------------------------------------------------------
# Error paths
# ---------------------------------------------------------------------------


class TestUnknownProvider:
    def test_unknown_provider_raises(self):
        with pytest.raises(ValueError, match="Unknown provider"):
            create_client("nonexistent")

    def test_unknown_provider_error_lists_mock(self):
        """Regression: after PR-2 the error message must advertise
        the new 'mock' provider alongside the live ones."""
        with pytest.raises(ValueError, match="mock"):
            create_client("nonexistent")


class TestCloudProviderMissingKey:
    """If no environment variable is set, the cloud branches must
    raise a clean ValueError instead of crashing inside the client
    constructor."""

    @pytest.fixture(autouse=True)
    def _clear_keys(self, monkeypatch):
        for var in ("ANTHROPIC_API_KEY", "OPENAI_API_KEY", "XAI_API_KEY"):
            monkeypatch.delenv(var, raising=False)

    def test_anthropic_without_key_raises(self):
        with pytest.raises(ValueError, match="ANTHROPIC_API_KEY"):
            create_client("anthropic")

    def test_openai_without_key_raises(self):
        with pytest.raises(ValueError, match="OPENAI_API_KEY"):
            create_client("openai")

    def test_xai_without_key_raises(self):
        with pytest.raises(ValueError, match="XAI_API_KEY"):
            create_client("xai")
