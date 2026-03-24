"""Client factory — creates LLM clients from configuration.

Reads connection info from environment variables by default.
Provides a single entry point for creating any supported client.

Supported providers:
  - ollama: Local LLM via Ollama (default, zero cost)
  - anthropic: Claude API
  - openai: GPT-4o API
  - xai: Grok API

Usage:
    from enfield_llm.factory import create_client, create_all_clients

    client = create_client("ollama", model="qwen2.5-coder:32b")
    clients = create_all_clients()  # all 3 local models
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Optional

from enfield_llm.base_client import LLMClient
from enfield_llm.claude_client import ClaudeClient
from enfield_llm.openai_client import (
    OpenAICompatibleClient,
    OPENAI_BASE_URL,
    XAI_BASE_URL,
    OLLAMA_BASE_URL,
)

logger = logging.getLogger(__name__)

# Default model identifiers — pin for reproducibility
DEFAULT_MODELS = {
    "ollama_qwen_coder": "qwen2.5-coder:32b",
    "ollama_qwen35": "qwen3.5:27b",
    "ollama_codellama": "codellama:34b",
    "anthropic": "claude-sonnet-4-20250514",
    "openai": "gpt-4o-2024-08-06",
    "xai": "grok-4.1-fast",
}

# Environment variable names
ENV_KEYS = {
    "ollama": "OLLAMA_HOST",        # e.g. "http://192.168.1.100:11434"
    "anthropic": "ANTHROPIC_API_KEY",
    "openai": "OPENAI_API_KEY",
    "xai": "XAI_API_KEY",
}

# Primary experiment models (local, zero cost)
EXPERIMENT_MODELS = {
    "qwen_coder": "qwen2.5-coder:32b",
    "qwen35": "qwen3.5:27b",
    "codellama": "codellama:34b",
}


def _get_ollama_base_url() -> str:
    """Get Ollama base URL from environment or use default."""
    host = os.environ.get("OLLAMA_HOST", "http://localhost:11434")
    # Normalize: ensure /v1 suffix for OpenAI-compatible endpoint
    host = host.rstrip("/")
    if not host.endswith("/v1"):
        host = f"{host}/v1"
    return host


def create_client(
    provider: str,
    api_key: Optional[str] = None,
    model: Optional[str] = None,
    log_dir: Optional[Path] = None,
    temperature: float = 0.0,
    max_tokens: int = 4096,
) -> LLMClient:
    """Create an LLM client for the given provider.

    Args:
        provider: One of 'ollama', 'anthropic', 'openai', 'xai'.
            For Ollama with specific model: 'ollama' + model param.
        api_key: API key. Not needed for Ollama. If None for cloud
            providers, reads from environment variable.
        model: Model identifier. If None, uses pinned default.
        log_dir: Directory for request/response logs.
        temperature: Sampling temperature (0.0 = deterministic).
        max_tokens: Maximum output tokens.

    Returns:
        Configured LLMClient instance.

    Raises:
        ValueError: If provider is unknown or API key not found.
    """
    provider = provider.lower()

    kwargs = dict(
        temperature=temperature,
        max_tokens=max_tokens,
        log_dir=log_dir,
    )

    # --- Ollama (local, no API key needed) ---
    if provider == "ollama":
        base_url = _get_ollama_base_url()
        model = model or EXPERIMENT_MODELS["qwen_coder"]
        return OpenAICompatibleClient(
            api_key="ollama",  # Ollama doesn't require auth
            model=model,
            base_url=base_url,
            provider="ollama",
            **kwargs,
        )

    # --- Cloud providers (API key required) ---
    if provider == "anthropic":
        if api_key is None:
            api_key = os.environ.get(ENV_KEYS["anthropic"])
            if not api_key:
                raise ValueError("ANTHROPIC_API_KEY not set")
        model = model or DEFAULT_MODELS["anthropic"]
        return ClaudeClient(api_key=api_key, model=model, **kwargs)

    if provider == "openai":
        if api_key is None:
            api_key = os.environ.get(ENV_KEYS["openai"])
            if not api_key:
                raise ValueError("OPENAI_API_KEY not set")
        model = model or DEFAULT_MODELS["openai"]
        return OpenAICompatibleClient(
            api_key=api_key, model=model,
            base_url=OPENAI_BASE_URL, provider="openai", **kwargs,
        )

    if provider == "xai":
        if api_key is None:
            api_key = os.environ.get(ENV_KEYS["xai"])
            if not api_key:
                raise ValueError("XAI_API_KEY not set")
        model = model or DEFAULT_MODELS["xai"]
        return OpenAICompatibleClient(
            api_key=api_key, model=model,
            base_url=XAI_BASE_URL, provider="xai", **kwargs,
        )

    raise ValueError(
        f"Unknown provider '{provider}'. "
        f"Supported: ollama, anthropic, openai, xai"
    )


def create_all_clients(
    log_dir: Optional[Path] = None,
    temperature: float = 0.0,
) -> dict[str, LLMClient]:
    """Create clients for all 3 local experiment models via Ollama.

    Returns:
        Dict mapping model short name to client instance.
        e.g. {"qwen_coder": <client>, "qwen35": <client>, "codellama": <client>}
    """
    base_url = _get_ollama_base_url()
    clients: dict[str, LLMClient] = {}

    for short_name, model_id in EXPERIMENT_MODELS.items():
        try:
            clients[short_name] = OpenAICompatibleClient(
                api_key="ollama",
                model=model_id,
                base_url=base_url,
                provider="ollama",
                temperature=temperature,
                log_dir=log_dir,
            )
            logger.info("Created Ollama client: %s (%s)", short_name, model_id)
        except Exception as e:
            logger.error("Failed to create %s client: %s", short_name, e)

    return clients
