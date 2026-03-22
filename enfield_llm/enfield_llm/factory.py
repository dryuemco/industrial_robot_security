"""Client factory — creates LLM clients from configuration.

Reads API keys from environment variables by default.
Provides a single entry point for creating any supported client.

Usage:
    from enfield_llm.factory import create_client, create_all_clients

    client = create_client("anthropic")
    clients = create_all_clients()  # all 3 providers
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
)

logger = logging.getLogger(__name__)

# Default model identifiers — pin versions for reproducibility
DEFAULT_MODELS = {
    "anthropic": "claude-sonnet-4-20250514",
    "openai": "gpt-4o-2024-08-06",
    "xai": "grok-4.1-fast",
}

# Environment variable names for API keys
ENV_KEYS = {
    "anthropic": "ANTHROPIC_API_KEY",
    "openai": "OPENAI_API_KEY",
    "xai": "XAI_API_KEY",
}


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
        provider: One of 'anthropic', 'openai', 'xai'.
        api_key: API key. If None, reads from environment variable.
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

    if provider not in DEFAULT_MODELS:
        raise ValueError(
            f"Unknown provider '{provider}'. "
            f"Supported: {list(DEFAULT_MODELS.keys())}"
        )

    if api_key is None:
        env_var = ENV_KEYS[provider]
        api_key = os.environ.get(env_var)
        if not api_key:
            raise ValueError(
                f"API key not provided and {env_var} not set in environment"
            )

    model = model or DEFAULT_MODELS[provider]

    kwargs = dict(
        api_key=api_key,
        model=model,
        temperature=temperature,
        max_tokens=max_tokens,
        log_dir=log_dir,
    )

    if provider == "anthropic":
        return ClaudeClient(**kwargs)
    elif provider == "openai":
        return OpenAICompatibleClient(
            base_url=OPENAI_BASE_URL,
            provider="openai",
            **kwargs,
        )
    elif provider == "xai":
        return OpenAICompatibleClient(
            base_url=XAI_BASE_URL,
            provider="xai",
            **kwargs,
        )

    raise ValueError(f"Unhandled provider: {provider}")  # pragma: no cover


def create_all_clients(
    log_dir: Optional[Path] = None,
    temperature: float = 0.0,
) -> dict[str, LLMClient]:
    """Create clients for all configured providers.

    Skips providers whose API keys are not set (logs a warning).

    Returns:
        Dict mapping provider name to client instance.
    """
    clients: dict[str, LLMClient] = {}

    for provider in DEFAULT_MODELS:
        env_var = ENV_KEYS[provider]
        if not os.environ.get(env_var):
            logger.warning(
                "Skipping %s: %s not set in environment", provider, env_var
            )
            continue

        try:
            clients[provider] = create_client(
                provider, log_dir=log_dir, temperature=temperature
            )
            logger.info("Created %s client (model: %s)",
                        provider, DEFAULT_MODELS[provider])
        except Exception as e:
            logger.error("Failed to create %s client: %s", provider, e)

    return clients
