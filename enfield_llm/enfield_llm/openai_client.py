"""OpenAI-compatible LLM client — used by both GPT-4o and Grok.

Grok's API is OpenAI-compatible, so both providers share this
implementation with different base URLs and model identifiers.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Optional

import httpx

from enfield_llm.base_client import LLMClient, LLMResponse, ResponseStatus

logger = logging.getLogger(__name__)

# Provider configurations
OPENAI_BASE_URL = "https://api.openai.com/v1"
XAI_BASE_URL = "https://api.x.ai/v1"


class OpenAICompatibleClient(LLMClient):
    """Client for OpenAI-compatible APIs (GPT-4o, Grok).

    Both OpenAI and xAI expose the same /chat/completions endpoint
    format, differing only in base URL and auth.
    """

    def __init__(
        self,
        api_key: str,
        model: str,
        base_url: str = OPENAI_BASE_URL,
        provider: str = "openai",
        temperature: float = 0.0,
        max_tokens: int = 4096,
        log_dir: Optional[Path] = None,
        timeout: float = 120.0,
    ) -> None:
        super().__init__(
            api_key=api_key,
            model=model,
            temperature=temperature,
            max_tokens=max_tokens,
            log_dir=log_dir,
        )
        self.base_url = base_url.rstrip("/")
        self._provider = provider
        self.timeout = timeout

    @property
    def provider_name(self) -> str:
        return self._provider

    def _call_api(
        self,
        system_prompt: str,
        user_prompt: str,
    ) -> LLMResponse:
        """Call the OpenAI-compatible /chat/completions endpoint."""
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }

        payload = {
            "model": self.model,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
        }

        try:
            with httpx.Client(timeout=self.timeout) as client:
                resp = client.post(
                    f"{self.base_url}/chat/completions",
                    headers=headers,
                    json=payload,
                )
                resp.raise_for_status()
                data = resp.json()
        except httpx.HTTPStatusError as e:
            logger.error("HTTP %d from %s: %s", e.response.status_code,
                         self._provider, e.response.text[:500])
            return LLMResponse(
                model=self.model,
                status=ResponseStatus.ERROR,
                raw_response=f"HTTP {e.response.status_code}: {e.response.text[:500]}",
            )
        except httpx.RequestError as e:
            logger.error("Request error for %s: %s", self._provider, e)
            return LLMResponse(
                model=self.model,
                status=ResponseStatus.ERROR,
                raw_response=str(e),
            )

        # Extract response content
        choice = data.get("choices", [{}])[0]
        message = choice.get("message", {})
        content = message.get("content", "")

        usage = data.get("usage", {})

        return LLMResponse(
            model=self.model,
            status=ResponseStatus.SUCCESS,
            raw_response=content,
            prompt_tokens=usage.get("prompt_tokens", 0),
            completion_tokens=usage.get("completion_tokens", 0),
            request_id=data.get("id", ""),
        )


def create_openai_client(
    api_key: str,
    model: str = "gpt-4o-2024-08-06",
    **kwargs,
) -> OpenAICompatibleClient:
    """Factory for GPT-4o client."""
    return OpenAICompatibleClient(
        api_key=api_key,
        model=model,
        base_url=OPENAI_BASE_URL,
        provider="openai",
        **kwargs,
    )


def create_grok_client(
    api_key: str,
    model: str = "grok-4.1-fast",
    **kwargs,
) -> OpenAICompatibleClient:
    """Factory for Grok client (xAI, OpenAI-compatible API)."""
    return OpenAICompatibleClient(
        api_key=api_key,
        model=model,
        base_url=XAI_BASE_URL,
        provider="xai",
        **kwargs,
    )
