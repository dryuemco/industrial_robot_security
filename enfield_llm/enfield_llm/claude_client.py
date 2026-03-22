"""Anthropic Claude client implementation."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Optional

import httpx

from enfield_llm.base_client import LLMClient, LLMResponse, ResponseStatus

logger = logging.getLogger(__name__)

ANTHROPIC_BASE_URL = "https://api.anthropic.com/v1"
ANTHROPIC_API_VERSION = "2023-06-01"


class ClaudeClient(LLMClient):
    """Client for Anthropic's Claude Messages API."""

    def __init__(
        self,
        api_key: str,
        model: str = "claude-sonnet-4-20250514",
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
        self.timeout = timeout

    @property
    def provider_name(self) -> str:
        return "anthropic"

    def _call_api(
        self,
        system_prompt: str,
        user_prompt: str,
    ) -> LLMResponse:
        """Call the Anthropic Messages API."""
        headers = {
            "x-api-key": self.api_key,
            "anthropic-version": ANTHROPIC_API_VERSION,
            "content-type": "application/json",
        }

        payload = {
            "model": self.model,
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
            "system": system_prompt,
            "messages": [
                {"role": "user", "content": user_prompt},
            ],
        }

        try:
            with httpx.Client(timeout=self.timeout) as client:
                resp = client.post(
                    f"{ANTHROPIC_BASE_URL}/messages",
                    headers=headers,
                    json=payload,
                )
                resp.raise_for_status()
                data = resp.json()
        except httpx.HTTPStatusError as e:
            logger.error("HTTP %d from Anthropic: %s",
                         e.response.status_code, e.response.text[:500])
            return LLMResponse(
                model=self.model,
                status=ResponseStatus.ERROR,
                raw_response=f"HTTP {e.response.status_code}: {e.response.text[:500]}",
            )
        except httpx.RequestError as e:
            logger.error("Request error for Anthropic: %s", e)
            return LLMResponse(
                model=self.model,
                status=ResponseStatus.ERROR,
                raw_response=str(e),
            )

        # Extract text from content blocks
        content_blocks = data.get("content", [])
        text_parts = [
            block.get("text", "")
            for block in content_blocks
            if block.get("type") == "text"
        ]
        content = "\n".join(text_parts)

        usage = data.get("usage", {})

        return LLMResponse(
            model=self.model,
            status=ResponseStatus.SUCCESS,
            raw_response=content,
            prompt_tokens=usage.get("input_tokens", 0),
            completion_tokens=usage.get("output_tokens", 0),
            request_id=data.get("id", ""),
            metadata={"stop_reason": data.get("stop_reason", "")},
        )


def create_claude_client(
    api_key: str,
    model: str = "claude-sonnet-4-20250514",
    **kwargs,
) -> ClaudeClient:
    """Factory for Claude client."""
    return ClaudeClient(api_key=api_key, model=model, **kwargs)
