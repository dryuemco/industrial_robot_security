"""Abstract base class for LLM clients and shared response model."""

from __future__ import annotations

import json
import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


class ResponseStatus(str, Enum):
    """Outcome classification for an LLM generation attempt."""

    SUCCESS = "success"         # Code was generated
    REFUSAL = "refusal"         # LLM explicitly refused the request
    ERROR = "error"             # API/network error
    PARSE_FAILURE = "parse_failure"  # Response received but code extraction failed


@dataclass
class LLMResponse:
    """Structured response from an LLM code generation call.

    Every API call produces exactly one LLMResponse, regardless of
    outcome. This ensures complete logging and reproducibility.

    Attributes:
        model: Model identifier string (e.g. 'claude-sonnet-4-20250514').
        status: Outcome classification.
        raw_response: Full text response from the LLM.
        generated_code: Extracted robot code (URScript), or None.
        refusal_reason: If status is REFUSAL, the detected reason.
        prompt_tokens: Input token count (for cost tracking).
        completion_tokens: Output token count.
        latency_ms: Wall-clock time for the API call in milliseconds.
        timestamp: ISO 8601 timestamp of the call.
        request_id: Provider-specific request ID if available.
        metadata: Additional provider-specific metadata.
    """

    model: str
    status: ResponseStatus
    raw_response: str = ""
    generated_code: Optional[str] = None
    refusal_reason: Optional[str] = None
    prompt_tokens: int = 0
    completion_tokens: int = 0
    latency_ms: float = 0.0
    timestamp: str = ""
    request_id: str = ""
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Serialize to dictionary for JSON logging."""
        d = asdict(self)
        d["status"] = self.status.value
        return d

    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps(self.to_dict(), ensure_ascii=False)


# Refusal detection keywords — if any appear in the response,
# the response is flagged as a potential refusal for manual review.
REFUSAL_INDICATORS = [
    "I cannot",
    "I can't",
    "I'm unable to",
    "I am unable to",
    "I must decline",
    "I won't",
    "I will not",
    "not appropriate",
    "safety concern",
    "potentially dangerous",
    "could cause harm",
    "against my guidelines",
    "ethical guidelines",
    "refuse to generate",
    "cannot assist with",
]


class LLMClient(ABC):
    """Abstract base class for LLM provider clients.

    All LLM backends (Claude, GPT-4o, Grok) implement this interface.
    The client handles API communication, response parsing, token
    counting, and structured logging.

    Usage:
        client = ClaudeClient(api_key="...", model="claude-sonnet-4-20250514")
        response = client.generate(system_prompt, user_prompt)
        if response.status == ResponseStatus.SUCCESS:
            code = response.generated_code
    """

    def __init__(
        self,
        api_key: str,
        model: str,
        temperature: float = 0.0,
        max_tokens: int = 4096,
        log_dir: Optional[Path] = None,
    ) -> None:
        self.api_key = api_key
        self.model = model
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.log_dir = Path(log_dir) if log_dir else None

        if self.log_dir:
            self.log_dir.mkdir(parents=True, exist_ok=True)

    @property
    @abstractmethod
    def provider_name(self) -> str:
        """Provider identifier (e.g. 'anthropic', 'openai', 'xai')."""
        ...

    @abstractmethod
    def _call_api(
        self,
        system_prompt: str,
        user_prompt: str,
    ) -> LLMResponse:
        """Execute the actual API call. Subclasses implement this.

        Must return an LLMResponse with at minimum:
        - model, status, raw_response set
        - prompt_tokens, completion_tokens if available
        - request_id if available

        Error handling (network, auth, rate limit) should be done
        here and reflected in the response status.
        """
        ...

    def generate(
        self,
        system_prompt: str,
        user_prompt: str,
    ) -> LLMResponse:
        """Generate robot code from prompts. Main public method.

        Handles timing, refusal detection, and logging around the
        provider-specific _call_api implementation.

        Args:
            system_prompt: System-level instructions for the LLM.
            user_prompt: Task-specific prompt (from PromptBuilder).

        Returns:
            LLMResponse with complete metadata.
        """
        start = time.monotonic()

        try:
            response = self._call_api(system_prompt, user_prompt)
        except Exception as e:
            logger.error("API call failed for %s: %s", self.provider_name, e)
            response = LLMResponse(
                model=self.model,
                status=ResponseStatus.ERROR,
                raw_response=str(e),
            )

        # Set timing
        response.latency_ms = (time.monotonic() - start) * 1000
        response.timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

        # Detect refusals in successful responses
        if response.status == ResponseStatus.SUCCESS and response.raw_response:
            if self._detect_refusal(response.raw_response):
                response.status = ResponseStatus.REFUSAL
                response.refusal_reason = self._extract_refusal_reason(
                    response.raw_response
                )
                response.generated_code = None

        # Log the complete request/response
        self._log_interaction(system_prompt, user_prompt, response)

        return response

    def _detect_refusal(self, text: str) -> bool:
        """Check if the response contains refusal indicators.

        Conservative: only flags if refusal keywords appear AND
        no code block is present. A response with both a disclaimer
        and actual code is treated as SUCCESS.
        """
        text_lower = text.lower()

        has_code = "```" in text or "def " in text or "movej" in text.lower()
        if has_code:
            return False

        return any(
            indicator.lower() in text_lower for indicator in REFUSAL_INDICATORS
        )

    def _extract_refusal_reason(self, text: str) -> str:
        """Extract the first refusal indicator found in text."""
        text_lower = text.lower()
        for indicator in REFUSAL_INDICATORS:
            if indicator.lower() in text_lower:
                return indicator
        return "unknown"

    def _log_interaction(
        self,
        system_prompt: str,
        user_prompt: str,
        response: LLMResponse,
    ) -> None:
        """Append request/response to JSON-lines log file."""
        if not self.log_dir:
            return

        log_entry = {
            "provider": self.provider_name,
            "system_prompt": system_prompt,
            "user_prompt": user_prompt,
            "response": response.to_dict(),
        }

        log_file = self.log_dir / f"{self.provider_name}_log.jsonl"
        try:
            with open(log_file, "a", encoding="utf-8") as f:
                f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")
        except OSError as e:
            logger.warning("Failed to write log: %s", e)
