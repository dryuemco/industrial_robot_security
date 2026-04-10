"""Mock LLM client for deterministic end-to-end runner smoke testing.

This client implements the LLMClient abstract interface without making
any network calls. It returns one of a fixed set of canned URScript (or
pseudo-URScript) responses chosen deterministically from a seeded RNG,
so that a full runner pipeline can be exercised without requiring an
Ollama server or any other live backend.

The canned templates are designed to cover every downstream code path
in the runner:

    CLEAN            — valid URScript, passes all DM and SM rules
    SM1_VIOLATION    — motion calls without v=/a= parameters (CWE-20)
    SM5_VIOLATION    — motion calls with hardcoded high speed literals
                       exceeding collaborative limits (CWE-798)
    SM6_VIOLATION    — motion sequence without set_tcp / set_payload
                       preamble (SM-6a / SM-6b)
    PSEUDOCODE       — prose that mentions motion keywords but has no
                       function-call pattern; triggers the runner's
                       ``invalid_pseudocode`` status via the URScript
                       validity gate in CodeParser
    REFUSAL          — natural-language refusal that the base-class
                       refusal detector flags as ResponseStatus.REFUSAL
    EMPTY            — empty response; exercises the parser's failure
                       branch and the runner's extraction-failure path

Determinism is obtained from an explicit seed (default 42) plus an
internal call counter: call *i* always produces template
``TEMPLATES[(seed + i) % len(TEMPLATES)]``. This is reproducible across
Python runs regardless of ``PYTHONHASHSEED`` and easy to document in
the preregistration and the paper.

Usage::

    from enfield_llm.mock_client import MockLLMClient, MockTemplate

    client = MockLLMClient(seed=42)
    response = client.generate(system_prompt, user_prompt)

    # Force a specific template (useful in unit tests):
    forced = MockLLMClient(seed=42, forced_template=MockTemplate.CLEAN)
"""

from __future__ import annotations

import random
from enum import Enum
from pathlib import Path
from typing import Optional

from enfield_llm.base_client import LLMClient, LLMResponse, ResponseStatus


class MockTemplate(str, Enum):
    """The fixed set of canned response templates.

    Order is stable and part of the public contract: the deterministic
    rotation depends on iteration order, so templates must not be
    reordered without updating the documentation and any tests that
    assert specific sequences.
    """

    CLEAN = "clean"
    SM1_VIOLATION = "sm1_violation"
    SM5_VIOLATION = "sm5_violation"
    SM6_VIOLATION = "sm6_violation"
    PSEUDOCODE = "pseudocode"
    REFUSAL = "refusal"
    EMPTY = "empty"


# Ordered list used for the deterministic rotation.
_TEMPLATE_ORDER: list[MockTemplate] = [
    MockTemplate.CLEAN,
    MockTemplate.SM1_VIOLATION,
    MockTemplate.SM5_VIOLATION,
    MockTemplate.SM6_VIOLATION,
    MockTemplate.PSEUDOCODE,
    MockTemplate.REFUSAL,
    MockTemplate.EMPTY,
]


# ----------------------------------------------------------------------
# Canned response bodies.
#
# Every non-empty template is wrapped in a fenced ``urscript`` code
# block so that CodeParser._extract_from_code_block (Strategy 1, the
# most reliable extraction strategy) picks them up. The PSEUDOCODE
# template intentionally has no code block and no def/end function so
# that the extractor yields code==None or is_valid_urscript==False.
# ----------------------------------------------------------------------

_CLEAN_BODY = (
    "Here is a safe URScript program for the task:\n"
    "\n"
    "```urscript\n"
    "def task():\n"
    "  set_tcp(p[0, 0, 0.150, 0, 0, 0])\n"
    "  set_payload(0.5, [0, 0, 0.05])\n"
    "  movej([0, -1.57, 1.57, -1.57, -1.57, 0], a=0.5, v=0.2)\n"
    "  movel(p[0.3, 0.2, 0.4, 0, 3.14, 0], a=0.3, v=0.1)\n"
    "end\n"
    "```\n"
)

_SM1_BODY = (
    "Generated code:\n"
    "\n"
    "```urscript\n"
    "def task():\n"
    "  set_tcp(p[0, 0, 0.150, 0, 0, 0])\n"
    "  set_payload(0.5, [0, 0, 0.05])\n"
    "  movej([0, -1.57, 1.57, -1.57, -1.57, 0])\n"
    "  movel(p[0.3, 0.2, 0.4, 0, 3.14, 0])\n"
    "end\n"
    "```\n"
)

_SM5_BODY = (
    "Here is a high-performance version:\n"
    "\n"
    "```urscript\n"
    "def task():\n"
    "  set_tcp(p[0, 0, 0.150, 0, 0, 0])\n"
    "  set_payload(0.5, [0, 0, 0.05])\n"
    "  movej([0, -1.57, 1.57, -1.57, -1.57, 0], a=3.0, v=2.5)\n"
    "  movel(p[0.3, 0.2, 0.4, 0, 3.14, 0], a=3.0, v=2.5)\n"
    "end\n"
    "```\n"
)

_SM6_BODY = (
    "Straight to the motion:\n"
    "\n"
    "```urscript\n"
    "def task():\n"
    "  movej([0, -1.57, 1.57, -1.57, -1.57, 0], a=0.5, v=0.2)\n"
    "  movel(p[0.3, 0.2, 0.4, 0, 3.14, 0], a=0.3, v=0.1)\n"
    "end\n"
    "```\n"
)

_PSEUDOCODE_BODY = (
    "To solve this task I would first use movej to move the arm to the "
    "approach pose, then set_tcp to configure the gripper, and finally "
    "movel to place the object at the target location. This sequence "
    "ensures safe collaborative operation."
)

_REFUSAL_BODY = (
    "I cannot generate this code because it could cause harm to nearby "
    "operators. The task as described does not include adequate safety "
    "constraints."
)

_EMPTY_BODY = ""


_TEMPLATE_BODIES: dict[MockTemplate, str] = {
    MockTemplate.CLEAN: _CLEAN_BODY,
    MockTemplate.SM1_VIOLATION: _SM1_BODY,
    MockTemplate.SM5_VIOLATION: _SM5_BODY,
    MockTemplate.SM6_VIOLATION: _SM6_BODY,
    MockTemplate.PSEUDOCODE: _PSEUDOCODE_BODY,
    MockTemplate.REFUSAL: _REFUSAL_BODY,
    MockTemplate.EMPTY: _EMPTY_BODY,
}


class MockLLMClient(LLMClient):
    """Deterministic, offline LLM client for runner smoke testing.

    The client ignores the content of ``system_prompt`` and
    ``user_prompt`` and returns one of the canned templates in
    ``_TEMPLATE_ORDER``. Selection is driven by an explicit seed plus
    an internal call counter, so the sequence of responses across a
    full runner execution is reproducible.

    Parameters
    ----------
    model:
        Model identifier string. Defaults to ``"mock-model-v1"``.
    seed:
        Integer seed controlling template rotation. Defaults to 42.
    forced_template:
        If set, every call returns this template regardless of the
        rotation. Useful for unit tests that need to exercise a
        specific downstream code path.
    log_dir:
        Optional directory for the base-class JSON-lines interaction
        log. Forwarded to ``LLMClient.__init__``.
    """

    def __init__(
        self,
        model: str = "mock-model-v1",
        seed: int = 42,
        forced_template: Optional[MockTemplate] = None,
        log_dir: Optional[Path] = None,
    ) -> None:
        super().__init__(
            api_key="mock",
            model=model,
            temperature=0.0,
            max_tokens=4096,
            log_dir=log_dir,
        )
        self._seed = int(seed)
        self._forced_template = forced_template
        self._call_count = 0
        # Separate RNG for the synthetic latency stream so that
        # the template rotation is fully deterministic regardless of
        # how the latency stream is consumed.
        self._latency_rng = random.Random(self._seed)

    @property
    def provider_name(self) -> str:
        return "mock"

    @property
    def call_count(self) -> int:
        """Number of ``_call_api`` invocations so far."""
        return self._call_count

    def _select_template(self) -> MockTemplate:
        """Pick the template for the current call index."""
        if self._forced_template is not None:
            return self._forced_template
        idx = (self._seed + self._call_count) % len(_TEMPLATE_ORDER)
        return _TEMPLATE_ORDER[idx]

    def _call_api(
        self,
        system_prompt: str,
        user_prompt: str,
    ) -> LLMResponse:
        template = self._select_template()
        body = _TEMPLATE_BODIES[template]
        self._call_count += 1

        # Rough token counts: LLMClient.generate handles latency and
        # timestamp fields, but we seed a plausible synthetic latency
        # into metadata so logs look realistic if inspected.
        prompt_tokens = (len(system_prompt) + len(user_prompt)) // 4
        completion_tokens = len(body) // 4
        synthetic_latency_ms = self._latency_rng.uniform(20.0, 50.0)

        return LLMResponse(
            model=self.model,
            status=ResponseStatus.SUCCESS,
            raw_response=body,
            prompt_tokens=prompt_tokens,
            completion_tokens=completion_tokens,
            metadata={
                "template": template.value,
                "call_index": self._call_count - 1,
                "synthetic_latency_ms": synthetic_latency_ms,
            },
        )
