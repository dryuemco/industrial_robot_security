"""ENFIELD LLM — Code generation client for adversarial testing.

Provides a unified interface for generating industrial robot code
from task IR descriptions using multiple LLM providers (Claude,
GPT-4o, Grok). Supports baseline generation, adversarial prompt
injection, and watchdog-in-the-loop feedback.
"""

from enfield_llm.base_client import LLMClient, LLMResponse
from enfield_llm.prompt_builder import PromptBuilder
from enfield_llm.code_parser import CodeParser

__all__ = [
    'LLMClient',
    'LLMResponse',
    'PromptBuilder',
    'CodeParser',
]
