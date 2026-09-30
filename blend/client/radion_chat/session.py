"""Builds a ready-to-run Agent from a profile (shared by the UI and headless scripts)."""

from .agent import Agent, AgentConfig, AgentListener
from .api_client import RadionApiClient
from .llm.openai_compat import OpenAICompatProvider


def make_provider(profile, api_key=""):
    return OpenAICompatProvider(
        profile.base_url, profile.model, api_key,
        vision=profile.vision, temperature=profile.temperature,
        stream=profile.stream, timeout=profile.request_timeout)


def make_agent(profile, api_key="", api_token=None, listener=None, confirm=None):
    config = AgentConfig(
        max_steps=profile.max_steps,
        max_history_chars=profile.context_chars,
        simplify_schema=profile.simplify_schema,
        extra_system_prompt=profile.system_prompt_extra)
    return Agent(make_provider(profile, api_key),
                 RadionApiClient(profile.api_url, api_token),
                 listener or AgentListener(), config, confirm)
