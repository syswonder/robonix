from uapi.runtime.provider import SkillProvider
from typing import List


class Registry:
    # store actual skill providers
    def __init__(self):
        self.providers: list[SkillProvider] = []

    def add_provider(self, provider: SkillProvider):
        self.providers.append(provider)

    def get_provider(self, name: str) -> SkillProvider:
        for provider in self.providers:
            if provider.name == name:
                return provider
        return None

    def __str__(self):
        return f"Registry(providers={self.providers})"
