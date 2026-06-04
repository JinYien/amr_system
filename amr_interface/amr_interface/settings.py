#!/usr/bin/env python3

from dataclasses import dataclass
from typing import FrozenSet
import yaml


@dataclass(frozen=True)
class HttpSettings:
    host: str
    port: int


@dataclass(frozen=True)
class RateSettings:
    command_publish: int


@dataclass(frozen=True)
class LimitSettings:
    max_linear_velocity: float
    max_angular_velocity: float


@dataclass(frozen=True)
class TopicSettings:
    command_publisher: str
    control_publisher: str
    state_subscriber: str


@dataclass(frozen=True)
class VocabularySettings:
    modes: FrozenSet[str]
    authorities: FrozenSet[str]
    middle_modes: FrozenSet[str]
    middle_actions: FrozenSet[str]


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    rates: RateSettings
    http: HttpSettings
    limits: LimitSettings
    vocabulary: VocabularySettings


def load_settings(path: str) -> Settings:
    with open(path, "r") as handle:
        raw = yaml.safe_load(handle)

    root = raw["interface"]
    vocabulary = root["vocabulary"]
    return Settings(
        topics=TopicSettings(**root["topics"]),
        rates=RateSettings(**root["rates"]),
        http=HttpSettings(**root["http"]),
        limits=LimitSettings(**root["limits"]),
        vocabulary=VocabularySettings(
            modes=frozenset(vocabulary["modes"]),
            authorities=frozenset(vocabulary["authorities"]),
            middle_modes=frozenset(vocabulary["middle_modes"]),
            middle_actions=frozenset(vocabulary["middle_actions"]),
        ),
    )
