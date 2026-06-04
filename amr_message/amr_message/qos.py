#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

RELIABILITY = {"best_effort": QoSReliabilityPolicy.BEST_EFFORT, "reliable": QoSReliabilityPolicy.RELIABLE}
DURABILITY = {"volatile": QoSDurabilityPolicy.VOLATILE, "transient_local": QoSDurabilityPolicy.TRANSIENT_LOCAL}
HISTORY = {"keep_last": QoSHistoryPolicy.KEEP_LAST, "keep_all": QoSHistoryPolicy.KEEP_ALL}
QOS_CONFIG_PATH = os.path.join(get_package_share_directory("amr_message"), "config", "qos.yaml")


def lookup(table: dict, key: str, value: str):
    try:
        return table[value]
    except KeyError:
        valid = ", ".join(table.keys())
        raise ValueError(f"qos.yaml の {key} が不正です: '{value}'（有効な値: {valid}）")


def load_qos_profile() -> QoSProfile:
    with open(QOS_CONFIG_PATH, "r") as handle:
        config = yaml.safe_load(handle)

    qos = config["qos"]
    return QoSProfile(
        reliability=lookup(RELIABILITY, "reliability", qos["reliability"]),
        durability=lookup(DURABILITY, "durability", qos["durability"]),
        history=lookup(HISTORY, "history", qos["history"]),
        depth=int(qos["depth"]),
    )


QOS_PROFILE = load_qos_profile()