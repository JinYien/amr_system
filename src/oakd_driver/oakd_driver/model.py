import json
import os
from dataclasses import dataclass
from typing import Tuple
from oakd_driver.settings import ModelSettings


@dataclass(frozen=True)
class ModelConfig:
    blob_path: str
    config_path: str
    labels: Tuple[str, ...]

    def label_name(self, index: int) -> str:
        if 0 <= index < len(self.labels):
            return self.labels[index]
        return str(index)


def load_model_config(settings: ModelSettings) -> ModelConfig:
    if not os.path.isfile(settings.blob_path):
        raise FileNotFoundError(f"Model blob not found ({settings.blob_path})")
    if not os.path.isfile(settings.config_path):
        raise FileNotFoundError(f"Model config not found ({settings.config_path})")

    with open(settings.config_path, "r") as handle:
        config = json.load(handle)

    if "model" not in config:
        if "nn_config" in config:
            raise ValueError(f"Legacy model config is not supported by DepthAI v3 ({settings.config_path})")
        raise ValueError(f"Model config format not recognized ({settings.config_path})")

    model = config["model"]
    blob_name = model.get("metadata", {}).get("path", "")
    if blob_name and blob_name != os.path.basename(settings.blob_path):
        raise ValueError(f"Blob not found ({blob_name} -> {os.path.basename(settings.blob_path)})")

    labels = tuple(model["heads"][0]["metadata"].get("classes", ()) or ())

    return ModelConfig(
        blob_path=settings.blob_path,
        config_path=settings.config_path,
        labels=labels,
    )
