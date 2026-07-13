import base64
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def parse_pgm(data: bytes):
    if not data.startswith(b"P5"):
        raise ValueError("map image is not a binary PGM file")
    values = []
    index = 2
    while len(values) < 3:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b"#":
            while index < len(data) and data[index:index + 1] != b"\n":
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        values.append(int(data[start:index]))
    index += 1
    width, height, maximum = values
    pixels = data[index:index + width * height]
    if maximum != 255 or len(pixels) != width * height:
        raise ValueError("map image has an unsupported PGM format")
    return width, height, pixels


def load_navigation_map() -> dict:
    directory = Path(get_package_share_directory("navigation")) / "map"
    meta = yaml.safe_load((directory / "map.yaml").read_text())
    width, height, pixels = parse_pgm((directory / meta["image"]).read_bytes())
    return {
        "width": width,
        "height": height,
        "resolution": float(meta["resolution"]),
        "origin": [float(meta["origin"][0]), float(meta["origin"][1])],
        "pixels": base64.b64encode(pixels).decode(),
    }


def find_sound_names() -> list:
    try:
        directory = os.path.join(get_package_share_directory("sound_driver"), "wav")
        return sorted(os.path.splitext(name)[0] for name in os.listdir(directory) if name.lower().endswith(".wav"))
    except Exception:
        return []


def find_pages_directory() -> Path:
    try:
        installed = Path(get_package_share_directory("web_interface")) / "pages"
        if (installed / "index.html").exists():
            return installed
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "pages"
