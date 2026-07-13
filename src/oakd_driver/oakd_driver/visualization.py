from dataclasses import dataclass
from typing import Tuple
import cv2

OBJECT_COLOR = (80, 200, 80)
TEXT_COLOR = (255, 255, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.45


@dataclass(frozen=True)
class Annotation:
    label: str
    confidence: float
    box: Tuple[float, float, float, float]


def clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


def draw_annotations(frame, annotations):
    height, width = frame.shape[:2]
    for annotation in annotations:
        xmin, ymin, xmax, ymax = annotation.box
        x0 = int(clamp(xmin) * width)
        y0 = int(clamp(ymin) * height)
        x1 = int(clamp(xmax) * width)
        y1 = int(clamp(ymax) * height)
        cv2.rectangle(frame, (x0, y0), (x1, y1), OBJECT_COLOR, 2)
        text = f"{annotation.label} {annotation.confidence * 100:.0f}%"
        cv2.putText(frame, text, (x0, max(y0 - 5, 12)), FONT, FONT_SCALE, TEXT_COLOR, 1, cv2.LINE_AA)
    return frame


def encode_jpeg(frame, quality: int):
    parameters = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    success, encoded = cv2.imencode(".jpg", frame, parameters)
    if not success:
        return None
    return encoded.tobytes()
