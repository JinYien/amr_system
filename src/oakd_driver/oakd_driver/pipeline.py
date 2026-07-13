import os
import tarfile
import tempfile
import depthai as dai
from oakd_driver.model import ModelConfig
from oakd_driver.settings import Settings

if not dai.__version__.startswith("3."):
    raise ImportError(f"depthai {dai.__version__} is not supported, this node uses the DepthAI v3 API\n")

COLOR_RESOLUTION = {
    "auto": None,
    "720p": (1280, 720),
    "800p": (1280, 800),
    "1080p": (1920, 1080),
    "4k": (3840, 2160),
}

SLOW_USB_SPEEDS = (dai.UsbSpeed.LOW, dai.UsbSpeed.FULL, dai.UsbSpeed.HIGH)


def lookup(table: dict, key: str, value: str):
    try:
        return table[value]
    except KeyError:
        valid = ", ".join(table.keys())
        raise ValueError(f"Pipeline option not valid ({key} : {value})\nOption: {valid}")


def pack_nn_archive(model: ModelConfig) -> dai.NNArchive:
    archive_path = os.path.join(tempfile.mkdtemp(prefix="oakd_driver_"), "model.tar")
    config_path = os.path.realpath(model.config_path)
    blob_path = os.path.realpath(model.blob_path)
    with tarfile.open(archive_path, "w") as tar:
        tar.add(config_path, arcname=os.path.basename(model.config_path))
        tar.add(blob_path, arcname=os.path.basename(model.blob_path))
    return dai.NNArchive(archive_path)


def apply_shaves(network, archive: dai.NNArchive, shaves: int):
    if shaves <= 0:
        return
    if archive.getModelType() != dai.ModelType.SUPERBLOB:
        raise ValueError(
            f"model.shaves ({shaves}) requires a superblob model, "
            f"got {archive.getModelType().name} (set model.shaves: 0 for fixed-shave blobs)"
        )
    network.setNNArchive(archive, shaves)


def build_pipeline(settings: Settings, model: ModelConfig, confidence_threshold: float):
    archive = pack_nn_archive(model)
    resize_mode = dai.ImgResizeMode.CROP if settings.device.keep_aspect_ratio else dai.ImgResizeMode.STRETCH

    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.Camera).build(
        dai.CameraBoardSocket.CAM_A,
        sensorResolution=lookup(COLOR_RESOLUTION, "device.resolution", settings.device.resolution),
    )

    network = pipeline.create(dai.node.DetectionNetwork).build(
        camera,
        archive,
        fps=settings.device.fps,
        resizeMode=resize_mode,
    )
    network.setConfidenceThreshold(confidence_threshold)
    apply_shaves(network, archive, settings.model.shaves)

    detection_queue = network.out.createOutputQueue(maxSize=4, blocking=False)
    preview_queue = None
    if settings.visualization.enable:
        preview_queue = network.passthrough.createOutputQueue(maxSize=4, blocking=False)
    return pipeline, detection_queue, preview_queue


def latest_packet(queue):
    latest = None
    while True:
        packet = queue.tryGet()
        if packet is None:
            return latest
        latest = packet


def slow_usb_speed(pipeline):
    speed = pipeline.getDefaultDevice().getUsbSpeed()
    if speed in SLOW_USB_SPEEDS:
        return speed.name
    return None
