import os
import subprocess
import threading
import numpy as np

SAMPLE_RATE = 44100
FADE_DURATION = 0.006
LOOKAHEAD = 0.15
TEST_BEEP_DURATION = 0.2
CHIME_NOTES = ((523.25, 0.12), (659.25, 0.12), (783.99, 0.28))  # C5 E5 G5


def tone_pcm(frequency: float, duration: float, volume: float) -> bytes:
    times = np.arange(int(SAMPLE_RATE * duration)) / SAMPLE_RATE
    envelope = np.clip(np.minimum(times / FADE_DURATION, (duration - times) / FADE_DURATION), 0.0, 1.0)
    samples = volume * envelope * np.sin(2.0 * np.pi * frequency * times)
    return (samples * 32767).astype(np.int16).tobytes()


def raw_aplay():
    return subprocess.Popen(
        ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(SAMPLE_RATE), "-c", "1", "-"],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


class SoundPlayer:
    def __init__(self, beep_settings, directory: str):
        self.beep = beep_settings
        self.clips = {}
        if os.path.isdir(directory):
            for name in sorted(os.listdir(directory)):
                if name.lower().endswith(".wav"):
                    with open(os.path.join(directory, name), "rb") as handle:
                        self.clips[os.path.splitext(name)[0]] = handle.read()
        self.processes = []
        self.beep_stream = None
        self.beep_start = 0.0
        self.beep_written = 0

    def is_playing(self) -> bool:
        self.processes = [process for process in self.processes if process.poll() is None]
        return bool(self.processes)

    def update_beep(self, pattern, now: float):
        if pattern is None or self.is_playing():
            self.stop_beep()
            return

        if self.beep_stream is None or self.beep_stream.poll() is not None:
            self.beep_stream = raw_aplay()
            self.beep_start = now
            self.beep_written = 0

        target = int((now - self.beep_start) * SAMPLE_RATE) + int(LOOKAHEAD * SAMPLE_RATE)
        count = target - self.beep_written
        if count <= 0:
            return

        indices = np.arange(self.beep_written, target)
        times = indices / SAMPLE_RATE
        sine = np.sin(2.0 * np.pi * self.beep.frequency * times)

        period, on_time = pattern
        if on_time >= period:
            envelope = np.ones(count)
        else:
            phase = np.mod(times, period)
            fade_in = np.clip(phase / FADE_DURATION, 0.0, 1.0)
            fade_out = np.clip((on_time - phase) / FADE_DURATION, 0.0, 1.0)
            envelope = np.where(phase < on_time, np.minimum(fade_in, fade_out), 0.0)

        chunk = (self.beep.volume * envelope * sine * 32767).astype(np.int16)
        try:
            self.beep_stream.stdin.write(chunk.tobytes())
            self.beep_stream.stdin.flush()
        except (BrokenPipeError, ValueError):
            self.beep_stream = None
            return
        self.beep_written = target

    def stop_beep(self):
        if self.beep_stream is not None:
            stream = self.beep_stream
            self.beep_stream = None
            try:
                stream.stdin.close()
                stream.terminate()
            except Exception:
                pass
            self.processes.append(stream)

    def play_single_beep(self):
        self.play_raw(tone_pcm(self.beep.frequency, TEST_BEEP_DURATION, self.beep.volume))

    def play_chime(self):
        pcm = b"".join(tone_pcm(frequency, duration, self.beep.volume) for frequency, duration in CHIME_NOTES)
        self.play_raw(pcm)

    def play_clip(self, label: str) -> bool:
        clip = self.clips.get(label)
        if clip is None:
            return False
        if not self.is_playing():
            self.play_data(["aplay", "-q", "-"], clip)
        return True

    def play_raw(self, pcm: bytes):
        self.play_data(["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(SAMPLE_RATE), "-c", "1", "-"], pcm)

    def play_data(self, command, data: bytes):
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.processes.append(process)

        def feed():
            try:
                process.stdin.write(data)
                process.stdin.close()
            except Exception:
                pass

        threading.Thread(target=feed, daemon=True).start()
