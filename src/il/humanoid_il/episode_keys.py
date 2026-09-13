"""Keyboard episode controls (lehome dataset_record.py key bindings)."""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class EpisodeFlags:
    """Shared flags between keyboard thread and record loop."""

    start: bool = False
    success: bool = False
    remove: bool = False
    abort: bool = False


class EpisodeKeyboard:
    """
    Register keys:
      I — start / resume logging frames for the current episode
      O — finish episode (save on next loop tick, only if started)
      P — discard buffered frames and re-record current episode (only if started)
      ESC — stop session and finalize dataset
    """

    def __init__(self, flags: EpisodeFlags) -> None:
        self.flags = flags
        self._listener = None
        self._pynput = None

    def start(self) -> bool:
        try:
            from pynput import keyboard as kb
        except ImportError:
            logger.warning(
                "pynput not installed; keyboard controls disabled. "
                "Use timed mode (--episode_time_s) or pip install pynput."
            )
            return False

        self._pynput = kb

        def on_press(key):
            try:
                if key == kb.Key.esc:
                    self.flags.abort = True
                    logger.info("[ESC] Abort recording.")
                    return False
                char = getattr(key, "char", None)
                if char is None:
                    return
                c = char.lower()
                if c == "i":
                    self.flags.start = True
                    logger.info("[I] Recording frames for this episode.")
                elif c == "o":
                    if self.flags.start:
                        self.flags.success = True
                        logger.info("[O] Mark episode complete.")
                    else:
                        print("\n[INFO] [RECORD] [O] Ignored: Recording is not active. Press 'I' to start recording first.")
                elif c == "p":
                    if self.flags.start:
                        self.flags.remove = True
                        logger.info("[P] Discard episode buffer.")
                    else:
                        print("\n[INFO] [RECORD] [P] Ignored: Recording is not active. Press 'I' to start recording first.")
            except Exception:
                logger.exception("Keyboard callback error")

        self._listener = kb.Listener(on_press=on_press)
        self._listener.daemon = True
        self._listener.start()
        return True

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()
            self._listener = None
