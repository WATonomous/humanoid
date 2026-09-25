"""Minimal logger shim.

The vendored garment code (``garment_object.py``, ``success_checker_garment.py``)
originally imported ``lehome.utils.logger.get_logger``. This is a stripped-down
drop-in: console-only, no per-run log files, no project-root discovery.
"""
from __future__ import annotations

import logging
import sys
from typing import Optional

_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
_DATEFMT = "%Y-%m-%d %H:%M:%S"


def get_logger(name: Optional[str] = None, level: int = logging.INFO, **_kwargs) -> logging.Logger:
    """Return a console logger. Extra kwargs are accepted and ignored for compat."""
    logger = logging.getLogger(name if name else "humanoid_garment_fold")
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(_FORMAT, datefmt=_DATEFMT))
        logger.addHandler(handler)
        logger.propagate = False
    logger.setLevel(level)
    return logger
