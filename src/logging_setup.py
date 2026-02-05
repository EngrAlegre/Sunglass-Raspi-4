from __future__ import annotations

import logging
import os
from logging.handlers import RotatingFileHandler

from .config import LoggingConfig


def setup_logging(cfg: LoggingConfig) -> None:
    log_level = getattr(logging, cfg.level.upper(), logging.INFO)
    root = logging.getLogger()
    root.setLevel(log_level)

    for h in list(root.handlers):
        root.removeHandler(h)

    formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(log_level)
    stream_handler.setFormatter(formatter)
    root.addHandler(stream_handler)

    log_path = cfg.file
    log_dir = os.path.dirname(log_path) or "."
    os.makedirs(log_dir, exist_ok=True)

    file_handler = RotatingFileHandler(
        log_path, maxBytes=cfg.max_bytes, backupCount=cfg.backup_count, encoding="utf-8"
    )
    file_handler.setLevel(log_level)
    file_handler.setFormatter(formatter)
    root.addHandler(file_handler)
