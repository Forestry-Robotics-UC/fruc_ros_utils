#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Author: Duda Andrada
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# License: MIT License (open source, free to modify and redistribute)
# Repository: fruc_ros_utils
#
# Description:
#   Lightweight logging utilities with sane defaults for CLI and ROS bag scripts.

"""Logging helpers with optional colorized console output."""

import logging
import os
import sys
from typing import Optional

class ColorFormatter(logging.Formatter):
    COLORS = {
        "DEBUG": "\033[36m",    # Cyan
        "INFO": "\033[92m",     # Bright Green
        "WARNING": "\033[93m",  # Bright Yellow
        "ERROR": "\033[91m",    # Bright Red
        "CRITICAL": "\033[97;41m", # White on Red background
    }
    RESET = "\033[0m"
    DIM = "\033[90m"
    CYAN = "\033[96m"

    def format(self, record: logging.LogRecord) -> str:
        color = self.COLORS.get(record.levelname.strip(), "")
        levelname_color = f"{color}{record.levelname}{self.RESET}"

        # Timestamp in dim white, function name in cyan
        ts_color = f"{self.DIM}{self.formatTime(record, self.datefmt)}{self.RESET}"
        func_color = f"{self.CYAN}{record.funcName}{self.RESET}"

        # Debug shows function name, others don’t
        if record.levelno == logging.DEBUG:
            msg_fmt = f"{ts_color} [{levelname_color}] {record.name}.{func_color}: %(message)s"
        else:
            msg_fmt = f"{ts_color} [{levelname_color}] {record.name}: %(message)s"

        formatter = logging.Formatter(msg_fmt, datefmt="%Y-%m-%d %H:%M:%S")
        return formatter.format(record)


def get_logger(name: str,
               level: str = "INFO",
               log_file: Optional[str] = None) -> logging.Logger:
    logger = logging.getLogger(name)
    log_level = getattr(logging, level.upper(), logging.INFO)
    logger.setLevel(log_level)   # <-- always update the level

    if not logger.handlers:
        # Console handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(ColorFormatter())
        logger.addHandler(console_handler)

        # File handler if requested
        if log_file:
            os.makedirs(os.path.dirname(log_file), exist_ok=True)
            file_handler = logging.FileHandler(log_file, mode="a")
            file_handler.setFormatter(
                logging.Formatter(
                    "%(asctime)s [%(levelname)s] %(name)s.%(funcName)s: %(message)s",
                    datefmt="%Y-%m-%d %H:%M:%S"
                )
            )
            logger.addHandler(file_handler)

    return logger
