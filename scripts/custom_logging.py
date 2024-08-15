#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Maintainer: Duda Andrada <duda.andrada@isr.uc.pt>
# Written: August 2024
# License: This code is licensed under the MIT License.
#
# Program: Custom Logging Script
# Purpose: Keep logging messages consistent throughout my scripts.

import logging

# Define color codes for different log levels
LOG_COLORS = {
    'DEBUG': '\033[94m',    # Blue
    'INFO': '\033[92m',     # Green
    'WARNING': '\033[93m',  # Yellow
    'ERROR': '\033[91m',    # Red
    'CRITICAL': '\033[95m', # Magenta
    'RESET': '\033[0m'      # Reset color
}

# Custom formatter to include colors and enclosing pattern
class CustomFormatter(logging.Formatter):
    def format(self, record):
        log_color = LOG_COLORS.get(record.levelname, LOG_COLORS['RESET'])
        record.msg = f'{log_color}\n\n[*** {record.msg} ***]\n{LOG_COLORS["RESET"]}'  # Enclosing pattern
        return super().format(record)

def setup_custom_logger():
    # Configure the logging with the custom formatter
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler()
        ]
    )

    # Set the custom formatter
    for handler in logging.root.handlers:
        handler.setFormatter(CustomFormatter('%(asctime)s - %(levelname)s - %(message)s'))

    return logging.getLogger()
