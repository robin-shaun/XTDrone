#!/usr/bin/env python
# coding: utf-8


# Author : Rahul Bhadani
# Initial Date: Nov 18, 2020
# License: MIT License

# logging

#   Permission is hereby granted, free of charge, to any person obtaining
#   a copy of this software and associated documentation files
#   (the "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject
#   to the following conditions:

#   The above copyright notice and this permission notice shall be
#   included in all copies or substantial portions of the Software.

#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
#   ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
#   TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#   PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
#   SHALL THE AUTHORS, COPYRIGHT HOLDERS OR ARIZONA BOARD OF REGENTS
#   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
#   AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
#   OR OTHER DEALINGS IN THE SOFTWARE.

import logging
import sys
##from contextlib import AbstractContextManager as ACM


def configure_logworker():
    """
    Logging Configuration
    """

    logworker = logging.getLogger('')
    logworker.setLevel(logging.INFO)
    logworker.propagate = False
    logworker.handlers = []

    log_format = '[%(asctime)s] (%(name)s) %(levelname)s: %(message)s'
    log_date_format = '%Y_%m_%d_%H_%M_%S'
    formatter = logging.Formatter(log_format, log_date_format)

    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(formatter)
    logworker.addHandler(handler)

    return logworker
