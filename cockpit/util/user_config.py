#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 University of Oxford
##
## This file is part of Cockpit.
##
## Cockpit is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Cockpit is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Cockpit.  If not, see <http://www.gnu.org/licenses/>.

import logging
import os
import os.path
import pprint


_logger = logging.getLogger(__name__)


## @package user_config
# This module handles loading and saving changes to user configuration, which
# is used to remember individual users' settings (and a few global settings)
# for dialogs and the like.

## In-memory version of the config; program singleton.
_config = {}
_config_path = ""


def _load_config(fpath):
    """Open the config file and unserialize its contents."""
    config = {}
    try:
        with open(fpath, "r") as fh:
            config = eval(fh.read())
    except FileNotFoundError:
        config = {}
    except SyntaxError as e:
        _logger.error(
            "invalid or corrupted user config file '%s': %s", fpath, str(e)
        )
    return config


def _write_config(config, fpath):
    """Serialize the current config state for the specified user to the
    appropriate config file."""
    ## Use pprint instead of pickle to write the config files so that
    ## their contents are readable.
    printer = pprint.PrettyPrinter()
    if not printer.isreadable(config):
        raise RuntimeError("user config file has non-writable data")

    dirname = os.path.dirname(fpath)
    if not os.path.exists(dirname):
        os.makedirs(dirname)

    with open(fpath, "w") as fh:
        fh.write(printer.pformat(config))


def clear_all_values() -> None:
    """Discard all previous configuration and writes that to file."""
    global _config
    global _config_path
    _config = {}
    _write_config(_config, _config_path)


def get_value(key, default=None):
    """Retrieve the config value referenced by key.

    If key is not found, default is inserted and returned. If the value changed
    as a result of the lookup (because we wrote the default value to config),
    then write config back to the file.
    """
    global _config
    global _config_path
    try:
        result = _config[key]
    except KeyError:
        _config[key] = default
        _write_config(_config, _config_path)
        result = default
    return result


def set_value(key, value):
    """Set the entry referenced by key to the given value.

    Users are set as in get_value.
    """
    global _config
    global _config_path
    _config[key] = value
    _write_config(_config, _config_path)


def initialize(cockpit_config):
    global _config
    global _config_path
    _config_path = os.path.join(
        cockpit_config["global"].get("config-dir"), "config.py"
    )
    _config = _load_config(_config_path)
