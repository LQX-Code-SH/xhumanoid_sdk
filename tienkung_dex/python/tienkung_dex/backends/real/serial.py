#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Robot serial number over /xsys/get_serial_number .

GetSerialNumber.Request is empty; the response carries serial_number.
Degrades like the optional services: get_serial_number() returns None on
any failure instead of raising.
"""

from __future__ import annotations

from typing import Optional

from tienkung_dex.core.base import SerialNumberBase

from . import _msgs


class RealSerialNumber(SerialNumberBase):
    """Service client; is_active reflects service reachability."""

    def __init__(self, node, service: str, logger=None):
        super().__init__(node)
        self._service = service
        self._log = logger
        self._cli = None
        self._srv_cls = None
        self._available = False

    def on_start(self) -> None:
        self._srv_cls, err = _msgs.serial_service()
        if self._srv_cls is None:
            if self._log is not None:
                self._log.error(f'serial: {err}; service inactive')
            return
        self._cli = self._node.create_client(self._srv_cls, self._service)
        self._available = self._cli.wait_for_service(timeout_sec=3.0)
        if self._log is not None:
            self._log.info(
                f'serial: client {self._service} '
                f'({"reachable" if self._available else "unreachable"})')

    def on_stop(self) -> None:
        self._cli = None
        self._available = False

    @property
    def is_active(self) -> bool:
        return self._available

    def get_serial_number(self, timeout: float = 5.0) -> Optional[str]:
        if self._cli is None or self._node is None:
            return None
        if not self._available and not self._cli.wait_for_service(
                timeout_sec=min(timeout, 3.0)):
            if self._log is not None:
                self._log.error('serial: service not reachable')
            return None
        self._available = True

        import rclpy
        future = self._cli.call_async(self._srv_cls.Request())
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout)
        if not future.done():
            if self._log is not None:
                self._log.error('serial: get_serial_number timed out')
            return None
        resp = future.result()
        return getattr(resp, 'serial_number', None)
