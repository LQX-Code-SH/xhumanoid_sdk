#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real backend: adapters over the TienKung DEX SDK topics/services.

Only modules in this package import vendor message packages, and always
lazily inside functions (design doc §3 constraint 2), so importing
tienkung_dex itself never requires the robot host message packages.
"""

from .factory import RealBackendFactory

__all__ = ['RealBackendFactory']
