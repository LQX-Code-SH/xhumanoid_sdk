#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Joint ID table loading (design doc appendix A).

The full arm/head/waist/leg ID map is not reproducible off-robot: it must
be exported from the real /robot_state and cross-checked against the SDK
reference doc. joints.yaml is the carrier; commands against unknown IDs
are rejected only in strict mode (default off, warn instead).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class JointMeta:
    name: str
    limits: tuple = ()          # (min, max) rad when known


class JointTable:
    """Lookup structure: group -> joint_id -> JointMeta."""

    def __init__(self, table: Optional[dict] = None, source: str = '',
                 strict: bool = False):
        self._by_group: Dict[str, Dict[int, JointMeta]] = {}
        self._source = source
        self.strict = strict
        if table:
            self._load(table)

    def _load(self, table: dict) -> None:
        for group, entries in table.items():
            metas = {}
            for entry in entries or []:
                metas[int(entry['id'])] = JointMeta(
                    name=str(entry.get('name', '')),
                    limits=tuple(entry.get('limits', ()) or ()),
                )
            self._by_group[group] = metas

    @property
    def is_empty(self) -> bool:
        return not any(self._by_group.values())

    def known(self, group: str, joint_id: int) -> bool:
        return joint_id in self._by_group.get(group, {})

    def meta(self, group: str, joint_id: int) -> Optional[JointMeta]:
        return self._by_group.get(group, {}).get(joint_id)

    def groups(self) -> list[str]:
        return sorted(self._by_group)

    def group_ids(self, group: str) -> set[int]:
        return set(self._by_group.get(group, {}))

    def __repr__(self) -> str:
        return (f'JointTable(source={self._source!r}, strict={self.strict}, '
                f'groups={self.groups()!r})')


def _default_candidates() -> list[str]:
    """Search paths for joints.yaml (package share, repo config, cwd)."""
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('tienkung_dex'), 'config', 'joints.yaml'))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    # <repo>/tienkung_dex/python/tienkung_dex/core -> <repo>/tienkung_dex/python/config
    candidates.append(os.path.normpath(
        os.path.join(here, '..', '..', 'config', 'joints.yaml')))
    candidates.append('config/joints.yaml')
    candidates.append('joints.yaml')
    return candidates


def load_joints_table(path: Optional[str] = None,
                      strict: bool = False) -> JointTable:
    """Load joints.yaml; an absent file yields an empty (non-strict) table."""
    try:
        import yaml
    except Exception:
        import warnings
        warnings.warn('tienkung_dex: pyyaml unavailable; joints table '
                      'loading disabled')
        return JointTable(strict=strict)
    candidates = [path] if path else _default_candidates()
    for candidate in candidates:
        if not candidate or not os.path.isfile(candidate):
            continue
        try:
            with open(candidate, 'r', encoding='utf-8') as fh:
                data = yaml.safe_load(fh) or {}
            return JointTable(data, source=candidate, strict=strict)
        except Exception as exc:  # pragma: no cover - bad user file
            import warnings
            warnings.warn(f'tienkung_dex: failed to load joints table '
                          f'{candidate}: {exc}')
    return JointTable(strict=strict)
