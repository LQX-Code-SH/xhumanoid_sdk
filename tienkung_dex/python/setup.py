#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Setup for the tienkung_dex package (ament_python)."""

import os

from setuptools import find_packages, setup

package_name = 'tienkung_dex'


def package_files(directory: str):
    """Map a directory tree relative to share/<package_name>/ into data_files."""
    paths = []
    for (path, _directories, filenames) in os.walk(directory):
        if not filenames:
            continue
        install_dir = os.path.join('share', package_name, path)
        paths.append((install_dir, [os.path.join(path, f) for f in filenames]))
    return paths


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests', 'tests.*',
                                     'examples', 'examples.*']),
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        *package_files('launch'),
        *package_files('config'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Open-X-Humanoid',
    maintainer_email='noreply@example.com',
    description='TienkungDex robot facade library (real/sim/mock backends)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tienkung_dex_demo = tienkung_dex.demo_node:main',
        ],
    },
)
