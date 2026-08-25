#!/usr/bin/env python3
# Copyright 2026 jerry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Regenerate the ``QMainWindow State`` blob in coverage.rviz.

RViz stores the dock layout as ``QMainWindow::saveState()`` hex and restores it
by matching each dock's ``objectName``, which it sets to the panel name listed
under ``Panels``. Without the blob Qt splits the left column evenly, and Tool
Properties -- two rows of tool settings -- ends up as tall as the operator
panel.

Editing the layout by hand in RViz and saving the config would work too, but
that rewrites the whole file including every display and comment. This writes
only the one line.

Usage::

    QT_QPA_PLATFORM=offscreen python3 make_rviz_window_state.py [--write]

Without ``--write`` it prints the blob and the resulting dock heights so the
split can be checked before it is committed. Requires PyQt5, which is not a
package dependency: this is a maintenance tool, not part of the build.

A width smaller than a panel's own minimum is silently clamped by Qt -- the
Displays tree refuses to go below roughly 364 px -- so the width here is a
request, not a guarantee. The heights are the point of the file.
"""

import argparse
import pathlib
import re
import sys

from PyQt5.QtCore import QByteArray, Qt
from PyQt5.QtWidgets import (
    QApplication, QDockWidget, QMainWindow, QToolBar, QWidget)

# Top to bottom in the left dock. These must match the ``Name`` fields under
# ``Panels`` in coverage.rviz: Qt restores a dock by objectName and silently
# ignores one it cannot find.
PANELS = ['Displays', 'Tool Properties', 'Coverage Task']
# Tool Properties is needed only while adjusting an RViz tool. Keep it one
# click away but do not spend the default task-planning height on it.
HIDDEN_PANELS = {'Tool Properties'}

# Of the roughly 764 px the left column gets in an 1200x850 window. Displays
# needs room for the display tree, Tool Properties holds two rows, and the rest
# goes to the operator panel so its messages do not have to be scrolled.
HEIGHTS = [300, 90, 374]
# Measured, not derived. This window has no menu bar or status bar and RViz's
# does, so Qt restores a column about 22 px wider than the number saved here.
# 342 lands on the 364 px the left dock occupies without a saved state, which
# keeps this change about the vertical split and nothing else.
WIDTH = 342

WINDOW = (1200, 850)
CONFIG = pathlib.Path(__file__).resolve().parent.parent / 'rviz' / 'coverage.rviz'


def build(width, heights):
    """Return a QMainWindow laid out the way RViz lays out this config."""
    window = QMainWindow()
    window.resize(*WINDOW)
    window.setCentralWidget(QWidget())

    # RViz's tool toolbar is part of the saved state; recreating it keeps the
    # restored state from dropping the toolbar row.
    toolbar = QToolBar('Tools')
    toolbar.setObjectName('Tools')
    window.addToolBar(Qt.TopToolBarArea, toolbar)

    docks = []
    for name in PANELS:
        dock = QDockWidget(name, window)
        dock.setObjectName(name)
        dock.setWidget(QWidget())
        window.addDockWidget(Qt.LeftDockWidgetArea, dock)
        if docks:
            window.splitDockWidget(docks[-1], dock, Qt.Vertical)
        docks.append(dock)

    window.show()
    QApplication.processEvents()
    window.resizeDocks(docks, [width] * len(docks), Qt.Horizontal)
    window.resizeDocks(docks, heights, Qt.Vertical)
    for dock in docks:
        dock.setVisible(dock.objectName() not in HIDDEN_PANELS)
    QApplication.processEvents()
    return window, docks


def restored_heights(blob):
    """Lay the blob back out, so what is committed is what was measured."""
    window, docks = build(WIDTH, HEIGHTS)
    assert window.restoreState(QByteArray.fromHex(blob.encode())), \
        'the generated state does not restore'
    QApplication.processEvents()
    return [(dock.objectName(), dock.width(), dock.height()) for dock in docks]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--write', action='store_true',
        help='replace the QMainWindow State line in coverage.rviz')
    arguments = parser.parse_args()

    # Kept in a name on purpose: an unreferenced QApplication is collected
    # immediately and every widget after it fails to construct.
    application = QApplication([])
    assert application is not None
    window, _ = build(WIDTH, HEIGHTS)
    blob = bytes(window.saveState().toHex()).decode()

    for name, width, height in restored_heights(blob):
        print('{:16s} {}x{}'.format(name, width, height), file=sys.stderr)

    if not arguments.write:
        print(blob)
        return 0

    text = CONFIG.read_text()
    replaced, count = re.subn(
        r'  QMainWindow State: .*', '  QMainWindow State: ' + blob, text)
    if count != 1:
        print(
            'expected exactly one QMainWindow State line in {}, found {}'
            .format(CONFIG, count), file=sys.stderr)
        return 1
    CONFIG.write_text(replaced)
    print('updated {}'.format(CONFIG), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
