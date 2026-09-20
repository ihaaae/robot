"""Build hook: compile the Python bridge at install time, when that is possible.

``pip install -e .`` on the robot's board should leave a working ``shensi_robot.sdk``, not a
package that needs a second command before it can talk to the robot.

The bridge needs a C++ compiler, Eigen and the SDK tree, none of which pip can install, so
this is deliberately **best effort**: if any of them is missing, or the compiler refuses, the
install still succeeds and prints what to run instead. Set ``SHENSI_SKIP_BRIDGE=1`` to skip it
entirely.

The output goes where ``python/build_bridge.sh`` puts it (``python/build/``), which the module
looks in for a checkout. A *non-editable* install copies the package elsewhere and cannot see
that directory; there, point ``JUXIE_SDK_BRIDGE`` at the file.
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

from setuptools import setup
from setuptools.command.build_py import build_py


class build_py_with_bridge(build_py):
    """build_py, plus the bridge."""

    def run(self) -> None:  # noqa: D102 - setuptools calls this
        self._build_bridge()
        super().run()

    def _build_bridge(self) -> None:
        if os.environ.get("SHENSI_SKIP_BRIDGE"):
            return
        repo = Path(__file__).resolve().parent
        script = repo / "python" / "build_bridge.sh"
        if not script.is_file():
            # An sdist or a vendored copy without the bridge source: nothing to build, and
            # nothing worth warning about.
            return
        try:
            # errors="replace": compiler output is not guaranteed to be decodable, and a
            # UnicodeDecodeError is neither an OSError nor a SubprocessError, so it would
            # escape this best-effort hook and fail the install.
            done = subprocess.run(
                [str(script)], capture_output=True, text=True, errors="replace",
                timeout=300, cwd=str(repo),
            )
        except (OSError, subprocess.SubprocessError) as error:
            print(f"warning: could not run python/build_bridge.sh ({error})")
            return
        if done.returncode != 0:
            print("warning: the Python bridge was not built. Its output follows, then run "
                  "./python/build_bridge.sh yourself once the toolchain is installed.")
            for line in (done.stderr or done.stdout).strip().splitlines():
                print(f"  {line}")
            return
        built = next((line for line in done.stdout.splitlines()
                      if line.startswith("wrote :")), "")
        print(f"built the Python bridge ({built.split('wrote :', 1)[-1].strip() or 'ok'}). "
              "An editable install finds it automatically; otherwise set JUXIE_SDK_BRIDGE.")


setup(cmdclass={"build_py": build_py_with_bridge})
