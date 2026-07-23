#!/usr/bin/env python3
"""Build the firmware via `idf.py build`, using $IDF_PATH from the environment.

Avoids needing `. $IDF_PATH/export.sh` sourced in the calling shell: invokes
`$IDF_PATH/tools/idf.py` directly with the current `sys.executable` (this project's
devcontainer already has the ESP-IDF Python env active), and just streams idf.py's own
stdout/stderr straight through.

Exit code is idf.py's own (0 on a clean build, non-zero on any build failure).
"""

import os
import subprocess
import sys


def main() -> int:
    idf_path = os.environ.get("IDF_PATH")
    if not idf_path:
        print("[build] $IDF_PATH is not set — run inside the ESP-IDF devcontainer "
              "(or `. $IDF_PATH/export.sh` first)", file=sys.stderr)
        return 1

    idf_py = os.path.join(idf_path, "tools", "idf.py")
    if not os.path.exists(idf_py):
        print(f"[build] {idf_py} not found — is $IDF_PATH correct?", file=sys.stderr)
        return 1

    project_dir = os.path.dirname(os.path.abspath(__file__))
    cmd = [sys.executable, idf_py, "build"]
    print(f"[build] {' '.join(cmd)} (in {project_dir})", file=sys.stderr)
    return subprocess.run(cmd, cwd=project_dir).returncode


if __name__ == "__main__":
    sys.exit(main())
