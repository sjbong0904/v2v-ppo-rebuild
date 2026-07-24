"""Build map.net.xml from nod/edg/con sources via netconvert."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys


HERE = os.path.dirname(os.path.abspath(__file__))


def find_netconvert() -> str:
    env_home = os.environ.get("SUMO_HOME", "").strip()
    candidates = [
        shutil.which("netconvert"),
        os.path.join(env_home, "bin", "netconvert.exe") if env_home else "",
        os.path.join(env_home, "bin", "netconvert") if env_home else "",
        r"C:\Program Files (x86)\Eclipse\Sumo\bin\netconvert.exe",
        r"C:\Program Files\Eclipse\Sumo\bin\netconvert.exe",
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    raise FileNotFoundError(
        "netconvert not found. Install SUMO and set SUMO_HOME, or add bin to PATH."
    )


def main() -> None:
    netconvert = find_netconvert()
    out_net = os.path.join(HERE, "map.net.xml")
    cmd = [
        netconvert,
        "--node-files", os.path.join(HERE, "map.nod.xml"),
        "--edge-files", os.path.join(HERE, "map.edg.xml"),
        "--connection-files", os.path.join(HERE, "map.con.xml"),
        "--output-file", out_net,
        "--no-turnarounds", "true",
        "--offset.disable-normalization", "true",
    ]
    print("running:", " ".join(cmd))
    subprocess.run(cmd, check=True)
    print(f"wrote: {out_net}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
