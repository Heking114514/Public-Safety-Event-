#!/usr/bin/env python3

from pathlib import Path
import runpy


if __name__ == "__main__":
    editor = (
        Path(__file__).resolve().parent
        / "src"
        / "visual_navigation"
        / "scripts"
        / "route_editor.py"
    )
    runpy.run_path(str(editor), run_name="__main__")
