#!/usr/bin/env python3
"""Repository-local entry point; no editable install required."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from physics_demo.cli import main

if __name__ == "__main__":
    main()
