"""USB HIL roundtrip; Mimir records native flight states and events."""
import sys
from flight import main

if __name__ == "__main__":
    raise SystemExit(main(["--hil", "--mission", "roundtrip", *sys.argv[1:]]))
