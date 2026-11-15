"""USB HIL roundtrip; Mimir records native flight states and events."""
import sys
from flight import main

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:], hil=True, mission="roundtrip"))
