"""Repository-local access to the shared vehicle physical configuration."""

from pathlib import Path
import sys


_PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / "src" / "vehicle_config"
if str(_PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SOURCE))

from vehicle_config import load_vehicle_physical_config  # noqa: E402


VEHICLE_PHYSICAL = load_vehicle_physical_config()


def main(arguments: list[str] | None = None) -> int:
    argv = sys.argv[1:] if arguments is None else arguments
    fields = {
        "tracking_point_x_m": VEHICLE_PHYSICAL.tracking_point_x_m,
        "tracking_point_y_m": VEHICLE_PHYSICAL.tracking_point_y_m,
    }
    if len(argv) != 1 or argv[0] not in fields:
        choices = ", ".join(sorted(fields))
        print(f"usage: vehicle_physical.py {{{choices}}}", file=sys.stderr)
        return 2
    print(fields[argv[0]])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
