Import("env")

from datetime import datetime, timezone
import subprocess


def git_value(args, fallback):
    try:
        return subprocess.check_output(
            ["git", *args], cwd=env.subst("$PROJECT_DIR"), text=True,
            stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return fallback


timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
revision = git_value(["rev-parse", "--short=8", "HEAD"], "nogit")
dirty = git_value(["status", "--porcelain"], "")
version = f"{timestamp}-{revision}{'-dirty' if dirty else ''}"

env.Append(CPPDEFINES=[("BUILD_VERSION", env.StringifyMacro(version))])
