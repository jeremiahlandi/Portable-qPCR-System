# Embeds version.txt (maintained by Release Please) as the FIRMWARE_VERSION macro.
from pathlib import Path

Import("env")  # noqa: F821 - provided by PlatformIO

version_file = Path(env.subst("$PROJECT_DIR")) / "version.txt"
version = version_file.read_text(encoding="utf-8").strip() if version_file.exists() else "0.0.0"
env.Append(CPPDEFINES=[("FIRMWARE_VERSION", env.StringifyMacro(version))])
