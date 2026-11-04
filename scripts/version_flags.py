# Embeds the firmware version as build macros (issue #41):
#   FIRMWARE_VERSION  the release number from version.txt (maintained by Release Please)
#   FIRMWARE_BUILD    the same, plus SemVer build metadata for anything built past the last
#                     release tag: "<version>+<commits>.g<sha>[.dirty]". At an exact tag the
#                     two are equal. Without git (source tarball) FIRMWARE_BUILD == version.
import subprocess
from pathlib import Path

Import("env")  # noqa: F821 - provided by PlatformIO

project_dir = Path(env.subst("$PROJECT_DIR"))
version_file = project_dir / "version.txt"
version = version_file.read_text(encoding="utf-8").strip() if version_file.exists() else "0.0.0"


def git_metadata() -> str:
    """Return "+N.gSHA[.dirty]" for a tree past its last tag, "" at an exact clean tag."""
    try:
        out = subprocess.run(
            ["git", "describe", "--tags", "--long", "--dirty", "--always", "--match", "v*"],
            cwd=project_dir,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    if out.returncode != 0:
        return ""
    describe = out.stdout.strip()
    dirty = describe.endswith("-dirty")
    if dirty:
        describe = describe[: -len("-dirty")]
    parts = describe.rsplit("-", 2)
    if len(parts) == 3 and parts[2].startswith("g"):
        commits, sha = parts[1], parts[2]
    else:  # no tag reachable: describe gave a bare sha
        commits, sha = "0", "g" + describe
    if commits == "0" and not dirty:
        return ""
    return "+" + commits + "." + sha + (".dirty" if dirty else "")


build = version + git_metadata()
env.Append(
    CPPDEFINES=[
        ("FIRMWARE_VERSION", env.StringifyMacro(version)),
        ("FIRMWARE_BUILD", env.StringifyMacro(build)),
    ]
)
print("version_flags: FIRMWARE_VERSION=%s FIRMWARE_BUILD=%s" % (version, build))
