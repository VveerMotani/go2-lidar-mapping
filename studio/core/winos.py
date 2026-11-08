"""Everything that touches Windows itself.

A web page is not allowed to open Explorer, show a folder dialog or start a
program; that is the reason Studio has a local server at all. Every function
here runs a fixed command with the paths passed as separate arguments, never
a string the page wrote.
"""
import glob
import os
import shutil
import subprocess
import sys

from .applog import log

NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def run(args, timeout=20):
    """Run and return (exit code, output). Never raises on failure."""
    try:
        p = subprocess.run(args, capture_output=True, timeout=timeout,
                           creationflags=NO_WINDOW)
        out = p.stdout + p.stderr
        # wsl.exe writes UTF-16 for its own messages and UTF-8 for Linux ones.
        text = out.decode("utf-16-le", "replace") if out[1:2] == b"\x00" \
            else out.decode("utf-8", "replace")
        return p.returncode, text.replace("\x00", "").strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return -1, str(e)


# ------------------------------------------------------------------ files
def show_in_folder(path):
    """Open Explorer with `path` selected (or the folder, if it is one)."""
    path = os.path.normpath(path)
    if not os.path.exists(path):
        return False, "not found: " + path
    if os.path.isdir(path):
        subprocess.Popen(["explorer.exe", path], creationflags=NO_WINDOW)
    else:
        # explorer wants "/select," and the path as ONE argument.
        subprocess.Popen('explorer.exe /select,"%s"' % path, creationflags=NO_WINDOW)
    log("inf", "os", "explorer /select %s" % path)
    return True, None


def open_file(path, program=None):
    """Open `path` with `program`, or with whatever Windows associates."""
    if not os.path.exists(path):
        return False, "not found: " + path
    try:
        if program:
            subprocess.Popen([program, path], creationflags=NO_WINDOW)
        else:
            os.startfile(path)                 # noqa: S606, the user's own file
    except OSError as e:
        return False, str(e)
    log("inf", "os", "open %s%s" % (path, " with " + program if program else ""))
    return True, None


def open_url(url):
    os.startfile(url)
    log("inf", "os", "open " + url)


def pick_folder(initial=""):
    """A real Windows folder dialog. Returns the chosen folder, or None.

    Two things this used to get wrong (2026-09-26): the starting folder was
    passed after -Command, where PowerShell glues it onto the script and the
    whole script fails to parse, so no dialog ever opened; and stderr was read
    together with stdout, so an error message could come back as the "folder".
    Now the starting folder travels in an environment variable, only stdout is
    read, and the answer must be a folder that exists.
    """
    ps = (
        "Add-Type -AssemblyName System.Windows.Forms;"
        "$d = New-Object System.Windows.Forms.FolderBrowserDialog;"
        "$d.Description = 'Choose a folder';"
        "$d.ShowNewFolderButton = $true;"
        "if ($env:GO2_PICK_INITIAL -and (Test-Path -LiteralPath $env:GO2_PICK_INITIAL)) { $d.SelectedPath = $env:GO2_PICK_INITIAL };"
        # An invisible top-most owner, so the dialog opens in front of Studio
        # rather than behind it.
        "$f = New-Object System.Windows.Forms.Form -Property @{TopMost=$true; ShowInTaskbar=$false};"
        "if ($d.ShowDialog($f) -eq [System.Windows.Forms.DialogResult]::OK) { [Console]::Out.Write($d.SelectedPath) }"
    )
    env = dict(os.environ, GO2_PICK_INITIAL=initial or "")
    try:
        p = subprocess.run(["powershell", "-NoProfile", "-STA", "-Command", ps],
                           capture_output=True, timeout=600, env=env, creationflags=NO_WINDOW)
    except (OSError, subprocess.TimeoutExpired) as e:
        log("wrn", "os", "folder dialog failed: %s" % e)
        return None
    chosen = p.stdout.decode("utf-8", "replace").strip()
    if p.stderr.strip():
        log("wrn", "os", "folder dialog said: %s" % p.stderr.decode("utf-8", "replace").strip()[:300])
    if chosen and os.path.isdir(chosen):
        log("inf", "os", "folder chosen: %s" % chosen)
        return chosen
    return None


def free_bytes(path):
    """Free space on the drive holding `path` (or its nearest existing parent)."""
    p = os.path.abspath(path)
    while p and not os.path.exists(p):
        parent = os.path.dirname(p)
        if parent == p:
            break
        p = parent
    try:
        u = shutil.disk_usage(p)
        return u.free, u.total
    except OSError:
        return None, None


def in_onedrive(path):
    """True when `path` is inside a OneDrive-synced folder."""
    p = os.path.abspath(path).lower()
    for var in ("OneDrive", "OneDriveConsumer", "OneDriveCommercial"):
        root = os.environ.get(var)
        if root and p.startswith(os.path.abspath(root).lower()):
            return True
    return "\\onedrive" in p


# --------------------------------------------------------------- programs
def _installed(name_part):
    """The .exe of an installed program, from Windows' own list of installed
    apps (what Settings > Apps shows). Foxglove, for one, installs itself as
    %LOCALAPPDATA%\\Programs\\desktop\\Foxglove.exe, which no guess would find."""
    try:
        import winreg
    except ImportError:
        return None
    roots = [(winreg.HKEY_CURRENT_USER, r"Software\Microsoft\Windows\CurrentVersion\Uninstall"),
             (winreg.HKEY_LOCAL_MACHINE, r"Software\Microsoft\Windows\CurrentVersion\Uninstall"),
             (winreg.HKEY_LOCAL_MACHINE, r"Software\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall")]
    for hive, key in roots:
        try:
            k = winreg.OpenKey(hive, key)
        except OSError:
            continue
        for i in range(winreg.QueryInfoKey(k)[0]):
            try:
                sub = winreg.OpenKey(k, winreg.EnumKey(k, i))
                name = winreg.QueryValueEx(sub, "DisplayName")[0]
            except OSError:
                continue
            if name_part.lower() not in str(name).lower():
                continue
            for field in ("DisplayIcon", "InstallLocation"):
                try:
                    v = str(winreg.QueryValueEx(sub, field)[0]).split(",")[0].strip('"')
                except OSError:
                    continue
                if v.lower().endswith(".exe") and os.path.isfile(v) and "unins" not in v.lower():
                    return v
                if os.path.isdir(v):
                    for exe in glob.glob(os.path.join(v, "*.exe")):
                        if name_part.lower() in os.path.basename(exe).lower():
                            return exe
    return None


def find_program(kind, configured=""):
    """Path to Foxglove or CloudCompare, or None."""
    if configured and os.path.isfile(configured):
        return configured
    found = _installed("Foxglove" if kind == "foxglove" else "CloudCompare")
    if found:
        return found
    local = os.environ.get("LOCALAPPDATA", "")
    pf = [os.environ.get("ProgramFiles", r"C:\Program Files"),
          os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")]
    if kind == "foxglove":
        cands = [os.path.join(local, "Programs", "foxglove-studio", "Foxglove Studio.exe"),
                 os.path.join(local, "Programs", "foxglove-studio", "Foxglove.exe"),
                 os.path.join(local, "Programs", "Foxglove", "Foxglove.exe")]
        cands += [os.path.join(p, "Foxglove", "Foxglove.exe") for p in pf]
        cands += glob.glob(os.path.join(local, "Programs", "*oxglove*", "*.exe"))
    else:
        cands = [os.path.join(p, "CloudCompare", "CloudCompare.exe") for p in pf]
    for c in cands:
        if os.path.isfile(c) and "uninstall" not in c.lower():
            return c
    return None


# ----------------------------------------------------------------- engines
def wsl_distro_ok(distro):
    code, out = run(["wsl.exe", "-l", "-q"], timeout=15)
    return code == 0 and distro.lower() in [l.strip().lower() for l in out.splitlines()]


def docker_desktop_exe():
    for base in (os.environ.get("ProgramFiles", r"C:\Program Files"),):
        p = os.path.join(base, "Docker", "Docker", "Docker Desktop.exe")
        if os.path.isfile(p):
            return p
    return None


def start_docker():
    exe = docker_desktop_exe()
    if not exe:
        return False
    subprocess.Popen([exe], creationflags=NO_WINDOW)
    log("inf", "engine", "started Docker Desktop")
    return True


def image_created(image):
    code, out = run(["docker", "image", "inspect", "--format", "{{.Created}}", image], timeout=15)
    return out[:19].replace("T", " ") if code == 0 else None


def docker_state(image):
    """(installed, running, image_present)."""
    if not shutil.which("docker"):
        return False, False, False
    code, _ = run(["docker", "info", "--format", "{{.ServerVersion}}"], timeout=15)
    if code != 0:
        return True, False, False
    code, _ = run(["docker", "image", "inspect", image], timeout=15)
    return True, True, code == 0


# ----------------------------------------------------------------- network
ADDR_ON = ("New-NetIPAddress -InterfaceAlias '{alias}' -IPAddress 192.168.123.99 "
           "-PrefixLength 24")
ADDR_OFF = ("Remove-NetIPAddress -InterfaceAlias '{alias}' -IPAddress 192.168.123.99 "
            "-Confirm:$false")


def set_robot_address(on, alias="Ethernet"):
    """Ask Windows (UAC prompt) to add or remove 192.168.123.99.

    A secondary address, so the adapter's normal DHCP address stays and the
    same port still reaches the internet on an ordinary network. Returns
    (started, command) -- the command is what the page shows for doing it by
    hand when the prompt is refused.
    """
    cmd = (ADDR_ON if on else ADDR_OFF).format(alias=alias.replace("'", "''"))
    inner = cmd.replace('"', '\\"')
    ps = ("Start-Process powershell -Verb RunAs -WindowStyle Hidden -Wait "
          "-ArgumentList '-NoProfile','-Command',\"%s\"" % inner)
    code, out = run(["powershell", "-NoProfile", "-Command", ps], timeout=120)
    log("inf" if code == 0 else "wrn", "os",
        "admin request (%s address): exit %s %s" % ("add" if on else "remove", code, out[:200]))
    return code == 0, cmd


def edge_path():
    for base in (os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
                 os.environ.get("ProgramFiles", r"C:\Program Files")):
        p = os.path.join(base, "Microsoft", "Edge", "Application", "msedge.exe")
        if os.path.isfile(p):
            return p
    return shutil.which("msedge")


def is_windows():
    return sys.platform == "win32"
