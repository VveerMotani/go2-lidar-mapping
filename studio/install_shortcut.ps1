# Put a "Go2 Studio" icon on the desktop. Run once:
#
#   powershell -ExecutionPolicy Bypass -File studio\install_shortcut.ps1
#   ... -StartMenu      also add it to the Start menu
#
# Only makes shortcuts to "Go2 Studio.vbs" in this folder. No admin rights
# needed; delete the shortcut to undo it. If the repo folder moves, run it again.
param([switch] $StartMenu)

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$target = Join-Path $here "Go2 Studio.vbs"
$icon = Join-Path $here "studio.ico"
$sh = New-Object -ComObject WScript.Shell
$places = @([Environment]::GetFolderPath("Desktop"))
if ($StartMenu) { $places += [Environment]::GetFolderPath("Programs") }
foreach ($dir in $places) {
    $lnk = $sh.CreateShortcut((Join-Path $dir "Go2 Studio.lnk"))
    $lnk.TargetPath = "$env:WINDIR\System32\wscript.exe"
    $lnk.Arguments = "`"$target`""
    $lnk.WorkingDirectory = $here
    if (Test-Path $icon) { $lnk.IconLocation = $icon }
    $lnk.Description = "Record Go2 walks and turn them into maps"
    $lnk.Save()
    Write-Host "made $($lnk.FullName)"
}
