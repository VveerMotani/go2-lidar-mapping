# Process Go2 walks on Windows, with Docker Desktop. No WSL setup, no ROS.
#
#   .\go2-walks.ps1 list
#   .\go2-walks.ps1 process Lab-loop-3.zip
#   .\go2-walks.ps1 map Lab-loop-3
#
# Walks live in $HOME\go2-walks unless -Walks says otherwise. Put the .zip from
# the robot dashboard's Download button there. Everything the processor writes
# lands beside it, in the walk's own folder.
#
# If PowerShell refuses to run this ("running scripts is disabled"), run it as
#   powershell -ExecutionPolicy Bypass -File .\go2-walks.ps1 list

param(
    [Parameter(Position = 0)] [string] $Command = "help",
    [Parameter(Position = 1, ValueFromRemainingArguments = $true)] [string[]] $Rest,
    [string] $Walks = (Join-Path $HOME "go2-walks"),
    [string] $Image = "ghcr.io/vveermotani/go2-lidar-mapping/go2-walks:latest",
    [switch] $Build
)

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Host "Docker is not installed. Install Docker Desktop from https://www.docker.com/products/docker-desktop/ and start it." -ForegroundColor Red
    exit 1
}
docker info *> $null
if ($LASTEXITCODE -ne 0) {
    Write-Host "Docker is installed but not running. Start Docker Desktop and wait for it to say 'Engine running'." -ForegroundColor Red
    exit 1
}

# -Build makes the image from this repo instead of downloading it: for when the
# published one is not reachable, or after changing the scripts in notes\.
if ($Build) {
    $Image = "go2-walks:local"
    docker build -t $Image -f (Join-Path $PSScriptRoot "docker\Dockerfile") $PSScriptRoot
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
} else {
    docker image inspect $Image *> $null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Downloading the walk processor (about 2 GB, once)..."
        docker pull $Image
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Could not download $Image. Build it from this repo instead:  .\go2-walks.ps1 -Build $Command $Rest" -ForegroundColor Red
            exit 1
        }
    }
}

New-Item -ItemType Directory -Force -Path $Walks | Out-Null
Write-Host "walks folder: $Walks"
docker run --rm -it -v "${Walks}:/walks" $Image $Command @Rest
exit $LASTEXITCODE
