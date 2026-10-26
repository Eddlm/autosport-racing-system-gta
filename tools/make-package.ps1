<#
  make-package.ps1 - build the release zip by hand.

  The owner assembles the package, CI never publishes one (deploy.yml only keeps a testable
  artifact on pushes to master). Output name follows the README: AutosportRacingSystem-DD-MM-YYYY.zip

  What goes in, and why:
    Dist\AutosportRacingSystem\      the hand-reviewed data mirror (tracks, rosters, tips)
    bin\Release\ARS.dll              the mod itself
    libs\LemonUI.SHVDN2.dll          MIT dependency, credited in README.txt
    build\build-number.txt           the banner the player will see
    tools\PACKAGE-README.txt         install + requirements, copied as README.txt

  What never goes in: ScriptHookVDotNet2.dll (the player installs their own pair),
  any *.ini (the mod writes defaults on first load), Vehicles\tests.txt, logs.

  Usage:
    powershell -NoProfile -File tools\make-package.ps1            # build, then package
    powershell -NoProfile -File tools\make-package.ps1 -SkipBuild # package what is on disk
#>
param(
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
Push-Location $repo
try {
    if (-not $SkipBuild) {
        $msbuild = 'C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe'
        if (-not (Test-Path $msbuild)) { throw "MSBuild not found at $msbuild" }
        & $msbuild NewRacingSystem.csproj /v:minimal /nologo /p:Configuration=Release
        if ($LASTEXITCODE -ne 0) { throw "MSBuild failed with exit code $LASTEXITCODE" }
    }

    $sources = @(
        @{ Path = 'Dist\AutosportRacingSystem'; Dir = 'AutosportRacingSystem' },
        @{ Path = 'bin\Release\ARS.dll';        Dir = 'AutosportRacingSystem' },
        @{ Path = 'libs\LemonUI.SHVDN2.dll';    Dir = 'AutosportRacingSystem' },
        @{ Path = 'build\build-number.txt';     Dir = 'AutosportRacingSystem' },
        @{ Path = 'tools\PACKAGE-README.txt';   Dir = 'AutosportRacingSystem'; Rename = 'README.txt' }
    )
    foreach ($s in $sources) {
        if (-not (Test-Path $s.Path)) { throw "missing input: $($s.Path)" }
    }

    $stage = Join-Path $env:TEMP ('ars-package-' + [guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $stage | Out-Null
    foreach ($s in $sources) {
        $dest = Join-Path $stage $s.Dir
        New-Item -ItemType Directory -Path $dest -Force | Out-Null
        if ((Get-Item $s.Path) -is [System.IO.DirectoryInfo]) {
            # Copy the children, not the folder: Copy-Item nests a directory inside the destination.
            Copy-Item "$($s.Path)\*" -Destination $dest -Recurse -Force
        } else {
            $name = if ($s.Rename) { $s.Rename } else { Split-Path $s.Path -Leaf }
            Copy-Item $s.Path -Destination (Join-Path $dest $name) -Force
        }
    }

    # Runtime files a local run leaves behind; the mirror already skips these, this is the second line.
    $junk = Get-ChildItem $stage -Recurse -Include '*.ini', '*.log', 'tests.txt', 'ScriptHookVDotNet2.dll' -File
    foreach ($f in $junk) { Remove-Item $f.FullName -Force }

    $zip = Join-Path $repo ('AutosportRacingSystem-' + (Get-Date -Format 'dd-MM-yyyy') + '.zip')
    if (Test-Path $zip) { Remove-Item $zip -Force }
    Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $zip -CompressionLevel Optimal
    Remove-Item $stage -Recurse -Force

    $files = (Get-ChildItem $repo -Filter 'AutosportRacingSystem-*.zip' | Where-Object { $_.FullName -eq $zip })
    $count = ([System.IO.Compression.ZipFile]::OpenRead($zip)).Entries.Count
    Write-Host ''
    Write-Host ("package : " + (Split-Path $zip -Leaf))
    Write-Host ("size    : " + [math]::Round((Get-Item $zip).Length / 1MB, 2) + " MB, $count entries")
    Write-Host ("build   : " + (Get-Content 'build\build-number.txt' -Raw).Trim())
    Write-Host ("path    : " + $zip)
}
finally {
    Pop-Location
}
