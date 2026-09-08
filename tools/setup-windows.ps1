param([string]$Directory = (Join-Path $PSScriptRoot '../build/deps'))
$ErrorActionPreference = 'Stop'
$Directory = [IO.Path]::GetFullPath($Directory)
New-Item -ItemType Directory -Path $Directory -Force | Out-Null

function Get-ZipDependency($Name, $Url, $Sentinel) {
    $destination = Join-Path $Directory $Name
    if (Test-Path -LiteralPath (Join-Path $destination $Sentinel)) { return }
    $archive = Join-Path $Directory "$Name.zip"
    Invoke-WebRequest $Url -OutFile $archive
    Expand-Archive -LiteralPath $archive -DestinationPath $destination -Force
}

Get-ZipDependency 'sdl2' 'https://github.com/libsdl-org/SDL/releases/download/release-2.0.22/SDL2-devel-2.0.22-VC.zip' 'SDL2-2.0.22/include/SDL.h'
Get-ZipDependency 'sdl2-image' 'https://github.com/libsdl-org/SDL_image/releases/download/release-2.6.3/SDL2_image-devel-2.6.3-VC.zip' 'SDL2_image-2.6.3/include/SDL_image.h'
Get-ZipDependency 'winflexbison' 'https://github.com/lexxmark/winflexbison/releases/download/v2.5.25/win_flex_bison-2.5.25.zip' 'win_bison.exe'

$boostRoot = Join-Path $Directory 'boost'
if (-not (Test-Path -LiteralPath (Join-Path $boostRoot '.extracted'))) {
    $archive = Join-Path $Directory 'boost.tar.gz'
    if (-not (Test-Path -LiteralPath $archive) -or (Get-Item -LiteralPath $archive).Length -ne 571759170) {
        Invoke-WebRequest 'https://github.com/MarkusJx/prebuilt-boost/releases/download/1.78.0/boost-1.78.0-windows-2022-msvc-static-x86.tar.gz' -OutFile $archive
    }
    # The source-tree boost/boost entries contain Windows-incompatible links.
    # Only the packaged headers and prebuilt libraries are needed here.
    & tar -xzf $archive -C $Directory 'boost/include' 'boost/lib'
    if ($LASTEXITCODE -ne 0) { throw 'Boost extraction failed' }
    Set-Content -LiteralPath (Join-Path $boostRoot '.extracted') -Value '1.78.0 windows-2022 msvc static'
}
$boostLibrary = Get-ChildItem -LiteralPath (Join-Path $boostRoot 'lib') -Filter '*boost_filesystem*.lib' |
    Where-Object Name -match 'x64' | Select-Object -First 1
if (-not $boostLibrary) { throw 'Could not find the Boost filesystem x64 libraries' }
$cache = Join-Path $Directory 'windows-deps.cmake'
$settings = @{
    SDL2_DIR = "$Directory/sdl2/SDL2-2.0.22"
    SDL2_IMAGE_INCLUDE_DIR = "$Directory/sdl2-image/SDL2_image-2.6.3/include"
    SDL2_IMAGE_LIBRARY = "$Directory/sdl2-image/SDL2_image-2.6.3/lib/x64/SDL2_image.lib"
    BOOST_ROOT = $boostRoot
    BOOST_LIBRARYDIR = $boostLibrary.DirectoryName
    FLEX_EXECUTABLE = "$Directory/winflexbison/win_flex.exe"
    BISON_EXECUTABLE = "$Directory/winflexbison/win_bison.exe"
}
$settings.GetEnumerator() | Sort-Object Key | ForEach-Object {
    'set(' + $_.Key + ' "' + ($_.Value -replace '\\','/') + '" CACHE STRING "Windows build dependency" FORCE)'
} | Set-Content -LiteralPath $cache -Encoding utf8
Add-Content -LiteralPath $cache -Value 'set(Boost_NO_BOOST_CMAKE ON CACHE BOOL "Use FindBoost for the prebuilt archive" FORCE)'
Write-Host "Dependencies ready. Configure with: cmake -S . -B build/windows -A x64 -C `"$cache`""
