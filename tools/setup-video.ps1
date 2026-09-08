param(
    [string]$Directory = (Join-Path $PSScriptRoot '../build/deps'),
    [string]$CMake = 'cmake'
)
$ErrorActionPreference = 'Stop'
$Directory = [IO.Path]::GetFullPath($Directory)
New-Item -ItemType Directory -Path $Directory -Force | Out-Null

$version = '9.0.1'
$package = "ffmpeg-$version-full_build-shared"
$sdk = Join-Path $Directory $package
$sha256 = 'CB4D5E8DB6A3353BFFDB2100D3EB4B76733457FA443215E236F57C99F9FFDCA4'
if (-not (Test-Path -LiteralPath (Join-Path $sdk '.verified'))) {
    $archive = Join-Path $Directory "$package.7z"
    if (-not (Test-Path -LiteralPath $archive)) {
        Invoke-WebRequest "https://www.gyan.dev/ffmpeg/builds/packages/$package.7z" -OutFile $archive
    }
    if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $sha256) {
        throw "FFmpeg checksum mismatch: $archive"
    }
    # The Windows Server system tar can lack LZMA support; CMake bundles it.
    Push-Location $Directory
    try {
        & $CMake -E tar xf $archive
        if ($LASTEXITCODE -ne 0) { throw 'FFmpeg extraction failed' }
    }
    finally { Pop-Location }
    Set-Content -LiteralPath (Join-Path $sdk '.verified') -Value $sha256
}
foreach ($required in 'include/libavcodec/avcodec.h', 'lib/avcodec.lib', 'bin/avcodec-63.dll', 'LICENSE') {
    if (-not (Test-Path -LiteralPath (Join-Path $sdk $required))) {
        throw "FFmpeg SDK is incomplete: $required"
    }
}
$cmakeSdk = $sdk.Replace('\', '/')
@"
set(CMAKE_PREFIX_PATH "$cmakeSdk" CACHE STRING "FFmpeg SDK" FORCE)
set(ENGINE_SIM_FFMPEG_RUNTIME_DIR "$cmakeSdk/bin" CACHE PATH "FFmpeg runtime" FORCE)
set(ENGINE_SIM_FFMPEG_LICENSE "$cmakeSdk/LICENSE" CACHE FILEPATH "FFmpeg license" FORCE)
"@ | Set-Content -LiteralPath (Join-Path $Directory 'video-deps.cmake')
Write-Output "FFmpeg $version ready; configure with -DDTV=ON -C build/deps/video-deps.cmake"
