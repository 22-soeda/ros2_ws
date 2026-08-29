#Requires -Version 5.1
<#
.SYNOPSIS
  docs/*.md を PDF にする (pandoc で HTML 化 -> Edge/Chrome headless で印刷)。
  Windows の開発 PC で回すもの。Pi 上では使わない。

.PARAMETER InputMd
  入力 .md (ws ルートからの相対パス、または絶対パス)。例: docs/teleop_tuning.md
.PARAMETER OutputPdf
  出力 .pdf。省略時は入力と同じ場所に同名 .pdf (docs/ の PDF はそこに置く約束)。

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File docs/build_md_pdf.ps1 docs/teleop_tuning.md
#>
param(
  [Parameter(Mandatory = $true)] [string] $InputMd,
  [string] $OutputPdf = ""
)

$ErrorActionPreference = "Stop"
$WsRoot = Split-Path -Parent $PSScriptRoot
Set-Location $WsRoot

if ([System.IO.Path]::IsPathRooted($InputMd)) { $mdPath = $InputMd } else { $mdPath = Join-Path $WsRoot $InputMd }
if (-not (Test-Path -LiteralPath $mdPath)) { Write-Error "markdown not found: $mdPath" }

$buildDir = Join-Path $WsRoot "build\docs"
if (-not (Test-Path $buildDir)) { New-Item -ItemType Directory -Force -Path $buildDir | Out-Null }

$baseName = [System.IO.Path]::GetFileNameWithoutExtension($mdPath)
$htmlPath = Join-Path $buildDir "$baseName.html"
if (-not $OutputPdf) {
  $OutputPdf = Join-Path (Split-Path -Parent $mdPath) "$baseName.pdf"
} elseif (-not [System.IO.Path]::IsPathRooted($OutputPdf)) {
  $OutputPdf = Join-Path $WsRoot $OutputPdf
}

# --- pandoc ---
function Get-PandocPath {
  $cmd = Get-Command pandoc -ErrorAction SilentlyContinue
  if ($cmd -and $cmd.Source) { return $cmd.Source }
  foreach ($p in @(
      (Join-Path $env:LOCALAPPDATA "Pandoc\pandoc.exe"),
      (Join-Path $env:ProgramFiles "Pandoc\pandoc.exe"),
      "${env:ProgramFiles(x86)}\Pandoc\pandoc.exe"
    )) {
    if ($p -and (Test-Path -LiteralPath $p)) { return $p }
  }
  return $null
}
$pandocExe = Get-PandocPath
if (-not $pandocExe) { Write-Error "pandoc not found (winget install JohnMacFarlane.Pandoc)" }

$css = Join-Path $WsRoot "docs\style\doc.css"
# 図や CSS は HTML に埋め込む (--embed-resources)。相対パスは md のある場所を基準にする。
& $pandocExe $mdPath -o $htmlPath -s --embed-resources --metadata "lang=ja" `
    --css $css -f markdown --toc --toc-depth=2 --resource-path (Split-Path -Parent $mdPath)
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# --- Edge / Chrome headless ---
$browser = $null
foreach ($cand in @(
    "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe",
    "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe",
    "$env:ProgramFiles\Google\Chrome\Application\chrome.exe"
  )) {
  if (Test-Path -LiteralPath $cand) { $browser = $cand; break }
}
if (-not $browser) { Write-Error "Edge / Chrome not found" }

if (Test-Path -LiteralPath $OutputPdf) { Remove-Item -LiteralPath $OutputPdf -Force }

$fileUri = ([Uri] $htmlPath).AbsoluteUri
# 起動中のブラウザとプロファイルが衝突すると headless が終了しないため専用プロファイルを使う
$tmpProfile = Join-Path ([System.IO.Path]::GetTempPath()) ("edge-pdf-" + [guid]::NewGuid().ToString("N"))
$browserArgs = @(
  "--headless",
  "--user-data-dir=$tmpProfile",
  "--disable-gpu",
  "--no-pdf-header-footer",
  "--print-to-pdf=$OutputPdf",
  $fileUri
)
# msedge.exe は起動直後に制御を返すことがあるので、終了を待ってからファイルを確かめる
Start-Process -FilePath $browser -ArgumentList $browserArgs -Wait -NoNewWindow
$deadline = (Get-Date).AddSeconds(60)
while (-not (Test-Path -LiteralPath $OutputPdf) -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 300 }
if (-not (Test-Path -LiteralPath $OutputPdf)) { Write-Error "PDF was not produced: $OutputPdf" }

Remove-Item -LiteralPath $tmpProfile -Recurse -Force -ErrorAction SilentlyContinue
$sizeKb = [math]::Round((Get-Item -LiteralPath $OutputPdf).Length / 1KB, 1)
Write-Host "Wrote $OutputPdf ($sizeKb KB)"
