# 検証スクリプトを 1 つ、GUI とは別プロセスの headless Isaac Sim で実行する。
# 使い方: powershell -File run_group.ps1 t01_api_schema.py
#   ログ: ../results/<スクリプト名>.log / 結果: ../results/<グループ名>.json
param([Parameter(Mandatory = $true)][string]$Script)
$ErrorActionPreference = 'Continue'
# システムの PYTHONPATH (C:\BuiltOpenUSD) は Isaac の pxr を隠すので空にする
$env:PYTHONPATH = ''
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here
$results = Join-Path (Split-Path -Parent $here) 'results'
New-Item -ItemType Directory -Force $results | Out-Null
$log = Join-Path $results ([IO.Path]::GetFileNameWithoutExtension($Script) + '.log')
$t0 = Get-Date
cmd /c "C:\isaacsim-6.0.2\python.bat `"$Script`" --no-ros-env > `"$log`" 2>&1"
$code = $LASTEXITCODE
Write-Output "script=$Script exit=$code elapsed=$((Get-Date) - $t0)"
Select-String -Path $log -Encoding UTF8 -Pattern '\[RESULT\]|\[DONE\]|\[WATCHDOG\]|!EXC|Traceback|ModuleNotFound|bad allocation' | ForEach-Object { $_.Line.Substring(0, [Math]::Min(1200, $_.Line.Length)) }
