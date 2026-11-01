# 全検証を順に実行する。起動中の GUI には接続せず、スクリプトごとに headless プロセスを起動する。
# 使い方: powershell -NoProfile -ExecutionPolicy Bypass -File run_all.ps1
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$scripts = Get-ChildItem $here -Filter 't*.py' | Sort-Object Name
foreach ($s in $scripts) {
    powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $here 'run_group.ps1') $s.Name |
        Select-String -Pattern 'script=|\[DONE\]|\[WATCHDOG\]|!EXC|Traceback|bad allocation' |
        ForEach-Object { $_.Line.Substring(0, [Math]::Min(600, $_.Line.Length)) }
}
