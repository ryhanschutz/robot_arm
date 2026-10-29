. .\win_tool.ps1
$windows = [WinTool]::ListAllWindows()
$swHwnd = [IntPtr]::Zero
foreach ($w in $windows) {
    if ($w -match "SOLIDWORKS|Chrome|Edge|Sheets|Planilha|Mont") {
        Write-Output $w
    }
    if ($w -match "SOLIDWORKS Education") {
        if ($w -match "HWND:(\d+)") {
            $swHwnd = [IntPtr][int64]$matches[1]
        }
    }
}

if ($swHwnd -ne [IntPtr]::Zero) {
    Write-Output "Bringing SW ($swHwnd) to front..."
    [WinTool]::BringToFront($swHwnd)
    Start-Sleep -Milliseconds 1000
    [WinTool]::Capture("c:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\sw_screen.png")
    Write-Output "Captured sw_screen.png"
}

