. .\win_tool.ps1

$windows = [WinTool]::ListAllWindows()
foreach ($w in $windows) {
    if ($w -match "SOLIDWORKS Education") {
        if ($w -match "HWND:(\d+)") {
            $swHwnd = [IntPtr][int64]$matches[1]
            [WinTool]::BringToFront($swHwnd)
            Start-Sleep -Milliseconds 600
            
            # Click "Avaliar" tab
            [WinTool]::Click(210, 165)
            Start-Sleep -Milliseconds 600
            
            [WinTool]::Capture("c:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\sw_avaliar.png")
            Write-Output "Captured sw_avaliar.png"
            break
        }
    }
}

