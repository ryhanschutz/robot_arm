. .\win_tool.ps1

$windows = [WinTool]::ListAllWindows()
foreach ($w in $windows) {
    if ($w -match "Edge") {
        Write-Output "Edge: $w"
        if ($w -match "HWND:(\d+)") {
            [WinTool]::BringToFront([IntPtr][int64]$matches[1])
            Start-Sleep -Milliseconds 800
            [WinTool]::Capture("c:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\edge_screen.png")
        }
    }
}

