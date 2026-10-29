. .\win_tool.ps1

$windows = [WinTool]::ListAllWindows()
foreach ($w in $windows) {
    if ($w -match "Chrome") {
        if ($w -match "HWND:(\d+)") {
            [WinTool]::BringToFront([IntPtr][int64]$matches[1])
            Start-Sleep -Milliseconds 500
            # Click tab 3
            [WinTool]::Click(450, 20)
            Start-Sleep -Milliseconds 800
            [WinTool]::Capture("c:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\chrome_tab3.png")
            
            # Click tab 4
            [WinTool]::Click(630, 20)
            Start-Sleep -Milliseconds 800
            [WinTool]::Capture("c:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\chrome_tab4.png")
        }
    }
}

