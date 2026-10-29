Add-Type -AssemblyName System.Windows.Forms
. .\win_tool.ps1

$windows = [WinTool]::ListAllWindows()
foreach ($w in $windows) {
    if ($w -match "Chrome" -and $w -notmatch "Visual") {
        if ($w -match "HWND:(\d+)") {
            [WinTool]::BringToFront([IntPtr][int64]$matches[1])
            Start-Sleep -Milliseconds 500
            
            # Click the '+' new tab button at ~ (730, 20)
            [WinTool]::Click(730, 20)
            Start-Sleep -Milliseconds 500
            
            # We can use clipboard or typing to navigate to Google Sheets
            Set-Clipboard -Value "https://docs.google.com/spreadsheets/u/0/"
            
            # Ctrl+V and Enter
            # Simulate key presses: CTRL (0x11), V (0x56), ENTER (0x0D)
            # Or use powershell SendKeys
            [System.Windows.Forms.SendKeys]::SendWait("^v{ENTER}")
            
            Start-Sleep -Milliseconds 3000
            [WinTool]::Capture("c:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\chrome_sheets.png")
            break
        }
    }
}
