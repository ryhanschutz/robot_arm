. .\win_tool.ps1

$windows = [WinTool]::ListWindows()
$swHwnd = [IntPtr]::Zero
foreach ($w in $windows) {
    if ($w -like "*SOLIDWORKS*") {
        Write-Output "Found: $w"
        if ($w -match "HWND:(\d+)") {
            $swHwnd = [IntPtr][int64]$matches[1]
        }
    }
}

if ($swHwnd -ne [IntPtr]::Zero) {
    Write-Output "Focusing SolidWorks HWND: $swHwnd"
    $res = [WinTool]::FocusAndCapture($swHwnd, "c:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\solidworks_focused.png")
    Write-Output $res
} else {
    Write-Output "SolidWorks window not found!"
}
