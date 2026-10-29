. .\win_tool.ps1
[WinTool]::ListAllWindows() | Where-Object { $_ -match 'MuJoCo|Simulate' }

