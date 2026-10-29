$code = @"
using System;
using System.Text;
using System.Runtime.InteropServices;

public class WinStationCheck {
    [DllImport("user32.dll")]
    public static extern IntPtr GetProcessWindowStation();

    [DllImport("user32.dll")]
    public static extern IntPtr GetThreadDesktop(int dwThreadId);

    [DllImport("kernel32.dll")]
    public static extern int GetCurrentThreadId();

    [DllImport("user32.dll", SetLastError = true)]
    public static extern bool GetUserObjectInformation(IntPtr hObj, int nIndex, StringBuilder pvInfo, int nLength, out int lpnLengthNeeded);

    public static string GetInfo() {
        IntPtr hWinst = GetProcessWindowStation();
        StringBuilder sb = new StringBuilder(256);
        int needed = 0;
        GetUserObjectInformation(hWinst, 2, sb, 256, out needed);
        string winstName = sb.ToString();

        IntPtr hDesk = GetThreadDesktop(GetCurrentThreadId());
        sb.Clear();
        GetUserObjectInformation(hDesk, 2, sb, 256, out needed);
        string deskName = sb.ToString();

        return "WinStation: " + winstName + " | Desktop: " + deskName;
    }
}
"@

Add-Type -TypeDefinition $code
[WinStationCheck]::GetInfo()

