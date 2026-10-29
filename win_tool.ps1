$code = @"
using System;
using System.Threading;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;

public class WinTool {
    [DllImport("user32.dll", SetLastError = true)]
    public static extern IntPtr OpenDesktop(string lpszDesktop, uint dwFlags, bool fInherit, uint dwDesiredAccess);

    [DllImport("user32.dll", SetLastError = true)]
    public static extern bool SetThreadDesktop(IntPtr hDesktop);

    [DllImport("user32.dll", SetLastError = true)]
    public static extern bool CloseDesktop(IntPtr hDesktop);

    [DllImport("user32.dll")]
    public static extern IntPtr GetDesktopWindow();

    [DllImport("user32.dll")]
    public static extern IntPtr GetDC(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern int ReleaseDC(IntPtr hWnd, IntPtr hDC);

    [DllImport("gdi32.dll")]
    public static extern IntPtr CreateCompatibleDC(IntPtr hdc);

    [DllImport("gdi32.dll")]
    public static extern IntPtr CreateCompatibleBitmap(IntPtr hdc, int nWidth, int nHeight);

    [DllImport("gdi32.dll")]
    public static extern IntPtr SelectObject(IntPtr hdc, IntPtr hgdiobj);

    [DllImport("gdi32.dll")]
    public static extern bool BitBlt(IntPtr hdcDest, int nXDest, int nYDest, int nWidth, int nHeight, IntPtr hdcSrc, int nXSrc, int nYSrc, uint dwRop);

    [DllImport("gdi32.dll")]
    public static extern bool DeleteDC(IntPtr hdc);

    [DllImport("gdi32.dll")]
    public static extern bool DeleteObject(IntPtr hObject);

    [DllImport("user32.dll")]
    public static extern int GetSystemMetrics(int nIndex);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool SetForegroundWindow(IntPtr hWnd);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);

    [DllImport("user32.dll")]
    public static extern void SwitchToThisWindow(IntPtr hWnd, bool fAltTab);

    [DllImport("user32.dll")]
    public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint lpdwProcessId);

    [DllImport("kernel32.dll")]
    public static extern uint GetCurrentThreadId();

    [DllImport("user32.dll")]
    public static extern bool AttachThreadInput(uint idAttach, uint idAttachTo, bool fAttach);

    [DllImport("user32.dll")]
    public static extern bool SetCursorPos(int X, int Y);

    [DllImport("user32.dll")]
    public static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint dwData, int dwExtraInfo);

    [DllImport("user32.dll")]
    public static extern void keybd_event(byte bVk, byte bScan, uint dwFlags, int dwExtraInfo);

    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool EnumDesktopWindows(IntPtr hDesktop, EnumWindowsProc lpEnumFunc, IntPtr lParam);

    [DllImport("user32.dll", SetLastError = true, CharSet = CharSet.Auto)]
    public static extern int GetWindowText(IntPtr hWnd, StringBuilder lpString, int nMaxCount);

    [DllImport("user32.dll", SetLastError = true, CharSet = CharSet.Auto)]
    public static extern int GetWindowTextLength(IntPtr hWnd);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool IsWindowVisible(IntPtr hWnd);

    public static List<string> ListAllWindows() {
        var list = new List<string>();
        Thread t = new Thread(() => {
            IntPtr hDesk = OpenDesktop("default", 0, false, 0x01FF);
            if (hDesk == IntPtr.Zero) return;
            if (!SetThreadDesktop(hDesk)) { CloseDesktop(hDesk); return; }

            EnumDesktopWindows(hDesk, (hWnd, lParam) => {
                int length = GetWindowTextLength(hWnd);
                if (length > 0) {
                    StringBuilder builder = new StringBuilder(length + 1);
                    GetWindowText(hWnd, builder, builder.Capacity);
                    uint pid;
                    GetWindowThreadProcessId(hWnd, out pid);
                    bool vis = IsWindowVisible(hWnd);
                    list.Add(string.Format("PID:{0}|HWND:{1}|Vis:{2}|Title:{3}", pid, hWnd, vis, builder.ToString()));
                }
                return true;
            }, IntPtr.Zero);

            CloseDesktop(hDesk);
        });
        t.SetApartmentState(ApartmentState.STA);
        t.Start();
        t.Join();
        return list;
    }

    public static string BringToFront(IntPtr hWnd) {
        string res = "";
        Thread t = new Thread(() => {
            IntPtr hDesk = OpenDesktop("default", 0, false, 0x01FF);
            if (hDesk == IntPtr.Zero) return;
            if (!SetThreadDesktop(hDesk)) { CloseDesktop(hDesk); return; }

            uint targetPid;
            uint targetTid = GetWindowThreadProcessId(hWnd, out targetPid);
            uint curTid = GetCurrentThreadId();

            AttachThreadInput(curTid, targetTid, true);
            ShowWindow(hWnd, 9); // SW_RESTORE
            ShowWindow(hWnd, 3); // SW_SHOWMAXIMIZED
            SetForegroundWindow(hWnd);
            SwitchToThisWindow(hWnd, true);
            AttachThreadInput(curTid, targetTid, false);

            CloseDesktop(hDesk);
            res = "OK";
        });
        t.SetApartmentState(ApartmentState.STA);
        t.Start();
        t.Join();
        return res;
    }

    public static string Capture(string capturePath) {
        string res = "";
        Thread t = new Thread(() => {
            IntPtr hDesk = OpenDesktop("default", 0, false, 0x01FF);
            if (hDesk == IntPtr.Zero) return;
            if (!SetThreadDesktop(hDesk)) { CloseDesktop(hDesk); return; }

            Thread.Sleep(400);

            int width = GetSystemMetrics(0);
            int height = GetSystemMetrics(1);

            IntPtr hDeskWnd = GetDesktopWindow();
            IntPtr hdcSrc = GetDC(hDeskWnd);
            IntPtr hdcDest = CreateCompatibleDC(hdcSrc);
            IntPtr hBitmap = CreateCompatibleBitmap(hdcSrc, width, height);
            IntPtr hOldBmp = SelectObject(hdcDest, hBitmap);

            BitBlt(hdcDest, 0, 0, width, height, hdcSrc, 0, 0, 0x00CC0020);

            SelectObject(hdcDest, hOldBmp);
            DeleteDC(hdcDest);
            ReleaseDC(hDeskWnd, hdcSrc);

            Bitmap bmp = Image.FromHbitmap(hBitmap);
            bmp.Save(capturePath, ImageFormat.Png);
            bmp.Dispose();
            DeleteObject(hBitmap);

            CloseDesktop(hDesk);
            res = "OK";
        });
        t.SetApartmentState(ApartmentState.STA);
        t.Start();
        t.Join();
        return res;
    }

    public static void Click(int x, int y) {
        Thread t = new Thread(() => {
            IntPtr hDesk = OpenDesktop("default", 0, false, 0x01FF);
            if (hDesk == IntPtr.Zero) return;
            if (!SetThreadDesktop(hDesk)) { CloseDesktop(hDesk); return; }

            SetCursorPos(x, y);
            Thread.Sleep(50);
            mouse_event(0x0002, 0, 0, 0, 0); // DOWN
            Thread.Sleep(50);
            mouse_event(0x0004, 0, 0, 0, 0); // UP

            CloseDesktop(hDesk);
        });
        t.SetApartmentState(ApartmentState.STA);
        t.Start();
        t.Join();
    }
}
"@

Add-Type -TypeDefinition $code -ReferencedAssemblies "System.Drawing"
