$code = @"
using System;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;

public class DesktopCapture {
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

    const int SM_CXSCREEN = 0;
    const int SM_CYSCREEN = 1;
    const uint SRCCOPY = 0x00CC0020;

    public static string CaptureScreen(string outputPath) {
        IntPtr hDesk = OpenDesktop("default", 0, false, 0x01FF);
        if (hDesk == IntPtr.Zero) {
            return "Failed to open default desktop: " + Marshal.GetLastWin32Error();
        }

        bool setOk = SetThreadDesktop(hDesk);
        if (!setOk) {
            int err = Marshal.GetLastWin32Error();
            CloseDesktop(hDesk);
            return "Failed to SetThreadDesktop: " + err;
        }

        int width = GetSystemMetrics(SM_CXSCREEN);
        int height = GetSystemMetrics(SM_CYSCREEN);

        IntPtr hDeskWnd = GetDesktopWindow();
        IntPtr hdcSrc = GetDC(hDeskWnd);
        IntPtr hdcDest = CreateCompatibleDC(hdcSrc);
        IntPtr hBitmap = CreateCompatibleBitmap(hdcSrc, width, height);
        IntPtr hOldBmp = SelectObject(hdcDest, hBitmap);

        BitBlt(hdcDest, 0, 0, width, height, hdcSrc, 0, 0, SRCCOPY);

        SelectObject(hdcDest, hOldBmp);
        DeleteDC(hdcDest);
        ReleaseDC(hDeskWnd, hdcSrc);

        Bitmap bmp = Image.FromHbitmap(hBitmap);
        bmp.Save(outputPath, ImageFormat.Png);
        bmp.Dispose();
        DeleteObject(hBitmap);

        CloseDesktop(hDesk);
        return "Saved " + width + "x" + height + " to " + outputPath;
    }
}
"@

Add-Type -TypeDefinition $code -ReferencedAssemblies "System.Drawing"
[DesktopCapture]::CaptureScreen("c:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\screen_desktop.png")

