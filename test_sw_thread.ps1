$code = @"
using System;
using System.Threading;
using System.Runtime.InteropServices;

public class SolidWorksTester {
    [DllImport("user32.dll", SetLastError = true)]
    public static extern IntPtr OpenDesktop(string lpszDesktop, uint dwFlags, bool fInherit, uint dwDesiredAccess);

    [DllImport("user32.dll", SetLastError = true)]
    public static extern bool SetThreadDesktop(IntPtr hDesktop);

    [DllImport("user32.dll", SetLastError = true)]
    public static extern bool CloseDesktop(IntPtr hDesktop);

    public static string TestSW() {
        string res = "";
        Thread t = new Thread(() => {
            IntPtr hDesk = OpenDesktop("default", 0, false, 0x01FF);
            if (hDesk == IntPtr.Zero) {
                res = "OpenDesktop failed: " + Marshal.GetLastWin32Error();
                return;
            }
            if (!SetThreadDesktop(hDesk)) {
                res = "SetThreadDesktop failed: " + Marshal.GetLastWin32Error();
                CloseDesktop(hDesk);
                return;
            }

            try {
                object swObj = Marshal.GetActiveObject("SldWorks.Application");
                if (swObj != null) {
                    dynamic sw = swObj;
                    res = "Connected! Revision: " + sw.RevisionNumber();
                    dynamic model = sw.IActiveDoc2;
                    if (model != null) {
                        res += "\nActive Doc: " + model.GetTitle() + " (Type: " + model.GetType() + ")";
                        res += "\nPath: " + model.GetPathName();
                    } else {
                        res += "\nNo active doc.";
                    }
                } else {
                    res = "Marshal.GetActiveObject returned null";
                }
            } catch (Exception ex) {
                res = "Exception: " + ex.ToString();
            }

            CloseDesktop(hDesk);
        });

        t.SetApartmentState(ApartmentState.STA);
        t.Start();
        t.Join();
        return res;
    }
}
"@

Add-Type -TypeDefinition $code -ReferencedAssemblies "Microsoft.CSharp"
[SolidWorksTester]::TestSW()
