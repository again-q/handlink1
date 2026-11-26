// HMax 数据手套直连桥（32 位）：加载厂家 h2sdk，把官方 ASCII 数据流直接喂给 handlink 的 UDP 5001。
// 不再需要 MotionVenus 在跑；只要它装过（DLL 和驱动在）。
//
// 用法： HmaxBridge.exe --dir "C:\Program Files (x86)\MotionVenus_HMax-2025.0.6.1648" --udp 127.0.0.1:5001
using System;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

internal static class HmaxBridge
{
    private const string DllName = "h2sdk_Release_v120_x86.dll";

    [DllImport("kernel32", CharSet = CharSet.Unicode, SetLastError = true)]
    private static extern bool SetDllDirectory(string path);

    [DllImport("kernel32", CharSet = CharSet.Unicode, SetLastError = true)]
    private static extern IntPtr LoadLibrary(string path);

    [DllImport("kernel32", CharSet = CharSet.Unicode)]
    private static extern bool SetCurrentDirectory(string path);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    private static extern int H2_SDK_Init();

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    private static extern int H2_SDK_DeInit();

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    private static extern int H2_SDK_getDevicesCount();

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    private static extern int H2_SDK_getGlovesSerialNumber(uint[] snArray);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    private static extern int H2_SDK_getStatus(uint sn, out uint leftOnline, out uint rightOnline,
        out uint leftBattery, out uint rightBattery, out uint leftFw, out uint rightFw);

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    private static extern int H2_SDK_getGloveStream(uint sn, byte[] outString, ref uint outLength);

    private static int Main(string[] args)
    {
        string dir = @"C:\Program Files (x86)\MotionVenus_HMax-2025.0.6.1648";
        string udp = "127.0.0.1:5001";
        int hz = 60;
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (args[i] == "--dir") dir = args[i + 1];
            else if (args[i] == "--udp") udp = args[i + 1];
            else if (args[i] == "--hz") int.TryParse(args[i + 1], out hz);
        }
        if (hz < 1) hz = 1;
        if (hz > 240) hz = 240;

        Console.OutputEncoding = Encoding.UTF8;
        Console.WriteLine("[hmax] dir=" + dir);
        SetDllDirectory(dir);
        SetCurrentDirectory(dir);
        IntPtr h = LoadLibrary(System.IO.Path.Combine(dir, DllName));
        if (h == IntPtr.Zero)
        {
            Console.WriteLine("[hmax] LoadLibrary failed: " + Marshal.GetLastWin32Error());
            return 2;
        }
        Console.WriteLine("[hmax] dll loaded");

        int rc = H2_SDK_Init();
        Console.WriteLine("[hmax] H2_SDK_Init -> " + rc);

        IPEndPoint target = ParseEndPoint(udp);
        using (var sock = new UdpClient())
        {
            uint lastSn = 0;
            byte[] buf = new byte[65536];
            byte[] frame = new byte[65536];
            bool stop = false;
            Console.CancelKeyPress += delegate(object s, ConsoleCancelEventArgs e) { e.Cancel = true; stop = true; };
            int tick = 1000 / hz;
            int logAt = 0;

            while (!stop)
            {
                int n = H2_SDK_getDevicesCount();
                if (n <= 0)
                {
                    if (Environment.TickCount - logAt > 2000)
                    {
                        Console.WriteLine("[hmax] devices=0（接收器没插，或串口被 MotionVenus 占着）");
                        logAt = Environment.TickCount;
                    }
                    Thread.Sleep(500);
                    continue;
                }

                uint[] sns = new uint[Math.Max(1, n)];
                int got = H2_SDK_getGlovesSerialNumber(sns);
                uint sn = (got > 0 && sns[0] != 0) ? sns[0] : 0;
                if (sn == 0)
                {
                    Thread.Sleep(200);
                    continue;
                }
                if (sn != lastSn)
                {
                    uint lo, ro, lb, rb, lf, rf;
                    H2_SDK_getStatus(sn, out lo, out ro, out lb, out rb, out lf, out rf);
                    Console.WriteLine("[hmax] sn=0x" + sn.ToString("X") + " left=" + lo + " right=" + ro
                        + " batt=" + lb + "/" + rb + " fw=" + lf + "/" + rf);
                    lastSn = sn;
                }

                uint len = (uint)buf.Length;
                int sr = H2_SDK_getGloveStream(sn, buf, ref len);
                if (sr == 0 && len > 0 && len <= buf.Length)
                {
                    Array.Copy(buf, frame, (int)len);
                    sock.Send(frame, (int)len, target);
                }
                Thread.Sleep(tick);
            }
        }
        H2_SDK_DeInit();
        Console.WriteLine("[hmax] bye");
        return 0;
    }

    private static IPEndPoint ParseEndPoint(string s)
    {
        string[] parts = s.Split(':');
        return new IPEndPoint(IPAddress.Parse(parts[0]), int.Parse(parts[1]));
    }
}
