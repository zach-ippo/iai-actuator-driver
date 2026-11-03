using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using IaiActuatorDriver.Interop;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver
{
    /// <summary>One serial port of this PC, as <see cref="PortManager.ListPorts"/> reports it. READ-ONLY: nothing is opened.</summary>
    /// <param name="Port">The port name ("COM3", "/dev/ttyUSB0").</param>
    /// <param name="Description">The device's friendly name ("" when it could not be found).</param>
    /// <param name="Vid">USB vendor ID (<c>null</c> when not a USB port or not found).</param>
    /// <param name="Pid">USB product ID (<c>null</c> when not a USB port or not found).</param>
    /// <param name="Manufacturer">The driver's manufacturer text ("" when not found).</param>
    public sealed record SerialPortInfo(string Port, string Description = "", int? Vid = null, int? Pid = null, string Manufacturer = "")
    {
        /// <summary>
        /// The IAI USB adapter (CP210x, VID 0x10C4 / PID 0x81D7, VERIFIED), or a port whose manufacturer or description
        /// starts with "IAI" (UNVERIFIED for other IAI adapters).
        /// </summary>
        public bool IsIai =>
            (Vid == PortManager.IaiUsbVid && Pid == PortManager.IaiUsbPid)
            || Manufacturer.TrimStart().StartsWith("IAI", StringComparison.OrdinalIgnoreCase)
            || Description.TrimStart().StartsWith("IAI ", StringComparison.OrdinalIgnoreCase);

        /// <summary>JSON-friendly dictionary: port, description, vid, pid, is_iai.</summary>
        public IDictionary<string, object?> ToDictionary() => new Dictionary<string, object?>
        {
            ["port"] = Port,
            ["description"] = Description,
            ["vid"] = Vid,
            ["pid"] = Pid,
            ["is_iai"] = IsIai,
        };
    }

    /// <summary>
    /// Port discovery and address scans (the <c>cell</c> module of the Python edition): <see cref="ListPorts"/> (IAI USB
    /// adapters first), <see cref="Scan(string, IEnumerable{int}?, TimeSpan?, bool, int?, TimeSpan?, LineKind)"/> one port, and the
    /// constants a <see cref="Cell"/> uses. Everything here is READ-ONLY.
    /// </summary>
    /// <remarks>
    /// Evidence: the IAI adapter's USB ID 10C4:81D7 ("IAI USB to UART Bridge Controller", maker "IAI Corporation") is
    /// VERIFIED; the identification of other adapters is UNVERIFIED. The port list itself comes from the native core
    /// (iad_list_ports_json: SetupAPI on Windows, with the USB IDs, names and manufacturers; no WMI);
    /// elsewhere what the core reads from the system (UNVERIFIED).
    /// </remarks>
    public static class PortManager
    {
        /// <summary>USB vendor ID of the IAI USB-RS485 adapter (Silicon Labs CP210x). VERIFIED.</summary>
        public const int IaiUsbVid = 0x10C4;

        /// <summary>USB product ID of the IAI adapter ("IAI USB to UART Bridge Controller", maker "IAI Corporation"). VERIFIED.</summary>
        public const int IaiUsbPid = 0x81D7;

        /// <summary>The port limit of a <see cref="Cell"/> up to 1.0, kept for compatibility: since 1.1 a cell takes any number of ports (one task each) and every axis on them (<see cref="CellOptions.MaxAxes"/> caps it).</summary>
        public const int MaxPorts = 16;

        private const string LogSource = "IaiActuatorDriver.PortManager";

        /// <summary>Addresses scanned by default: axis numbers 0..15 (slave = axis number + 1).</summary>
        public static IReadOnlyList<int> DefaultSlaves => Port.DefaultScanAddresses;

        /// <summary>The name of the axis at <paramref name="slave"/> on <paramref name="port"/> in a <see cref="Cell"/>, e.g. "COM3:1".</summary>
        public static string AxisName(string port, int slave) => port + ":" + slave.ToString(CultureInfo.InvariantCulture);

        /// <summary>
        /// The serial ports of this PC, IAI adapters first, then in natural order (COM3 before COM10). READ-ONLY: no port is
        /// opened. On Windows each port's USB vendor / product ID, name and manufacturer are read from the device registry
        /// when present; a port without them is still listed (<see cref="SerialPortInfo.IsIai"/> false).
        /// </summary>
        public static IReadOnlyList<SerialPortInfo> ListPorts()
        {
            // the core's list (iad_list_ports_json: SetupAPI on Windows, /dev and sysfs elsewhere); opens nothing
            string json;
            try
            {
                json = NativeDriver.ListPortsJson();
            }
            catch (Exception exc) when (!(exc is DllNotFoundException || exc is BadImageFormatException || exc is NotSupportedException))
            {
                DriverLog.Warning(LogSource, "no serial ports listed: " + exc.Message);
                return Array.Empty<SerialPortInfo>();
            }

            var entries = new List<SerialPortInfo>();
            if (MiniJson.TryParse(json) is List<object?> list)
            {
                foreach (var item in list.OfType<JsonObject>())
                {
                    var port = item.String("port");
                    if (string.IsNullOrWhiteSpace(port))
                    {
                        continue;
                    }

                    entries.Add(new SerialPortInfo(port!.Trim(), item.String("description") ?? string.Empty, item.Int("vid"), item.Int("pid"), item.String("manufacturer") ?? string.Empty));
                }
            }

            return Sort(entries.GroupBy(e => e.Port, StringComparer.OrdinalIgnoreCase).Select(g => g.First()));
        }

        /// <summary>The names of the IAI adapters among <see cref="ListPorts"/>.</summary>
        public static IReadOnlyList<string> IaiPortNames() => ListPorts().Where(p => p.IsIai).Select(p => p.Port).ToList().AsReadOnly();

        /// <summary>
        /// Scan one serial port for answering addresses (READ-ONLY; see <see cref="IaiActuatorDriver.Port.ScanUnits"/>). The port is
        /// opened for the scan and closed again. <paramref name="baudRate"/> <c>null</c> detects the rate in place.
        /// </summary>
        public static IReadOnlyList<ScanHit> Scan(
            string portName,
            IEnumerable<int>? slaves = null,
            TimeSpan? timeout = null,
            bool identify = true,
            int? baudRate = null,
            TimeSpan? readTimeout = null,
            LineKind line = LineKind.Auto)
        {
            if (!Enum.IsDefined(typeof(LineKind), line))  // before the serial port is opened
            {
                throw new ArgumentOutOfRangeException(nameof(line), line, "line must be a LineKind");
            }

            var transport = new SerialTransport(portName, baudRate ?? ModbusRtu.DefaultBaudRate, readTimeout);
            Port port;
            try
            {
                port = new Port(transport, portName, autoBaud: baudRate is null, line: line);
            }
            catch
            {
                transport.Dispose();  // the port that never took it would keep the serial port open
                throw;
            }

            using (port)
            {
                return port.ScanUnits(slaves, timeout, identify);
            }
        }

        /// <summary>Scan a transport (e.g. a simulated line); it is wrapped in a <see cref="IaiActuatorDriver.Port"/> and closed again.</summary>
        public static IReadOnlyList<ScanHit> Scan(ITransport transport, string? name = null, IEnumerable<int>? slaves = null, TimeSpan? timeout = null, bool identify = true, LineKind line = LineKind.Auto)
        {
            using var port = new Port(transport, name, line: line);
            return port.ScanUnits(slaves, timeout, identify);
        }

        /// <summary>Scan an open <see cref="IaiActuatorDriver.Port"/> (left open).</summary>
        public static IReadOnlyList<ScanHit> Scan(Port port, IEnumerable<int>? slaves = null, TimeSpan? timeout = null, bool identify = true) =>
            (port ?? throw new ArgumentNullException(nameof(port))).ScanUnits(slaves, timeout, identify);

        /// <summary>IAI adapters first, then natural order of the name ("COM3" before "COM10").</summary>
        internal static IReadOnlyList<SerialPortInfo> Sort(IEnumerable<SerialPortInfo> entries) =>
            entries
                .OrderBy(e => e.IsIai ? 0 : 1)
                .ThenBy(e => NameHead(e.Port), StringComparer.Ordinal)
                .ThenBy(e => NameNumber(e.Port))
                .ThenBy(e => e.Port, StringComparer.Ordinal)
                .ToList()
                .AsReadOnly();

        /// <summary>"1-3,5,7-8" for [1, 2, 3, 5, 7, 8].</summary>
        internal static string Ranges(IEnumerable<int> values)
        {
            var items = values.Distinct().OrderBy(v => v).ToList();
            if (items.Count == 0)
            {
                return "-";
            }

            var parts = new List<string>();
            var start = items[0];
            var prev = items[0];
            foreach (var v in items.Skip(1).Select(v => (int?)v).Concat(new int?[] { null }))
            {
                if (v.HasValue && v.Value == prev + 1)
                {
                    prev = v.Value;
                    continue;
                }

                parts.Add(start == prev ? start.ToString(CultureInfo.InvariantCulture) : $"{start}-{prev}");
                if (v.HasValue)
                {
                    start = prev = v.Value;
                }
            }

            return string.Join(",", parts);
        }

        private static string NameHead(string name) => name.TrimEnd('0', '1', '2', '3', '4', '5', '6', '7', '8', '9');

        private static long NameNumber(string name)
        {
            var tail = name.Substring(NameHead(name).Length);
            return tail.Length > 0 && tail.Length < 10 && long.TryParse(tail, NumberStyles.None, CultureInfo.InvariantCulture, out var n) ? n : -1;
        }
    }
}
