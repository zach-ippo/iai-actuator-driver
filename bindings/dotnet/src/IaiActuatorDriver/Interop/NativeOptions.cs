using System;
using IaiActuatorDriver.Native;

namespace IaiActuatorDriver.Interop
{
    // Settings passed to the open functions. Every property is optional: null keeps the library's default (written by
    // the iad_*_options_init functions, so the defaults live in one place, the native core).

    /// <summary>Settings of one axis (iad_axis_options; C++ AxisOptions). A cap can only narrow the unit's own range.</summary>
    internal sealed class NativeAxisOptions
    {
        /// <summary>Application soft limits (min, max), intersected with the unit's; null = the unit's own.</summary>
        public (double Min, double Max)? Stroke { get; set; }

        /// <summary>Cap for any commanded speed (units/s); null = the unit's own maximum.</summary>
        public double? MaxSpeedMmS { get; set; }

        /// <summary>Cap for any acceleration / deceleration (G); null = the unit's own maximum.</summary>
        public double? MaxAccelG { get; set; }

        /// <summary>Cap for push power (%); null = the unit's own maximum.</summary>
        public double? MaxPushPct { get; set; }

        /// <summary>How servo / home / alarm reset are sent; null = <see cref="CommandMethod.Coil"/> (Iai2: EXPERIMENTAL, UNVERIFIED).</summary>
        public CommandMethod? Strategy { get; set; }

        /// <summary>Status poll interval while waiting (s); null = 0.02.</summary>
        public double? PollIntervalS { get; set; }

        /// <summary>"Done" needs the position unchanged this long (s); null = 0.1.</summary>
        public double? SettleS { get; set; }

        /// <summary>Take Modbus control (PMSL) automatically at rest before a command; null = true.</summary>
        public bool? AutoModbusControl { get; set; }

        /// <summary>(0.2) A push outcome must hold this long (s); null = 0.15; a negative value = no settle (the first sample counts).</summary>
        public double? PushSettleS { get; set; }

        /// <summary>(0.2) When a move counts as done; null = <see cref="NativeCompletion.Safe"/>.</summary>
        public NativeCompletion? Completion { get; set; }

        /// <summary>The library's defaults with this object's settings on top.</summary>
        internal IadAxisOptions ToNative()
        {
            var o = SizedStruct.Fresh<IadAxisOptions>();
            NativeErrors.Check(NativeMethods.iad_axis_options_init(ref o));
            if (Stroke is { } stroke)
            {
                o.StrokeMin = stroke.Min;
                o.StrokeMax = stroke.Max;
            }

            o.MaxSpeedMmS = MaxSpeedMmS ?? o.MaxSpeedMmS;
            o.MaxAccelG = MaxAccelG ?? o.MaxAccelG;
            o.MaxPushPct = MaxPushPct ?? o.MaxPushPct;
            if (Strategy is { } strategy)
            {
                o.Method = MethodCode(strategy);
            }

            o.PollIntervalS = PollIntervalS ?? o.PollIntervalS;
            o.SettleS = SettleS ?? o.SettleS;
            if (AutoModbusControl is { } auto)
            {
                o.AutoModbusControl = Known.Flag(auto);
            }

            o.PushSettleS = PushSettleS ?? o.PushSettleS;
            o.Completion = Completion is { } completion ? (int)completion : o.Completion;
            return o;
        }

        /// <summary>The settings an axis reports (iad_axis_get_options), every property set.</summary>
        internal static NativeAxisOptions FromNative(in IadAxisOptions o) => new NativeAxisOptions
        {
            Stroke = double.IsNaN(o.StrokeMin) || double.IsNaN(o.StrokeMax) ? ((double, double)?)null : (o.StrokeMin, o.StrokeMax),
            MaxSpeedMmS = Known.Double(o.MaxSpeedMmS),
            MaxAccelG = Known.Double(o.MaxAccelG),
            MaxPushPct = Known.Double(o.MaxPushPct),
            Strategy = o.Method == (int)NativeCommandStrategy.Iai ? CommandMethod.Iai
                : o.Method == (int)NativeCommandStrategy.Iai2 ? CommandMethod.Iai2 : CommandMethod.Coil,
            PollIntervalS = o.PollIntervalS,
            SettleS = o.SettleS,
            AutoModbusControl = Known.Bool(o.AutoModbusControl),
            PushSettleS = Known.Double(o.PushSettleS),
            Completion = o.Completion == 2 ? NativeCompletion.Fast : NativeCompletion.Safe,
        };

        /// <summary>IAD_METHOD_* of a managed <see cref="CommandMethod"/>; null = IAD_METHOD_DEFAULT (0).</summary>
        internal static int MethodCode(CommandMethod? method)
        {
            switch (method)
            {
                case null:
                    return 0;
                case CommandMethod.Coil:
                    return (int)NativeCommandStrategy.Coil;
                case CommandMethod.Iai:
                    return (int)NativeCommandStrategy.Iai;
                case CommandMethod.Iai2:
                    return (int)NativeCommandStrategy.Iai2;
                default:
                    throw new NotSupportedException($"command method {method} is not in the native core");
            }
        }
    }

    /// <summary>Settings of a simulated unit (iad_sim_options; the part of C++ SimulatedUnitOptions a test needs).</summary>
    internal sealed class NativeSimOptions
    {
        /// <summary>Simulated seconds per real second; null = 1 (e.g. 20 makes tests fast).</summary>
        public double? TimeScale { get; set; }

        /// <summary>Initial position; null = the profile's.</summary>
        public double? PositionMm { get; set; }

        /// <summary>Soft limits -0.30 .. stroke + 0.30; null = the profile's.</summary>
        public double? StrokeMm { get; set; }

        /// <summary>MAXV; null = the profile's.</summary>
        public double? MaxSpeedMmS { get; set; }

        /// <summary>The rate the unit listens at; null = no baud rate modelled.</summary>
        public int? BaudRate { get; set; }

        /// <summary>Initial Modbus control; null = true (the hardware boots with false).</summary>
        public bool? Pmsl { get; set; }

        internal static IadSimOptions ToNative(NativeSimOptions? options)
        {
            var o = SizedStruct.Fresh<IadSimOptions>();
            NativeErrors.Check(NativeMethods.iad_sim_options_init(ref o));
            if (options is null)
            {
                return o;
            }

            o.TimeScale = options.TimeScale ?? o.TimeScale;
            o.PositionMm = options.PositionMm ?? o.PositionMm;
            o.StrokeMm = options.StrokeMm ?? o.StrokeMm;
            o.MaxSpeedMmS = options.MaxSpeedMmS ?? o.MaxSpeedMmS;
            o.BaudRate = options.BaudRate ?? o.BaudRate;
            if (options.Pmsl is { } pmsl)
            {
                o.Pmsl = Known.Flag(pmsl);
            }

            return o;
        }
    }

    /// <summary>Settings of <see cref="NativeCell.Open(string?, NativeCellOptions?)"/> (iad_cell_options; C++ CellOptions).</summary>
    internal sealed class NativeCellOptions
    {
        /// <summary>The most axes the cell takes; null or 0 = no limit.</summary>
        public int? MaxAxes { get; set; }

        /// <summary>This rate for every port; null = detect each port once.</summary>
        public int? BaudRate { get; set; }

        /// <summary>Per port; null = 0.2 s.</summary>
        public TimeSpan? ReadTimeout { get; set; }

        /// <summary>Minimum silence between transactions, per port; null = 5 ms.</summary>
        public TimeSpan? MinGap { get; set; }

        /// <summary>Per probed address while scanning; null = 50 ms.</summary>
        public TimeSpan? ScanTimeout { get; set; }

        /// <summary>Extra attempts per port after a timeout / CRC / mismatched frame; null = 3.</summary>
        public int? Retries { get; set; }

        /// <summary>First address scanned; null = 1.</summary>
        public int? ScanFirst { get; set; }

        /// <summary>Last address scanned; null = 16.</summary>
        public int? ScanLast { get; set; }

        /// <summary>Identify every axis at open (READ-ONLY); null = true.</summary>
        public bool? Identify { get; set; }

        /// <summary>A link fault on one axis stops the others; null = true.</summary>
        public bool? StopOnFault { get; set; }

        /// <summary>(0.2) Every port's line kind; null = <see cref="NativeLineKind.Auto"/> (probe once for a gateway).</summary>
        public NativeLineKind? Line { get; set; }

        /// <summary>(0.2) RCON lines: the axes' position unit, 0.01 or 0.001 mm; null = not stated.</summary>
        public double? PositionUnitMm { get; set; }

        /// <summary>Settings of every axis; null = the defaults.</summary>
        public NativeAxisOptions? AxisOptions { get; set; }

        internal static IadCellOptions ToNative(NativeCellOptions? options)
        {
            var o = SizedStruct.Fresh<IadCellOptions>();
            NativeErrors.Check(NativeMethods.iad_cell_options_init(ref o));
            if (options is null)
            {
                return o;
            }

            o.MaxAxes = options.MaxAxes ?? o.MaxAxes;
            o.BaudRate = options.BaudRate ?? o.BaudRate;
            o.ReadTimeoutS = options.ReadTimeout?.TotalSeconds ?? o.ReadTimeoutS;
            o.MinGapS = options.MinGap?.TotalSeconds ?? o.MinGapS;
            o.ScanTimeoutS = options.ScanTimeout?.TotalSeconds ?? o.ScanTimeoutS;
            o.Retries = options.Retries ?? o.Retries;
            o.ScanFirst = options.ScanFirst ?? o.ScanFirst;
            o.ScanLast = options.ScanLast ?? o.ScanLast;
            if (options.Identify is { } identify)
            {
                o.Identify = Known.Flag(identify);
            }

            if (options.StopOnFault is { } stopOnFault)
            {
                o.StopOnFault = Known.Flag(stopOnFault);
            }

            o.Line = options.Line is { } line ? (int)line : o.Line;
            o.PositionUnitMm = options.PositionUnitMm ?? o.PositionUnitMm;
            return o;
        }
    }
}
