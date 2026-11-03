using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace IaiActuatorDriver.Simulation
{
    /// <summary>
    /// One simulated serial line carrying several <see cref="SimulatedUnit"/>s by slave address, the way an RCON gateway or
    /// an SIO daisy chain puts several axes on one RS-485 line (slave address = axis number + 1; UNVERIFIED on hardware).
    /// Use it wherever an <see cref="ITransport"/> is: <c>new Port(line)</c> or <c>new ModbusRtu(line)</c>.
    /// </summary>
    /// <remarks>
    /// Every request frame is delivered to every unit on the line: the addressed unit answers, the others stay silent, a
    /// frame to the broadcast address 0 reaches all of them and none answers, and an address nobody has stays silent (the
    /// master times out). Two units cannot share an address (that would garble every reply on a real line). A baud-rate
    /// change of the line and a line BREAK reach every unit, like the shared wire. <see cref="Requests"/> records
    /// (time, frame) for every request, so tests can check how the frames of several axes interleave.
    /// </remarks>
    public sealed class SimLine : ITransport, IEnumerable<SimulatedUnit>
    {
        private readonly object _sync = new object();
        private readonly Dictionary<int, SimulatedUnit> _units = new Dictionary<int, SimulatedUnit>();
        private readonly List<(double Time, byte[] Frame)> _requests = new List<(double, byte[])>();
        private TimeSpan _readTimeout = TimeSpan.FromSeconds(0.2);
        private bool _closed;

        /// <summary>A line named <paramref name="name"/> (e.g. "SIM1") with <paramref name="units"/> on it.</summary>
        /// <exception cref="ArgumentException">A unit's address is outside 1..247 or already taken on the line.</exception>
        public SimLine(IEnumerable<SimulatedUnit>? units = null, string name = "SIM")
        {
            Name = name ?? throw new ArgumentNullException(nameof(name));
            foreach (var unit in units ?? Enumerable.Empty<SimulatedUnit>())
            {
                Add(unit);
            }
        }

        /// <summary>The line's name (e.g. "SIM1").</summary>
        public string Name { get; }

        /// <summary>The units on the line by slave address (a copy).</summary>
        public IReadOnlyDictionary<int, SimulatedUnit> Units
        {
            get
            {
                lock (_sync)
                {
                    return new Dictionary<int, SimulatedUnit>(_units);
                }
            }
        }

        /// <summary>The slave addresses on the line, in order.</summary>
        public IReadOnlyList<int> Slaves => Units.Keys.OrderBy(s => s).ToList();

        /// <summary>The number of units on the line.</summary>
        public int Count
        {
            get
            {
                lock (_sync)
                {
                    return _units.Count;
                }
            }
        }

        /// <summary>The unit at <paramref name="slave"/>.</summary>
        /// <exception cref="KeyNotFoundException">No unit has that address.</exception>
        public SimulatedUnit this[int slave]
        {
            get
            {
                lock (_sync)
                {
                    return _units[slave];
                }
            }
        }

        /// <summary>Every request sent so far: (monotonic time in s, frame), oldest first.</summary>
        public IReadOnlyList<(double Time, byte[] Frame)> Requests
        {
            get
            {
                lock (_sync)
                {
                    return _requests.ToArray();
                }
            }
        }

        /// <summary>True once <see cref="Close"/> was called.</summary>
        public bool Closed
        {
            get
            {
                lock (_sync)
                {
                    return _closed;
                }
            }
        }

        /// <summary>
        /// The line's (PC side) baud rate: the units' own model of it, 0 when they do not model one. Setting it changes the
        /// PC side's rate on the shared wire (every unit hears the line at the new rate).
        /// </summary>
        public int BaudRate
        {
            get
            {
                var rates = Units.Values.Select(u => u.BaudRate).Where(r => r > 0).Distinct().ToList();
                return rates.Count == 0 ? 0 : rates.Max();
            }

            set
            {
                var units = Units.Values.ToList();
                if (units.Count == 0 || BaudRate == 0)
                {
                    throw new NotSupportedException($"simulated line {Name} has no baud rate (set SimulatedUnitOptions.BaudRate on the units)");
                }

                foreach (var unit in units)
                {
                    unit.BaudRate = value;
                }
            }
        }

        /// <inheritdoc />
        public TimeSpan ReadTimeout
        {
            get
            {
                lock (_sync)
                {
                    return _readTimeout;
                }
            }

            set
            {
                lock (_sync)
                {
                    _readTimeout = value;
                }
            }
        }

        /// <summary>A line with one new unit per address in <paramref name="slaves"/>, each built from <paramref name="options"/>.</summary>
        public static SimLine WithSlaves(IEnumerable<int> slaves, SimulatedUnitOptions? options = null, string name = "SIM")
        {
            var o = options ?? new SimulatedUnitOptions();
            return new SimLine(slaves.Select(n => new SimulatedUnit(o with { Slave = n })).ToList(), name);
        }

        /// <summary>
        /// <paramref name="n"/> simulated ports named "SIM1" .. "SIM&lt;n&gt;" (with <paramref name="prefix"/>), each a line with
        /// one unit per address in <paramref name="slaves"/> (default: one unit at slave 1 per port - the normal setup: one
        /// axis per COM port).
        /// </summary>
        public static IReadOnlyDictionary<string, SimLine> SimPorts(int n, IEnumerable<int>? slaves = null, SimulatedUnitOptions? options = null, string prefix = "SIM")
        {
            if (n < 1 || n > 64)
            {
                throw new ArgumentOutOfRangeException(nameof(n), n, "n must be 1..64");
            }

            var addresses = (slaves ?? new[] { 1 }).ToList();
            var ports = new Dictionary<string, SimLine>();
            for (var i = 1; i <= n; i++)
            {
                var name = prefix + i.ToString(CultureInfo.InvariantCulture);
                ports[name] = WithSlaves(addresses, options, name);
            }

            return ports;
        }

        /// <summary>Put <paramref name="unit"/> on the line (its address must be free) and return it.</summary>
        public SimulatedUnit Add(SimulatedUnit unit)
        {
            if (unit is null)
            {
                throw new ArgumentNullException(nameof(unit));
            }

            if (unit.Slave < 1 || unit.Slave > 247)
            {
                throw new ArgumentException($"slave address must be 1..247, got {unit.Slave}", nameof(unit));
            }

            lock (_sync)
            {
                if (_units.ContainsKey(unit.Slave))
                {
                    throw new ArgumentException($"slave {unit.Slave} is already on line {Name}", nameof(unit));
                }

                _units[unit.Slave] = unit;
            }

            return unit;
        }

        /// <summary>Take the unit at <paramref name="slave"/> off the line (e.g. to model a cable pulled) and return it.</summary>
        public SimulatedUnit Remove(int slave)
        {
            lock (_sync)
            {
                var unit = _units[slave];
                _units.Remove(slave);
                return unit;
            }
        }

        /// <summary>The address byte of every request sent so far, in order (0 = broadcast).</summary>
        public IReadOnlyList<int> RequestSlaves()
        {
            lock (_sync)
            {
                return _requests.Where(r => r.Frame.Length > 0).Select(r => (int)r.Frame[0]).ToList();
            }
        }

        /// <summary>Forget the recorded requests.</summary>
        public void ClearRequests()
        {
            lock (_sync)
            {
                _requests.Clear();
            }
        }

        /// <inheritdoc />
        public void Write(byte[] buffer, int offset, int count)
        {
            var frame = new byte[count];
            Array.Copy(buffer, offset, frame, 0, count);
            List<SimulatedUnit> units;
            lock (_sync)
            {
                if (_closed)
                {
                    throw new IOException($"simulated line {Name} is closed");
                }

                _requests.Add((Timing.Now, frame));
                units = _units.Values.ToList();
            }

            foreach (var unit in units)
            {
                unit.Write(frame, 0, frame.Length); // every unit sees every frame; only the addressed one (none for 0) answers
            }
        }

        /// <inheritdoc />
        public int Read(byte[] buffer, int offset, int count)
        {
            var got = 0;
            foreach (var unit in Units.OrderBy(kv => kv.Key).Select(kv => kv.Value))
            {
                if (got >= count)
                {
                    break;
                }

                got += unit.Read(buffer, offset + got, count - got);
            }

            return got;
        }

        /// <inheritdoc />
        public void ResetInputBuffer()
        {
            foreach (var unit in Units.Values)
            {
                unit.ResetInputBuffer();
            }
        }

        /// <summary>A line BREAK reaches every unit on the wire.</summary>
        public void SendBreak(TimeSpan duration)
        {
            foreach (var unit in Units.Values)
            {
                unit.SendBreak(duration);
            }
        }

        /// <inheritdoc />
        public void Close()
        {
            List<SimulatedUnit> units;
            lock (_sync)
            {
                _closed = true;
                units = _units.Values.ToList();
            }

            foreach (var unit in units)
            {
                unit.Close();
            }
        }

        /// <inheritdoc />
        public void Dispose() => Close();

        /// <summary>The units in address order.</summary>
        public IEnumerator<SimulatedUnit> GetEnumerator() => Units.OrderBy(kv => kv.Key).Select(kv => kv.Value).ToList().GetEnumerator();

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        /// <inheritdoc />
        public override string ToString() => $"SimLine({Name}, slaves=[{string.Join(", ", Slaves)}])";
    }
}
