using System;
using System.Collections.Generic;

namespace IaiActuatorDriver
{
    /// <summary>
    /// In-memory <see cref="ITransport"/> for tests: records every frame written and answers it from a script of
    /// responses (<see cref="Enqueue"/>; <c>null</c> = no answer) or, when the script is empty, from a
    /// <see cref="Responder"/> callback (request frame in, response bytes or <c>null</c> out). Reads never block: what
    /// is buffered is returned at once, so a missing answer times out immediately unless a wait was asked for.
    /// </summary>
    public sealed class FakeTransport : ITransport
    {
        private readonly object _sync = new object();
        private readonly Queue<byte[]?> _script = new Queue<byte[]?>();
        private readonly List<byte> _rx = new List<byte>();
        private readonly List<byte[]> _writes = new List<byte[]>();
        private readonly List<double> _writeTimes = new List<double>();
        private readonly List<int> _baudRates = new List<int>();
        private readonly List<TimeSpan> _breaks = new List<TimeSpan>();
        private int _baudRate;
        private TimeSpan _readTimeout = TimeSpan.FromSeconds(0.2);

        /// <summary>Create a fake.</summary>
        /// <param name="responder">Answers a request frame when the script is empty (<c>null</c> return = silence).</param>
        /// <param name="baudRate">0 = no baud rate (like a simulator); otherwise the starting rate, settable in place.</param>
        public FakeTransport(Func<byte[], byte[]?>? responder = null, int baudRate = 0)
        {
            Responder = responder;
            _baudRate = baudRate;
        }

        /// <summary>Answers a request when the script is empty; <c>null</c> = no answer.</summary>
        public Func<byte[], byte[]?>? Responder { get; set; }

        /// <summary>When set, the fake only answers while <see cref="BaudRate"/> equals it (a unit at another rate is silent).</summary>
        public int? AnswerAtBaudRate { get; set; }

        /// <summary>Every frame written, in order.</summary>
        public IReadOnlyList<byte[]> Writes
        {
            get
            {
                lock (_sync)
                {
                    return _writes.ToArray();
                }
            }
        }

        /// <summary>Monotonic time (seconds) of every write.</summary>
        public IReadOnlyList<double> WriteTimes
        {
            get
            {
                lock (_sync)
                {
                    return _writeTimes.ToArray();
                }
            }
        }

        /// <summary>Every baud rate set, in order.</summary>
        public IReadOnlyList<int> BaudRates
        {
            get
            {
                lock (_sync)
                {
                    return _baudRates.ToArray();
                }
            }
        }

        /// <summary>Every line break sent.</summary>
        public IReadOnlyList<TimeSpan> Breaks
        {
            get
            {
                lock (_sync)
                {
                    return _breaks.ToArray();
                }
            }
        }

        /// <summary>True once <see cref="Close"/> was called.</summary>
        public bool Closed { get; private set; }

        /// <inheritdoc />
        public int BaudRate
        {
            get => _baudRate;
            set
            {
                lock (_sync)
                {
                    if (_baudRate == 0)
                    {
                        throw new NotSupportedException("this fake transport has no baud rate");
                    }

                    _baudRate = value;
                    _baudRates.Add(value);
                    _rx.Clear();
                }
            }
        }

        /// <inheritdoc />
        public TimeSpan ReadTimeout
        {
            get => _readTimeout;
            set => _readTimeout = value;
        }

        /// <summary>Append scripted responses; each answers one request (<c>null</c> = no answer).</summary>
        public void Enqueue(params byte[]?[] responses)
        {
            lock (_sync)
            {
                foreach (var response in responses)
                {
                    _script.Enqueue(response);
                }
            }
        }

        /// <summary>Put bytes into the receive buffer directly (line noise, a late answer).</summary>
        public void Inject(params byte[] bytes)
        {
            lock (_sync)
            {
                _rx.AddRange(bytes);
            }
        }

        /// <inheritdoc />
        public void Write(byte[] buffer, int offset, int count)
        {
            var frame = new byte[count];
            Array.Copy(buffer, offset, frame, 0, count);
            Func<byte[], byte[]?>? responder = null;
            lock (_sync)
            {
                if (Closed)
                {
                    throw new System.IO.IOException("fake transport is closed");
                }

                _writes.Add(frame);
                _writeTimes.Add(Timing.Now);
                if (AnswerAtBaudRate.HasValue && AnswerAtBaudRate.Value != _baudRate)
                {
                    return; // garbled at the wrong rate: no answer
                }

                if (_script.Count > 0)
                {
                    var response = _script.Dequeue();
                    if (response != null)
                    {
                        _rx.AddRange(response);
                    }

                    return;
                }

                responder = Responder;
            }

            var answer = responder?.Invoke(frame);
            if (answer != null)
            {
                lock (_sync)
                {
                    _rx.AddRange(answer);
                }
            }
        }

        /// <inheritdoc />
        public int Read(byte[] buffer, int offset, int count)
        {
            lock (_sync)
            {
                var n = Math.Min(count, _rx.Count);
                _rx.CopyTo(0, buffer, offset, n);
                _rx.RemoveRange(0, n);
                return n;
            }
        }

        /// <inheritdoc />
        public void ResetInputBuffer()
        {
            lock (_sync)
            {
                _rx.Clear();
            }
        }

        /// <inheritdoc />
        public void SendBreak(TimeSpan duration)
        {
            lock (_sync)
            {
                _breaks.Add(duration);
                _rx.Clear();
            }
        }

        /// <summary>
        /// Open the transport again after <see cref="Close"/> (the next "program" opens the same port): the receive buffer is
        /// emptied, everything recorded so far is kept.
        /// </summary>
        public void Reopen()
        {
            lock (_sync)
            {
                Closed = false;
                _rx.Clear();
            }
        }

        /// <inheritdoc />
        public void Close() => Closed = true;

        /// <inheritdoc />
        public void Dispose() => Close();
    }
}
