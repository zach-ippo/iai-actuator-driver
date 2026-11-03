using System.Collections.Generic;
using System.Linq;

namespace IaiActuatorDriver.Tests
{
    /// <summary>Frame logger collecting (direction, frame) pairs (thread-safe).</summary>
    internal sealed class FrameRecorder
    {
        private readonly object _sync = new object();
        private readonly List<(FrameDirection Direction, byte[] Frame)> _frames = new List<(FrameDirection, byte[])>();

        /// <summary>Every frame logged so far.</summary>
        public IReadOnlyList<(FrameDirection Direction, byte[] Frame)> Frames
        {
            get
            {
                lock (_sync)
                {
                    return _frames.ToArray();
                }
            }
        }

        /// <summary>The requests sent, in order.</summary>
        public IReadOnlyList<byte[]> Tx
        {
            get
            {
                lock (_sync)
                {
                    return _frames.Where(f => f.Direction == FrameDirection.Tx).Select(f => f.Frame).ToList();
                }
            }
        }

        /// <summary>The bus's frame-logger callback.</summary>
        public void Log(FrameDirection direction, byte[] frame)
        {
            lock (_sync)
            {
                _frames.Add((direction, (byte[])frame.Clone()));
            }
        }

        /// <summary>Forget everything logged so far.</summary>
        public void Clear()
        {
            lock (_sync)
            {
                _frames.Clear();
            }
        }
    }
}
