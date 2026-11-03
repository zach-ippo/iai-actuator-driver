using System;
using System.Threading;

namespace IaiActuatorDriver.Cli
{
    /// <summary>Entry point of <c>iai-actuator-driver-cs</c>.</summary>
    internal static class Program
    {
        private static int Main(string[] args)
        {
            using var cts = new CancellationTokenSource();
            var presses = 0;

            // First Ctrl+C: cancel the running command (a waiting move / home sends its decel STOP sequence, then the tool
            // exits with 130). A second Ctrl+C ends the process at once. The STOP first: the console line comes after
            // the cancel (a console write can block, e.g. while text is selected in the window).
            Console.CancelKeyPress += (_, e) =>
            {
                if (Interlocked.Increment(ref presses) == 1)
                {
                    e.Cancel = true;
                    cts.Cancel();
                    Console.Error.WriteLine("stopping (Ctrl+C again to quit at once)");
                }
            };

            return CliApp.Run(args, Console.Out, Console.Error, CliEnvironment.Default, cts.Token);
        }
    }
}
