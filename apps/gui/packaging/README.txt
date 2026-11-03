iai-gui: local web GUI for IAI actuators (ELECYLINDER and CON-family controllers)

Unofficial. This is not an IAI product and is not supported, documented or endorsed by IAI.
It is not a safety function: keep a hardware emergency stop and guarding.

iai-gui.exe holds Python, the iai-actuator-driver package with its native library, and the GUI; nothing else needs
installing.
It talks to the units over the RS-485 serial port (Modbus RTU), through the IAI USB adapter.


START

Double-click iai-gui.exe. A console window opens and the browser shows the GUI.
Quit with the Quit button in the page (top right) or with Ctrl+C in the console window. Closing the console
window also stops every axis and leaves each unit as found before the program ends.

Windows may warn about an unknown publisher: see WINDOWS SMARTSCREEN below.

Demo without hardware: open a command prompt in this folder and run
    iai-gui.exe --sim-only
(simulators only; no serial port is listed or opened), or press "Add two simulated axes".

If the browser does not open, open the address printed in the console window: it starts with
http://127.0.0.1:8780/ and ends with #k= and a key. The key is this run's access key: without it the page
cannot talk to the program, so type the whole address, and do not share it. (Another browser tab needs the
same address; the GUI page itself stays in the tab it was opened in.)
If another program uses port 8780: iai-gui.exe --port 8781
Backups made in the GUI go into a backups folder next to iai-gui.exe, or, when that folder cannot be written to
(for example under Program Files), into %LOCALAPPDATA%/IAI Actuator Driver/backups. The Back up result shows the path.


WINDOWS SMARTSCREEN

iai-gui.exe is not code-signed, so Windows (SmartScreen) or an antivirus may warn about it.
Press More info, then Run anyway, or ask your IT department to allow it.
Download it only from the GitHub release page of the project.


NO HARDWARE

Simulated axes run the same driver code and never open a serial port.
Press "Add two simulated axes": two sliders on two ports, enough for the Group and
Coordinated tabs.


WITH A UNIT

Close IA-OS first: only one program can use a port. Tick the port marked IAI and press
Connect selected ports; the baud rate is detected. Start slow and short, and keep a hand on the
emergency stop. Back up a unit (Functions > Back up this unit) before writing parameters or
stored positions.


THINGS TO TRY

1. Axis tab: Servo ON, Home, then a short slow move.
2. During a move press Space or Esc: STOP ALL.
3. Stored positions: run a row. Unit info & alarms: alarm history and Self-test.
4. Group tab: move two axes together.
5. Coordinated tab (experimental): Plan a small circle, check the plot, then Run.
6. PC / PIO switch (Axis tab, under the lamps): shows who has control of the unit. PIO lets the unit follow its PIO
   inputs and run its stored rows at their own speeds; the page then refuses motion commands until you switch back to PC.
   Only at rest.
7. Test report: Copy, then paste it into a GitHub issue with what you did.


WHAT TO REPORT

The test report, the steps, what you expected and what happened, in an issue at
https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose (a form for a bug, one for a unit report).
IAI does not support this program.


KNOWN LIMITS

Unofficial, not an IAI product.
STOP is a Modbus decelerate-stop, not a safety function.
A unit has no communication watchdog: a move in progress completes if the PC or the cable fails.
Coordinated motion is experimental: one real axis with a simulated second VERIFIED, two real axes
together UNVERIFIED.
Several axes on one port: UNVERIFIED.
The GUI listens on 127.0.0.1 only: other computers cannot reach it.


LICENCE

Apache License 2.0: see LICENSE and NOTICE. iai-gui.exe also carries Python, pySerial, the PyInstaller
bootloader and, linked into its native library, libc++, zig's compiler_rt and the mingw-w64 runtime, each under its
own licence: see THIRD_PARTY_NOTICES.txt, NATIVE_THIRD_PARTY.txt and PYTHON_LICENSE.txt.
