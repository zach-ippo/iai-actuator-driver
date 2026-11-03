# Security

How to report a security problem in the IAI Actuator Driver. The project is unofficial: IAI does not take reports
about it.

## Versions

Security fixes go into the newest release. Update to it before you report.

## Report a problem

Do not post the details of a security problem in a public issue.

1. If the repository's **Security** tab offers **Report a vulnerability**, use it: the report stays private.
2. Otherwise open an [issue](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose) that says only that
   you have a security report and which part it concerns (the GUI, a library, a command line or a release file). Leave
   out the details, the steps and any file. The maintainer then arranges a private way to send them.

## What counts

- The GUI: it listens on 127.0.0.1 only and every request needs the access key of the run. A way around that is a
  security problem.
- A release file that does not match `SHA256SUMS.txt` of its release.
- A crash or memory error in the native library caused by bytes on the serial line.

A move the driver should have refused, or a unit left in a bad state, is a safety fault, not a security one: use the
[bug report form](https://github.com/zach-ippo/iai-actuator-driver/issues/new/choose).
