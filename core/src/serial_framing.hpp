// The character framing of a serial port (SerialFraming) on the platform's port settings: the DCB on Windows
// (serial_win32.cpp), termios elsewhere (serial_posix.cpp). Private (not installed); declared here so the tests can
// check the mapping without opening a port.
#pragma once

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <termios.h>
#endif

#include "iai_actuator_driver/transport.hpp"

namespace iai_actuator_driver {
namespace detail {

// A byte that fails the parity check is not the same on every platform: Windows (fParity TRUE, fErrorChar FALSE)
// passes the byte on as received, POSIX (INPCK set, IGNPAR and PARMRK clear) reads it as 0. Either way the frame then
// fails the Modbus CRC and is dropped as a bad reply.
#ifdef _WIN32
/// ByteSize, Parity, fParity and StopBits of `dcb` for `framing`; the rest of the DCB is left as it is.
void apply_framing(DCB& dcb, const SerialFraming& framing);
#else
/// CSIZE, PARENB, PARODD, CSTOPB (and CMSPAR, always cleared) of `tio.c_cflag` and INPCK and IGNPAR (always cleared)
/// of `tio.c_iflag` for `framing`; the other flags are left as they are.
void apply_framing(termios& tio, const SerialFraming& framing);
/// Whether `tio` (read back after tcsetattr) still has the CSIZE, PARENB, PARODD (with a parity bit) and CSTOPB of
/// `framing`. A driver that cannot carry a framing changes them without an error: a Linux pseudo terminal keeps only
/// 8 data bits and no parity bit (and the C library's tcsetattr then sometimes reports EINVAL, sometimes not).
bool has_framing(const termios& tio, const SerialFraming& framing);
#endif

}  // namespace detail
}  // namespace iai_actuator_driver
