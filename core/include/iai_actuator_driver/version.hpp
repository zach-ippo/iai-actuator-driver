// iai_actuator_driver - unofficial C++ driver for IAI actuators over serial Modbus RTU. Not an IAI product.
//
// Version of the C++ edition: the release version of the repository, the same in every package
// (.github/release/check_version.py). The behaviour it ports is the Python edition's and the C# edition's.
#pragma once

namespace iai_actuator_driver {

/// Version of this C++ edition.
inline constexpr const char* kVersion = "1.1.0";

/// Short label used by the CLI and in JSON output.
inline constexpr const char* kEdition = "iai-actuator-driver-cpp";

/// One-line disclaimer shown by the CLI (--version) and the README.
inline constexpr const char* kDisclaimer =
    "Unofficial driver: not an IAI product. VERIFIED on hardware only where docs/COMPATIBILITY.md says so; anything "
    "else is UNVERIFIED.";

}  // namespace iai_actuator_driver
