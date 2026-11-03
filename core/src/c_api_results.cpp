// C ABI 0.7 (py-fixes): a call's own result and one pass of a cell's link monitor on the caller's thread
// (include/iai_actuator_driver.h "library: per-thread results; cell: link monitor pass").
//
// iad_last_result_json gives the result the last call on the calling thread kept on its handle, so a binding reads
// the result of its own iad_axis_recover / iad_axis_self_test / iad_cell_recover_all even when a call on another
// thread (a link monitor's iad_axis_check_link, a cell's check_links) has replaced the handle's last result since.
// iad_cell_link_monitor_check_now runs recovery::LinkMonitor::check_now of the cell's monitor: its states, passes and
// on_change (IAD_EVENT_LINK_STATE) are those of the pass, as a pass of its own thread.
//
// Evidence: UNVERIFIED on hardware (the C ABI is tested on the simulator only).
#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "c_api_impl.hpp"
#include "iai_actuator_driver.h"
#include "iai_actuator_driver/recovery.hpp"

using namespace iai_actuator_driver;
using namespace iai_actuator_driver::capi;

iad_result IAD_CALL iad_last_result_json(char* buffer, size_t buffer_size, size_t* required) {
    // Does not change the last error (not even when it fails), as iad_last_error_json, so it is not guarded.
    try {
        check_text_output(buffer, buffer_size, required);
        const std::optional<std::string> text = thread_result();
        if (!text) {
            if (required) *required = 0;
            return IAD_ERR_NOT_READY;
        }
        write_text_output(*text, buffer, buffer_size, required);
        return IAD_OK;
    } catch (...) {
        return error_code(std::current_exception());
    }
}

iad_result IAD_CALL iad_cell_link_monitor_check_now(iad_cell* cell, const char* axes, iad_link_check* checks,
                                                    size_t capacity, size_t* count) {
    return guarded([&] {
        if (count) *count = 0;
        const auto e = find_cell_to_run(cell);  // a fault stops the others on core threads: refused inside a bus hold
        std::shared_ptr<recovery::LinkMonitor> monitor;
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            monitor = e->link_monitor;
        }
        if (!monitor) throw EcNotReadyError("no link monitor runs on this cell (iad_cell_monitor_links)");
        const std::vector<std::string> wanted = split_list(axes);
        const std::map<std::string, recovery::LinkCheck> pass = monitor->check_now();
        std::vector<std::string> names = wanted;
        if (names.empty())
            for (const auto& kv : pass) names.push_back(kv.first);
        for (std::size_t i = 0; i < names.size() && checks && i < capacity; ++i) {
            const auto it = pass.find(names[i]);
            if (it != pass.end()) {
                to_c(it->second, &checks[i]);
                continue;
            }
            // not in the pass: its check threw (the monitor's errors() has it) or the monitor does not watch it
            recovery::LinkCheck failed;
            failed.state = "error";
            failed.error_text = "not checked in this pass (see the monitor's errors)";
            failed.axis = names[i];
            to_c(failed, &checks[i]);
            checks[i].ok = IAD_FALSE;
        }
        if (count) *count = names.size();
    });
}
