/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/monitor/hardware/gps/gps_monitor.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(gps_hardware_name, "GPS", "Name of the GPS hardware.");
DEFINE_string(gps_monitor_name, "GpsMonitor", "Name of the GPS monitor.");
DEFINE_double(gps_monitor_interval, 3, "GPS status checking interval (s).");

namespace apollo {
namespace monitor {

using apollo::common::adapter::AdapterManager;
using apollo::common::gnss_status::InsStatus;

GpsMonitor::GpsMonitor() : RecurrentRunner(FLAGS_gps_monitor_name,
                                           FLAGS_gps_monitor_interval) {
  CHECK(AdapterManager::GetGnssStatus()) <<
      "GnssStatusAdapter is not initialized.";
  CHECK(AdapterManager::GetInsStatus()) <<
      "InsStatusAdapter is not initialized.";
}

void GpsMonitor::RunOnce(const double current_time) {
  static auto *status = MonitorManager::GetHardwareStatus(
      FLAGS_gps_hardware_name);
  // Check Gnss status.
  auto *gnss_status_adapter = AdapterManager::GetGnssStatus();
  gnss_status_adapter->Observe();
  if (gnss_status_adapter->Empty()) {
    status->set_status(HardwareStatus::ERR);
    status->set_msg("No GNSS status message.");
    return;
  }
  if (!gnss_status_adapter->GetLatestObserved().solution_completed()) {
    status->set_status(HardwareStatus::ERR);
    status->set_msg("GNSS solution uncompleted.");
    return;
  }

  // Check Ins status.
  auto *ins_status_adapter = AdapterManager::GetInsStatus();
  ins_status_adapter->Observe();
  if (ins_status_adapter->Empty()) {
    status->set_status(HardwareStatus::ERR);
    status->set_msg("No INS status message.");
    return;
  }
  switch (ins_status_adapter->GetLatestObserved().type()) {
    case InsStatus::CONVERGING:
      status->set_status(HardwareStatus::NOT_READY);
      status->set_msg("INS ALIGNING");
      break;
    case InsStatus::GOOD:
      status->set_status(HardwareStatus::OK);
      status->set_msg("OK");
      break;
    case InsStatus::INVALID:
    default:
      status->set_status(HardwareStatus::ERR);
      status->set_msg("INS status invalid.");
      break;
  }
}

}  // namespace monitor
}  // namespace apollo
