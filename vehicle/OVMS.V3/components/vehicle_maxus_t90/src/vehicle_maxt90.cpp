#include "ovms_log.h"
static const char *TAG = "v-maxt90";

#include "vehicle_maxt90.h"

OvmsVehicleMaxt90::OvmsVehicleMaxt90()
{
  ESP_LOGI(TAG, "Initialising Maxus T90 EV (SOC/SOH only)");

  RegisterCanBus(1, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);

  // Poll list: Only SOC and SOH
  static const OvmsPoller::poll_pid_t maxt90_polls[] = {
    // SOC
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE002,
      { 0, 10, 10 }, 0, ISOTP_STD },
    // SOH
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE003,
      { 0, 1800, 1800 }, 0, ISOTP_STD },

    POLL_LIST_END
  };

  PollSetPidList(m_can1, maxt90_polls);
  PollSetState(1); // Force state 1 to ensure constant polling
}

OvmsVehicleMaxt90::~OvmsVehicleMaxt90() {}

void OvmsVehicleMaxt90::IncomingPollReply(const OvmsPoller::poll_job_t& job,
                                          uint8_t* data, uint8_t length)
{
  switch (job.pid)
  {
    case 0xE002: { // SOC (%)
      if (length >= 1) {
        float soc = data[0];
        if (soc >= 0 && soc <= 100) {
          StdMetrics.ms_v_bat_soc->SetValue(soc);
          ESP_LOGD(TAG, "SOC: %.0f %%", soc);
        }
      }
      break;
    }

    case 0xE003: { // SOH (%)
      if (length >= 2) {
        uint16_t raw = u16be(data);
        float soh = raw / 100.0f;
        if (soh > 50.0f && soh <= 100.0f) {
          StdMetrics.ms_v_bat_soh->SetValue(soh);
          ESP_LOGD(TAG, "SOH: %.2f %%", soh);
        }
      }
      break;
    }
  }
}

// Module Registration remains the same
class OvmsVehicleMaxt90Init { public: OvmsVehicleMaxt90Init(); };
OvmsVehicleMaxt90Init MyOvmsVehicleMaxt90Init __attribute__((init_priority(9000)));
OvmsVehicleMaxt90Init::OvmsVehicleMaxt90Init()
{
  MyVehicleFactory.RegisterVehicle<OvmsVehicleMaxt90>("MT90", "Maxus T90 EV");
}
