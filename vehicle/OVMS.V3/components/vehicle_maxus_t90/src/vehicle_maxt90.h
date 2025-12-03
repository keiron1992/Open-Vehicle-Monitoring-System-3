#ifndef __VEHICLE_MAXT90_H__
#define __VEHICLE_MAXT90_H__

#include "vehicle_obdii.h"
#include "ovms_metrics.h"

class OvmsVehicleMaxt90 : public OvmsVehicleOBDII
{
public:
  OvmsVehicleMaxt90();
  ~OvmsVehicleMaxt90();

protected:
  // OBD-II poll replies (VIN, SOC, SOH, READY, Plug, Temps, …):
  void IncomingPollReply(const OvmsPoller::poll_job_t& job,
                         uint8_t* data, uint8_t length) override;

  // Raw CAN1 frames:
  //  - 0x281 : lock status (BCM)
  //  - 0x362 : AC charge voltage / current (candidate)
  //  - 0x373 : AC charge voltage / current (candidate)
  //  - 0x540 : odometer
  void IncomingFrameCan1(CAN_frame_t* p_frame) override;

private:
  // ─────────────────────────────────────────────
  //  Custom metrics
  // ─────────────────────────────────────────────
  //  xmt.v.hvac.temp : HVAC / coolant temperature (°C)
  //  xmt.b.capacity  : Nominal pack capacity (kWh, before SOH)
  OvmsMetricFloat* m_hvac_temp_c       = nullptr;
  OvmsMetricFloat* m_pack_capacity_kwh = nullptr;

  // Cached AC charge line values derived from 0x362 / 0x373:
  // Used to populate standard charge metrics:
  //   - ms_v_charge_voltage
  //   - ms_v_charge_current
  //   - ms_v_charge_power
float m_ac_voltage = 0.0f;  // Volts
float m_ac_current = 0.0f;  // Amps

  // ─────────────────────────────────────────────
  //  Helpers
  // ─────────────────────────────────────────────

  // Big-endian 16-bit:
  static inline uint16_t u16be(const uint8_t* p)
  {
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
  }

  // Little-endian 24-bit:
  //  raw = p[0] + p[1]*256 + p[2]*65536
  static inline uint32_t u24le(const uint8_t* p)
  {
    return (uint32_t(p[0]))
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16);
  }
};

#endif // __VEHICLE_MAXT90_H__
