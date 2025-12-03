/**
 *  vehicle_maxt90.h
 *
 *  Maxus T90 EV support for OVMS.
 *
 *  This declares a light-weight vehicle module derived from OvmsVehicleOBDII.
 *  The implementation combines:
 *
 *    - Vendor extended OBD-II PIDs (VIN, SOC, SOH, READY, plug, temps…)
 *    - Native CAN frames for things that aren’t exposed as PIDs
 *      (lock state, parking brake, AC charge info, odometer, gear candidate).
 *
 *  Most native signals are still partially reverse-engineered from logs, so
 *  expect some of the semantics / scaling to evolve as more data arrives.
 */

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
  // ─────────────────────────────────────────────
  //  OBD-II poll replies
  // ─────────────────────────────────────────────
  //
  // Handles responses to the extended PIDs defined in the poll list, e.g.:
  //   - 0xF190 : VIN (ASCII)
  //   - 0xE002 : SOC (%)
  //   - 0xE003 : SOH (% * 100)
  //   - 0xE004 : READY flag (bitfield)
  //   - 0xE009 : Plug present (bitfield → charge pilot)
  //   - 0xE010 : HVAC / coolant temperature (0.1 °C units)
  //   - 0xE025 : Ambient temperature (0.1 °C units)
  //
  // See vehicle_maxt90.cpp for the actual mappings and filters.
  void IncomingPollReply(const OvmsPoller::poll_job_t& job,
                         uint8_t* data, uint8_t length) override;

  // ─────────────────────────────────────────────
  //  Raw CAN1 frames
  // ─────────────────────────────────────────────
  //
  // Decodes native Maxus frames observed on CAN1 and maps them onto
  // standard OVMS metrics where possible:
  //
  //   0x266 : Parking brake (handbrake lever status)
  //   0x281 : Lock state (BCM – locked / unlocked)
  //   0x362 : AC line voltage   (candidate, V * 100)
  //   0x373 : AC line current   (candidate, empirical scaling)
  //   0x510 : Gear selector candidate (currently logged only)
  //   0x540 : Odometer (24-bit little-endian, 0.1 km resolution)
  //
  // Anything not explicitly handled here is currently ignored.
  void IncomingFrameCan1(CAN_frame_t* p_frame) override;

private:
  // ─────────────────────────────────────────────
  //  Custom metrics
  // ─────────────────────────────────────────────
  //
  // These use the "xmt" (experimental Maxus T90) prefix to avoid
  // clashing with standard metrics and to match other OVMS vehicles:
  //
  //   xmt.v.hvac.temp : HVAC / coolant temperature in °C (from PID 0xE010)
  //   xmt.b.capacity  : Nominal pack capacity in kWh (before SOH scaling)
  //
  // The nominal capacity is used together with SOH to derive
  // ms_v_bat_capacity (usable capacity).
  OvmsMetricFloat* m_hvac_temp_c       = nullptr;
  OvmsMetricFloat* m_pack_capacity_kwh = nullptr;

  // Cached AC charge line values derived from 0x362 / 0x373:
  //
  //   - 0x362 → m_ac_voltage  (V * 100 → Volts)
  //   - 0x373 → m_ac_current  (empirical factor → Amps)
  //
  // These are combined to populate the standard charge metrics:
  //   ms_v_charge_voltage
  //   ms_v_charge_current
  //   ms_v_charge_power
  float m_ac_voltage = 0.0f;  // Volts
  float m_ac_current = 0.0f;  // Amps

  // ─────────────────────────────────────────────
  //  Helpers
  // ─────────────────────────────────────────────
  //
  // Simple byte-order helpers used by the decoder. Kept inline so
  // the compiler can optimise them away easily.

  // Big-endian 16-bit:
  //   raw = (p[0] << 8) | p[1]
  static inline uint16_t u16be(const uint8_t* p)
  {
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
  }

  // Little-endian 24-bit:
  //   raw = p[0] + p[1]*256 + p[2]*65536
  // Used by the odometer decoder for CAN ID 0x540.
  static inline uint32_t u24le(const uint8_t* p)
  {
    return (uint32_t(p[0]))
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16);
  }
};

#endif // __VEHICLE_MAXT90_H__
