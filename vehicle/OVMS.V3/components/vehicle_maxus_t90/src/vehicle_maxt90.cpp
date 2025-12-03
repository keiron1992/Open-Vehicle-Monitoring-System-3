/**
 *  vehicle_maxt90.cpp
 *
 *  Maxus T90 EV support for OVMS.
 *
 *  This module is built as a light-weight OBD-II–derived vehicle
 *  using a mix of:
 *    - Vendor extended OBD-II PIDs (for SOC, SOH, temps, READY, etc.)
 *    - Native CAN frames (locks, handbrake, AC charge info, odometer)
 *
 *  Most of the native scaling & bitfields are reverse-engineered
 *  from live logs, so some of the logic is based on observation
 *  and may be refined over time.
 */

#include "ovms_log.h"
static const char *TAG = "v-maxt90";

#include <stdio.h>
#include <string>
#include "vehicle_maxt90.h"
#include "vehicle_obdii.h"
#include "metrics_standard.h"
#include "ovms_metrics.h"

OvmsVehicleMaxt90::OvmsVehicleMaxt90()
{
  ESP_LOGI(TAG, "Initialising Maxus T90 EV vehicle module (derived from OBDII)");

  // Register CAN1 as the vehicle bus at 500 kbps.
  // All native frames decoded in this file come from CAN1.
  RegisterCanBus(1, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);

  // ─────────────────────────────────────────────
  // Custom / vehicle-specific metrics
  // ─────────────────────────────────────────────
  //
  // Prefix "xmt" = "X" (experimental) + "M"axus + "T"90
  // to match the style used by other OVMS vehicles (xnl, xmg, etc.).

  // HVAC / coolant temperature from OEM extended PID 0xE010.
  m_hvac_temp_c =
    MyMetrics.InitFloat("xmt.v.hvac.temp", 10, 0.0f, Celcius, false);

  // Nominal battery pack capacity (kWh).
  // This is used as the base value for usable capacity once SOH is known.
  m_pack_capacity_kwh =
    MyMetrics.InitFloat("xmt.b.capacity", 0, 88.5f, kWh, true);

  // Seed the standard usable capacity metric so OVMS knows
  // there is a battery with a nominal size, even before SOH
  // has been polled and applied.
  StdMetrics.ms_v_bat_capacity->SetValue(m_pack_capacity_kwh->AsFloat(), kWh);

  // ─────────────────────────────────────────────
  // OBD-II Poll List
  // ─────────────────────────────────────────────
  //
  // We currently run a single poll list with three "states":
  //   State 0: vehicle off
  //   State 1: vehicle on / driving
  //   State 2: charging (reserved for future extension)
  //
  // READY (0xE004) is polled even in state 0 to detect wake-up,
  // everything else only when the car is ON / CHARGING.

  static const OvmsPoller::poll_pid_t maxt90_polls[] = {
    // VIN (0xF190, extended OBD-II).
    // Slow rate, only when car is on / charging.
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xF190,
      { 0, 3600, 3600 }, 0, ISOTP_STD },

    // SOC (0xE002, %)
    // Polled fairly quickly while on / charging.
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE002,
      { 0, 10, 10 }, 0, ISOTP_STD },

    // SOH (0xE003, % * 100)
    // Slow rate, this doesn’t change often.
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE003,
      { 0, 1800, 1800 }, 0, ISOTP_STD },

    // READY flag (0xE004, bitfield)
    // Polled in all states: faster in "off" to catch wake-up quickly.
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE004,
      { 5, 10, 10 }, 0, ISOTP_STD },

    // AC plug present (0xE009, bitfield)
    // Used to drive the standard charge pilot metric.
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE009,
      { 0, 10, 10 }, 0, ISOTP_STD },

    // HVAC / coolant temperature (0xE010, 0.1 °C units)
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE010,
      { 0, 30, 30 }, 0, ISOTP_STD },

    // Ambient temperature (0xE025, 0.1 °C units)
    { 0x7e3, 0x7eb, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0xE025,
      { 0, 30, 30 }, 0, ISOTP_STD },

    POLL_LIST_END
  };

  // Attach the poll list to CAN1 & start in "off" state.
  // When we see READY go true, we switch to state 1 in IncomingPollReply().
  PollSetPidList(m_can1, maxt90_polls);
  PollSetState(0);

  // Cached AC line values derived from native CAN frames:
  m_ac_voltage = 0.0f;  // from CAN ID 0x362 (V * 100)
  m_ac_current = 0.0f;  // from CAN ID 0x373 (scaled)

  ESP_LOGI(TAG, "Maxus T90 EV poller configured on CAN1 @ 500 kbps");
}

OvmsVehicleMaxt90::~OvmsVehicleMaxt90()
{
  ESP_LOGI(TAG, "Shutdown Maxus T90 EV vehicle module");
}

// ─────────────────────────────────────────────
//  Live CAN Frame Handler
//  (Locks / handbrake / AC charge / odometer / gear candidate)
// ─────────────────────────────────────────────
//
// This processes *native* Maxus CAN frames seen on CAN1 and
// maps them onto standard OVMS metrics where possible.
//
void OvmsVehicleMaxt90::IncomingFrameCan1(CAN_frame_t* p_frame)
{
  // Let the base OBD-II vehicle process the frame as well
  // (for generic logging / diagnostics).
  OvmsVehicleOBDII::IncomingFrameCan1(p_frame);

  // Ignore frames from other buses if any are configured.
  if (p_frame->origin != m_can1)
    return;

  const uint8_t* d = p_frame->data.u8;

  switch (p_frame->MsgID)
  {
    // ───────────── Handbrake (parking brake) – CAN ID 0x266 ─────────────
    //
    // From measurements: bit 0 of byte[2] flips with the handbrake
    // lever. We map this straight into the standard ms_v_env_handbrake
    // metric so apps and Home Assistant can use it.
    case 0x266:
    {
      if (p_frame->FIR.B.DLC < 3)
        break;

      bool handbrake_on = (d[2] & 0x01) != 0;

      StdMetrics.ms_v_env_handbrake->SetValue(handbrake_on);

      ESP_LOGD(TAG, "Parking brake: %s (CAN 0x266 byte2 bit0)",
               handbrake_on ? "ON" : "OFF");
      break;
    }

    // ───────────── Lock state – CAN ID 0x281 ─────────────
    //
    // Byte[1] toggles between:
    //   0xA9 = locked
    //   0xA8 = unlocked
    //
    // We de-bounce this by only logging when the value changes.
    case 0x281:
    {
      uint8_t state = d[1];
      static uint8_t last_state = 0x00;

      if (state != last_state && (state == 0xA9 || state == 0xA8))
      {
        bool locked = (state == 0xA9);

        // Standard OVMS metric for "vehicle locked" –
        // consumed by the mobile app, MQTT, HA, etc.
        StdMetrics.ms_v_env_locked->SetValue(locked);

        ESP_LOGI(TAG, "Lock state changed: %s (CAN 0x281 byte1=0x%02x)",
                 locked ? "LOCKED" : "UNLOCKED", state);

        last_state = state;
      }
      break;
    }

    // ───────────── AC line voltage – CAN ID 0x362 ─────────────
    //
    // Observed format:
    //   bytes[0..1] = voltage in V * 100 (big-endian).
    // Typical values: ~22600 → 226.00 V
    //
    // This is mapped onto the standard ms_v_charge_voltage metric.
    case 0x362:
    {
      uint16_t raw = u16be(&d[0]);     // bytes 0–1
      float v = raw / 100.0f;          // Volts

      // Sanity range for UK / EU single-phase mains.
      if (v >= 150.0f && v <= 280.0f)
      {
        m_ac_voltage = v;
        StdMetrics.ms_v_charge_voltage->SetValue(v);
        ESP_LOGD(TAG, "AC line voltage: %.2f V (raw=0x%04x)", v, raw);

        // If we already know an AC current, update AC charge power.
        if (m_ac_current > 0.1f)
        {
          float p_kw = (m_ac_voltage * m_ac_current) / 1000.0f;
          StdMetrics.ms_v_charge_power->SetValue(p_kw);
          ESP_LOGD(TAG, "AC charge power: %.3f kW", p_kw);
        }
      }
      else
      {
        ESP_LOGV(TAG, "AC voltage raw=0x%04x (%.2f V) out of range, ignored",
                 raw, v);
      }
      break;
    }

    // ───────────── AC line current – CAN ID 0x373 ─────────────
    //
    // Observed example:
    //   raw = 2388 when the car reports ≈3.28 kW @ ≈226.1 V → ~14.5 A.
    //
    // That implies:
    //   I ≈ raw / 164 (i.e. one LSB ≈ 6 mA)
    //
    // We treat this as empirical and may refine the factor if
    // more capture data becomes available.
    case 0x373:
    {
      uint16_t raw = u16be(&d[0]);   // bytes 0–1
      float i = raw / 164.0f;        // Amps (≈ 6 mA resolution)

      // Sanity range for typical single-phase EVSE currents.
      if (i >= 0.1f && i <= 40.0f)
      {
        m_ac_current = i;
        StdMetrics.ms_v_charge_current->SetValue(i);
        ESP_LOGD(TAG, "AC line current: %.2f A (raw=0x%04x)", i, raw);

        // Combine with voltage (from 0x362) to derive AC charge power.
        if (m_ac_voltage > 10.0f)
        {
          float p_kw = (m_ac_voltage * m_ac_current) / 1000.0f;
          StdMetrics.ms_v_charge_power->SetValue(p_kw);
          ESP_LOGD(TAG, "AC charge power: %.3f kW", p_kw);
        }
      }
      else
      {
        ESP_LOGV(TAG, "AC current raw=0x%04x scaled out of range, ignored", raw);
      }
      break;
    }

    // ───────────── Gear candidate – CAN ID 0x510 ─────────────
    //
    // We initially suspected this held gear selector state,
    // but current captures show a fixed payload:
    //   10 49 5c 33 00 10 1e 00
    //
    // i.e. d[3] = 0x33 regardless of selector position.
    // For now we *only* log this for debugging and DO NOT map
    // it to ms_v_env_gear to avoid publishing bogus data.
    case 0x510:
    {
      if (p_frame->FIR.B.DLC < 4)
        break;

      uint8_t raw = d[3];
      ESP_LOGD(TAG, "Gear candidate frame 0x510: raw byte3=0x%02x", raw);
      break;
    }

    // ───────────── Odometer – CAN ID 0x540 ─────────────
    //
    // Observed example:
    //   540 00 00 00 00 90 f0 02 00
    //
    // We treat bytes [4..6] as a 24-bit little-endian value with 0.1 km
    // resolution:
    //   raw = d4 + d5*256 + d6*65536
    //   km  = raw / 10
    //
    // This is mapped to the standard ms_v_pos_odometer metric.
    case 0x540:
    {
      uint32_t raw =
        (uint32_t)d[4] |
        ((uint32_t)d[5] << 8) |
        ((uint32_t)d[6] << 16);

      float km = raw / 10.0f;

      // Reasonable odometer sanity bounds (0 … 1,000,000 km).
      if (km > 0 && km < 1000000.0f)
      {
        if (StdMetrics.ms_v_pos_odometer->AsFloat() != km)
        {
          StdMetrics.ms_v_pos_odometer->SetValue(km);
          ESP_LOGI(TAG, "Odometer: %.1f km (raw=0x%06x)", km, raw);
        }
      }
      else
      {
        ESP_LOGW(TAG, "Odometer raw=0x%06x (%.1f km) out of range, ignored",
                 raw, km);
      }
      break;
    }

    default:
      // Unknown / currently unused native CAN frame – ignore.
      break;
  }
}

// ─────────────────────────────────────────────
//  OBD-II Poll Reply Handler
// ─────────────────────────────────────────────
//
// This handles responses to the extended OBD-II PIDs defined
// in the poll list above and maps them to OVMS metrics.
//
void OvmsVehicleMaxt90::IncomingPollReply(const OvmsPoller::poll_job_t& job,
                                          uint8_t* data, uint8_t length)
{
  switch (job.pid)
  {
    // ───────────── VIN – PID 0xF190 ─────────────
    case 0xF190:
    {
      if (length >= 1) {
        std::string vin(reinterpret_cast<char*>(data),
                        reinterpret_cast<char*>(data) + length);
        StdMetrics.ms_v_vin->SetValue(vin);
        ESP_LOGD(TAG, "VIN: %s", vin.c_str());
      }
      break;
    }

    // ───────────── SOC – PID 0xE002 ─────────────
    //
    // Simple 0–100 % value in the first byte.
    case 0xE002:
    {
      if (length >= 1) {
        float soc = data[0];
        if (soc > 0 && soc <= 100) {
          if (StdMetrics.ms_v_bat_soc->AsFloat() != soc) {
            StdMetrics.ms_v_bat_soc->SetValue(soc);
            ESP_LOGD(TAG, "SOC: %.0f %%", soc);
          }
        } else {
          // When the car is off / times out, we sometimes see bogus values.
          ESP_LOGW(TAG,
                   "Invalid SOC %.1f ignored (car likely off or poll timeout)",
                   soc);
        }
      }
      break;
    }

    // ───────────── SOH – PID 0xE003 ─────────────
    //
    // Format: u16 big-endian, units are % * 100.
    // We also use this to derive the usable pack capacity
    // from the nominal capacity metric.
    case 0xE003:
    {
      if (length >= 2) {
        uint16_t raw = u16be(data);
        float soh = raw / 100.0f;

        // Filter out obviously bogus default values:
        //  - 0xFFFF: typical error / timeout pattern
        //  - 0x1800 (~61.44 %): seen as a placeholder
        //  - outside 50–150 % range is suspicious
        if (raw == 0xFFFF || raw == 0x1800 || soh <= 50.0f || soh > 150.0f) {
          ESP_LOGW(TAG, "Invalid SOH raw=0x%04x (%.2f %%) ignored", raw, soh);
          break;
        }

        if (StdMetrics.ms_v_bat_soh->AsFloat() != soh) {
          StdMetrics.ms_v_bat_soh->SetValue(soh);
          ESP_LOGD(TAG, "SOH: %.2f %%", soh);

          // Update usable battery capacity [kWh] based on SOH.
          if (m_pack_capacity_kwh) {
            float usable_kwh = m_pack_capacity_kwh->AsFloat() * (soh / 100.0f);
            StdMetrics.ms_v_bat_capacity->SetValue(usable_kwh, kWh);
            ESP_LOGD(TAG,
                     "Usable battery capacity: %.2f kWh (nom=%.2f, SOH=%.2f %%)",
                     usable_kwh, m_pack_capacity_kwh->AsFloat(), soh);
          }
        }
      }
      break;
    }

    // ───────────── READY flag – PID 0xE004 ─────────────
    //
    // 16-bit bitfield. We currently treat any non-zero of bits 2 or 3
    // as "vehicle ON / READY".
    //
    // This drives:
    //   - ms_v_env_on
    //   - poller state transitions (0 ⇄ 1)
    case 0xE004:
    {
      if (length >= 2) {
        uint16_t v = u16be(data);
        bool ready = (v & 0x000C) != 0;
        bool prev_ready = StdMetrics.ms_v_env_on->AsBool();
        StdMetrics.ms_v_env_on->SetValue(ready);

        if (ready != prev_ready) {
          ESP_LOGI(TAG, "READY flag changed: raw=0x%04x ready=%s",
                   v, ready ? "true" : "false");

          if (!ready && m_poll_state != 0) {
            ESP_LOGI(TAG, "Vehicle OFF detected, setting poll state 0");
            PollSetState(0);
          }
          else if (ready && m_poll_state == 0) {
            ESP_LOGI(TAG, "Vehicle ON detected, setting poll state 1");
            PollSetState(1);
          }
        }
      }
      break;
    }

    // ───────────── Plug present – PID 0xE009 ─────────────
    //
    // Format is a bitfield; empirically we treat "low byte == 0"
    // as "plug present". This drives the standard charge pilot
    // metric used by OVMS to decide if the car is plugged in.
    case 0xE009:
    {
      if (length >= 2) {
        uint16_t v = u16be(data);
        bool plug_present = ((v & 0x00FF) == 0x00);
        StdMetrics.ms_v_charge_pilot->SetValue(plug_present);
        ESP_LOGD(TAG, "Plug present: raw=0x%04x plug=%s",
                 v, plug_present ? "true" : "false");
      }
      break;
    }

    // ───────────── HVAC / coolant temperature – PID 0xE010 ─────────────
    //
    // Format: u16 big-endian, 0.1 °C units.
    // When the car is off, we see a stable bogus value (45.8 °C),
    // so we filter that out along with some other obvious defaults.
    case 0xE010:
    {
      if (length >= 2 && m_hvac_temp_c) {
        uint16_t raw = u16be(data);
        float t = raw / 10.0f;

        bool env_on = StdMetrics.ms_v_env_on->AsBool();

        // Known bogus pattern: 45.8 °C when car is off.
        if (!env_on && raw == 458) {
          ESP_LOGW(TAG,
                   "HVAC temp raw=0x%04x (%.1f °C) ignored (car off/default)",
                   raw, t);
          break;
        }

        // Generic default / error patterns and insane values.
        if (raw == 0x0200 || raw == 0xFFFF || t < -40 || t > 125) {
          ESP_LOGW(TAG, "Invalid HVAC temp raw=0x%04x (%.1f °C) ignored",
                   raw, t);
          break;
        }

        m_hvac_temp_c->SetValue(t);
        ESP_LOGD(TAG, "HVAC/Coolant temp: %.1f °C", t);
      }
      break;
    }

    // ───────────── Ambient temperature – PID 0xE025 ─────────────
    //
    // Format: u16 big-endian, 0.1 °C units.
    // We again filter known bogus values, including a default 7.5 °C
    // seen when the car is off.
    case 0xE025:
    {
      if (length >= 2) {
        uint16_t raw = u16be(data);
        float ta = raw / 10.0f;

        bool env_on = StdMetrics.ms_v_env_on->AsBool();

        // Known bogus pattern: 7.5 °C when car is off.
        if (!env_on && raw == 75) {
          ESP_LOGW(TAG,
                   "Ambient temp raw=0x%04x (%.1f °C) ignored (car off/default)",
                   raw, ta);
          break;
        }

        // Filter default / error patterns and silly values.
        if (raw == 0x0200 || raw == 0xFFFF || ta < -50 || ta > 80) {
          ESP_LOGW(TAG, "Invalid ambient temp raw=0x%04x (%.1f °C) ignored",
                   raw, ta);
          break;
        }

        StdMetrics.ms_v_env_temp->SetValue(ta);
        ESP_LOGD(TAG, "Ambient temp: %.1f °C", ta);
      }
      break;
    }

    default:
      // Unknown / not yet implemented extended PID.
      break;
  }
}

// ─────────────────────────────────────────────
//   Module Registration
// ─────────────────────────────────────────────
//
// This registers the vehicle type "MT90" with the OVMS
// vehicle factory so it can be selected via the WEB / app.
//
class OvmsVehicleMaxt90Init
{
public:
  OvmsVehicleMaxt90Init();
} MyOvmsVehicleMaxt90Init __attribute__((init_priority(9000)));

OvmsVehicleMaxt90Init::OvmsVehicleMaxt90Init()
{
  ESP_LOGI(TAG, "Registering Vehicle: Maxus T90 EV (9000)");

  // Vehicle type string "MT90" is the type code used in OVMS:
  //   config set vehicle type MT90
  MyVehicleFactory.RegisterVehicle<OvmsVehicleMaxt90>(
    "MT90", "Maxus T90 EV");
}
