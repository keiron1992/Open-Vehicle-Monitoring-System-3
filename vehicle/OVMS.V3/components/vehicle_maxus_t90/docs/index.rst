Maxus T90 EV (MT90)
===================

Vehicle type code: ``MT90``

The Maxus T90 EV module provides basic battery, temperature, lock/odometer and
AC charge-line integration using the vehicle OBD-II port and a single CAN bus
at 500 kbps.

The implementation is still under active development; this page reflects the
current feature set in the initial versions of the module and some parts are
marked as experimental where scaling or semantics are still being refined.


Hardware & Installation
-----------------------

.. list-table::
   :widths: 40 60
   :header-rows: 1

   * - Item
     - Notes
   * - OVMS hardware
     - OVMS v3 module (or later)
   * - Vehicle connection
     - OBD-II port using the standard OVMS OBD-II to DB9 data cable
   * - CAN bus
     - CAN1 at 500 kbps, active mode
   * - GPS / GSM antennas
     - Standard OVMS antennas (or compatible) as per OVMS documentation


Feature Coverage
----------------

.. list-table::
   :widths: 45 15 40
   :header-rows: 1

   * - Function
     - Status
     - Notes
   * - SOC display
     - Yes
     - From OBD-II PID ``0xE002`` → ``ms_v_bat_soc``
   * - SOH display
     - Yes
     - From OBD-II PID ``0xE003`` → ``ms_v_bat_soh`` (with filtering)
   * - Battery capacity (usable)
     - Yes
     - Nominal pack capacity from ``xmt.b.capacity`` (88.5 kWh) × SOH → ``ms_v_bat_capacity``
   * - Odometer
     - Yes
     - From CAN ID ``0x540`` → ``ms_v_pos_odometer`` (0.1 km resolution)
   * - Vehicle READY / ignition state
     - Yes
     - From OBD-II PID ``0xE004`` → ``ms_v_env_on`` and poll state control
   * - Lock status
     - Yes
     - From CAN ID ``0x281`` → ``ms_v_env_locked`` (locked/unlocked)
   * - Parking brake / handbrake
     - Yes
     - From CAN ID ``0x266`` bit 0 → ``ms_v_env_handbrake`` (and optional ``xmt.v.brake.park``)
   * - Charge plug / pilot present
     - Yes
     - From OBD-II PID ``0xE009`` → ``ms_v_charge_pilot``
   * - AC charge voltage
     - Yes (experimental)
     - From CAN ID ``0x362`` → ``ms_v_charge_voltage`` (V, sanity-filtered)
   * - AC charge current
     - Yes (experimental)
     - From CAN ID ``0x373`` → ``ms_v_charge_current`` (A, empirical scaling)
   * - AC charge power
     - Yes (experimental)
     - Derived from voltage × current → ``ms_v_charge_power`` (single-phase AC only)
   * - Cabin / coolant temperature
     - Yes
     - From OBD-II PID ``0xE010`` → custom metric ``xmt.v.hvac.temp`` (°C, with filtering)
   * - Ambient temperature
     - Yes
     - From OBD-II PID ``0xE025`` → ``ms_v_env_temp`` (°C, with filtering)
   * - GPS location
     - Yes
     - Provided by the OVMS modem GPS (not vehicle-specific)
   * - Speed display
     - No (vehicle-specific)
     - Only GPS-based speed available via OVMS core
   * - Gear selector state
     - Partial (experimental)
     - Candidate signal from CAN ID ``0x510`` is currently logged / exposed as a raw byte
   * - Charge state / energy counters
     - No
     - State machine and kWh counters not yet implemented for this vehicle
   * - Charge control (start/stop, limits)
     - No
     - Not yet implemented
   * - Charging interruption alerts
     - No
     - Not yet implemented
   * - Trip counters / consumption
     - No
     - Not yet implemented for MT90
   * - TPMS
     - No
     - No TPMS integration yet
   * - Door/window state
     - No
     - Only global lock/unlock and parking brake are currently decoded
   * - Remote lock/unlock control
     - No
     - Read-only lock and brake status only
   * - Pre-heat / HVAC remote control
     - No
     - Not yet implemented
   * - Valet mode
     - No
     - Not implemented for this vehicle
   * - AC / DC charge mode detection
     - No
     - Not yet implemented


Implementation Notes
--------------------

* The module derives from ``OvmsVehicleOBDII`` and registers CAN1 at
  500 kbps in active mode.
* Polling is done on ECU ``0x7E3 / 0x7EB`` using extended OBD-II PIDs.
* The poller currently defines three poll states:

  * State 0: vehicle off  
  * State 1: vehicle on / driving  
  * State 2: charging (reserved for future use)

* In the current implementation only the READY flag (PID ``0xE004``) is polled
  in state 0 to avoid keeping ECUs awake while parked; other PIDs are polled
  when the vehicle is on.
* The READY bitfield is used to drive ``ms_v_env_on`` and to switch poll
  states between 0 (off) and 1 (on).

* Odometer is taken from CAN ID ``0x540`` using bytes [4..6] as a 24-bit
  little-endian value with 0.1 km resolution. The value is range-checked before
  updating ``ms_v_pos_odometer``.

* Lock status is decoded from CAN ID ``0x281`` (body control module) using
  byte 1 values:

  * ``0xA9`` → locked  
  * ``0xA8`` → unlocked  

  and mapped to the standard metric ``ms_v_env_locked`` with change logging.

* Parking brake status is decoded from CAN ID ``0x266`` using bit 0 of byte 2:

  * ``0`` → handbrake off  
  * ``1`` → handbrake on  

  The flag is mapped to ``ms_v_env_handbrake`` and may optionally be mirrored
  into a custom metric ``xmt.v.brake.park`` for debugging.

* AC charge line measurements:

  * CAN ID ``0x362``: bytes [0..1] are treated as a big-endian 16-bit value,
    scaled to Volts (``V = raw / 100``) and assigned to ``ms_v_charge_voltage``
    if within a plausible mains range (roughly 150–280 V).
  * CAN ID ``0x373``: bytes [0..1] are treated as a big-endian 16-bit value.
    An empirically derived scaling factor is used to obtain Amps from logs
    (single-phase EVSE, ~3.3 kW at ~226 V), and the result is range-checked
    before updating ``ms_v_charge_current``.

  When both voltage and current are valid, AC charge power in kW is
  calculated as:

  ``P_kW = (V * I) / 1000``

  and written to ``ms_v_charge_power``. These signals are marked experimental
  as the scaling has been inferred from limited captures and may be refined.

* HVAC / coolant and ambient temperatures are taken from PIDs ``0xE010`` and
  ``0xE025`` respectively, with multiple sanity filters applied to ignore
  default or bogus readings that tend to appear when the vehicle is off or
  an ECU returns fallback frames.

* A nominal pack capacity of 88.5 kWh is exposed via ``xmt.b.capacity`` and
  combined with SOH to derive a usable capacity in ``ms_v_bat_capacity``.
  This allows OVMS to provide more realistic range estimates based on battery
  ageing.

* A candidate gear selector signal has been observed on CAN ID ``0x510``.
  At present this is treated as an experimental/raw value only and is primarily
  exposed for logging/inspection. Once the semantics are fully understood it
  may be mapped onto ``ms_v_env_gear``.


Planned / Potential Extensions
------------------------------

The following features are candidates for future updates once the relevant
PIDs and CAN messages have been fully reverse engineered:

* Full charge state machine (stopped / charging / done) and energy counters.
* Distinguishing AC vs. DC charging based on additional CAN or OBD-II data.
* Refinement of AC current scaling and validation across more charge currents.
* More detailed BMS data (cell voltages, min/max temperatures, etc.).
* Additional body / door / window / tailgate state.
* Mapping the gear selector candidate to the standard gear metric.
* Remote climate control and other remote vehicle actions, if feasible.
