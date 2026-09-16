"""Display-only brake-light status for Subaru angle cars.

Inspired by SubiPilot's indicator. Use light commands, not brake pressure or
pedal position; neither implies that the vehicle has switched its lamps on.
"""


def brake_light_status(camera, brakes, now_nanos):
  values = []
  for parser, message, signal in ((camera, 'ES_DashStatus', 'Brake_Lights'),
                                  (brakes, 'ES_Brake', 'Cruise_Brake_Lights')):
    try:
      value = parser.vl[message][signal]
      stamp = parser.ts_nanos[message][signal]
    except KeyError:
      return False, False
    if stamp <= 0 or not 0 <= now_nanos - stamp <= 500_000_000:
      return False, False
    values.append(bool(value))
  return True, any(values)
