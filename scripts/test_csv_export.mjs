import assert from "node:assert/strict";
import { createTelemetryCsv } from "../web/csv-export.mjs";

const csv = createTelemetryCsv([{
  timestamp: 1742528221,
  profile: 0,
  desired: 0.27975133061408997,
  measured: -1.401298464324817e-45,
  output: -0.02427089959383011,
  pTerm: 0.0004440486372914165,
  iTerm: -0.00009577703167451546,
  dTerm: 0,
  current: 2.2100000381469727,
  supplyVoltage: 12.073293685913086,
  zeroTime: 0,
  count: 1505048n,
  zeroCount: 0n,
  rotorPosition: 270.25,
  zeroSequence: 3,
  zeroRejected: 2,
  load: 0,
  state: 2,
  faults: 0
}]);

const [header, row] = csv.split("\n");
assert.match(header, /desired_velocity_rad_s,measured_velocity_rad_s/);
assert.match(header, /rotor_position_deg,last_zero_timestamp_us,encoder_count,last_zero_encoder_count,zero_index_sequence,zero_index_rejected_count/);
assert.equal(row,
  "1742528221,0,0.2798,0.0000,-0.024271,0.000444,-0.000096,0.000000,2.2100,12.0733,270.250,0,1505048,0,3,2,0,2,0");
assert.doesNotMatch(row, /e-/i);

console.log("CSV export tests passed");
