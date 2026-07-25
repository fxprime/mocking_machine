import assert from "node:assert/strict";
import { createLoadConfigurationCsv, createTelemetryCsv, defaultExportBaseName, sanitizeExportBaseName } from "../web/csv-export.mjs";

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
  "1742528221,0,0.2798,0.0000,-0.024271,0.000444,-0.000096,0.000000,2.2100,12.0733,270.250,0,1505048,0,3,2,0,2,0,good");
assert.doesNotMatch(row, /e-/i);
assert.match(createTelemetryCsv([{
  timestamp: 1, profile: 0, desired: 0, measured: 0, output: 0, pTerm: 0, iTerm: 0,
  dTerm: 0, current: 0, supplyVoltage: 0, rotorPosition: 0, zeroTime: 0,
  count: 0, zeroCount: 0, zeroSequence: 0, zeroRejected: 0, load: 1, state: 0, faults: 0
}], true), /,broken$/);

assert.equal(createLoadConfigurationCsv(7, [
  { slot: 2, position: 60, strength: 4 },
  { slot: 9, position: 270, strength: 10 }
]), "load_setting_id,slot_id,position_deg,strength\n7,2,60,4\n7,9,270,10");
assert.equal(sanitizeExportBaseName(" bearing test.csv "), "bearing test");
assert.equal(sanitizeExportBaseName("phase:1/run*2"), "phase-1-run-2");
assert.equal(sanitizeExportBaseName("...", "fallback"), "fallback");
assert.equal(defaultExportBaseName(new Date("2026-07-21T01:02:03.456Z")),
  "mocking-machine-run-2026-07-21T01-02-03-456Z");

console.log("CSV export tests passed");
