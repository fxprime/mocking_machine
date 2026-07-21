import assert from "node:assert/strict";
import { createParameterCsv, parseParameterCsv } from "../web/parameter-csv.mjs";

const definitions = {
  gain: { id: 1, min: 0, max: 10, decimals: 3 },
  samples: { id: 2, min: 2, max: 32, decimals: 0 },
  direction: { id: 3, min: -1, max: 1, decimals: 0 },
  enabled: { id: 4, min: 0, max: 1, decimals: 0 },
  readonly: { decimals: 0 }
};

assert.equal(createParameterCsv(
  { gain: 1.25, samples: 5, direction: -1, enabled: true, readonly: 9 }, definitions),
  "parameter,value\r\ngain,1.25\r\nsamples,5\r\ndirection,-1\r\nenabled,1\r\n");

assert.deepEqual(parseParameterCsv(
  "\uFEFFparameter,value\r\ngain,2.5\r\nsamples,10\r\ndirection,1\r\n", definitions), [
  { parameter: "gain", id: 1, value: 2.5 },
  { parameter: "samples", id: 2, value: 10 },
  { parameter: "direction", id: 3, value: 1 }
]);

assert.throws(() => parseParameterCsv("name,value\ngain,1\n", definitions), /header/);
assert.throws(() => parseParameterCsv("parameter,value\nunknown,1\n", definitions), /unknown/);
assert.throws(() => parseParameterCsv("parameter,value\nreadonly,1\n", definitions), /read-only/);
assert.throws(() => parseParameterCsv("parameter,value\ngain,1\ngain,2\n", definitions), /duplicate/);
assert.throws(() => parseParameterCsv("parameter,value\ngain,\n", definitions), /empty/);
assert.throws(() => parseParameterCsv("parameter,value\ngain,11\n", definitions), /between/);
assert.throws(() => parseParameterCsv("parameter,value\nsamples,2.5\n", definitions), /integer/);
assert.throws(() => parseParameterCsv("parameter,value\ndirection,0\n", definitions), /-1 or 1/);

console.log("Parameter CSV tests passed");
