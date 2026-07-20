import assert from "node:assert/strict";
import { normalizeRotorLoads, slotPosition, synchronizeRotorLoadDraft } from "../web/rotor-load.mjs";

assert.deepEqual(normalizeRotorLoads([{ slot: 3, strength: 7 }, { slot: 0, strength: 1 }]), [
  { slot: 0, position: 0, strength: 1 },
  { slot: 3, position: 90, strength: 7 }
]);
assert.throws(() => normalizeRotorLoads([{ slot: 0, strength: 1 }, { slot: 0, strength: 2 }]), /unique/);
assert.throws(() => normalizeRotorLoads([{ slot: 12, strength: 1 }]), /0–11/);
assert.throws(() => normalizeRotorLoads([{ slot: 1, strength: 11 }]), /1–10/);
assert.deepEqual(slotPosition(0), { x: 210, y: 60 });
assert.ok(Math.abs(slotPosition(3).x - 360) < 1e-9);
assert.ok(Math.abs(slotPosition(3).y - 210) < 1e-9);

const lateSync = synchronizeRotorLoadDraft(
  [{ slot: 2, strength: 6 }], [], []);
assert.deepEqual(lateSync.draft, [{ slot: 2, position: 60, strength: 6 }],
  "A late firmware sync must not erase a local load draft");
assert.equal(lateSync.preservedDraft, true);
const cleanSync = synchronizeRotorLoadDraft([], [], [{ slot: 4, strength: 3 }]);
assert.deepEqual(cleanSync.draft, [{ slot: 4, position: 120, strength: 3 }]);

console.log("Rotor load model tests passed");
