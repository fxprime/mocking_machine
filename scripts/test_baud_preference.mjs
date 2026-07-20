import assert from "node:assert/strict";
import { BAUD_STORAGE_KEY, readBaudPreference, writeBaudPreference } from "../web/baud-preference.mjs";

const values = new Map();
const storage = {
  getItem: key => values.get(key) ?? null,
  setItem: (key, value) => values.set(key, value)
};
assert.equal(readBaudPreference(storage), 115200);
assert.equal(writeBaudPreference(storage, 460800), true);
assert.equal(values.get(BAUD_STORAGE_KEY), "460800");
assert.equal(readBaudPreference(storage), 460800);
assert.equal(writeBaudPreference(storage, 12345), false);
values.set(BAUD_STORAGE_KEY, "invalid");
assert.equal(readBaudPreference(storage), 115200);

console.log("Baud preference tests passed");
