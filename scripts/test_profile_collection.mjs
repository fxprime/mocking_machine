import assert from "node:assert/strict";
import { nextAvailableProfileId } from "../web/profile-collection.mjs";

assert.equal(nextAvailableProfileId([{ id: 0 }]), 1);
assert.equal(nextAvailableProfileId([{ id: 0 }, { id: 2 }]), 1);
assert.equal(nextAvailableProfileId(Array.from({ length: 8 }, (_, id) => ({ id }))), undefined);

console.log("Profile collection tests passed");
