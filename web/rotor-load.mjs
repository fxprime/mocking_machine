export const ROTOR_SLOT_COUNT = 12;
export const MAX_LOAD_STRENGTH = 10;

export function slotPosition(slot, radius = 150, center = 210) {
  if (!Number.isInteger(slot) || slot < 0 || slot >= ROTOR_SLOT_COUNT) throw new RangeError("slot");
  const radians = (slot * 30 - 90) * Math.PI / 180;
  return { x: center + radius * Math.cos(radians), y: center + radius * Math.sin(radians) };
}

export function normalizeRotorLoads(loads) {
  const unique = new Map();
  for (const load of loads ?? []) {
    const slot = Number(load.slot);
    const strength = Number(load.strength);
    if (!Number.isInteger(slot) || slot < 0 || slot >= ROTOR_SLOT_COUNT ||
        !Number.isInteger(strength) || strength < 1 || strength > MAX_LOAD_STRENGTH) {
      throw new RangeError("Rotor load must use a unique slot 0–11 and integer strength 1–10");
    }
    if (unique.has(slot)) throw new RangeError("Rotor slots must be unique");
    unique.set(slot, { slot, position: slot * 30, strength });
  }
  return [...unique.values()].sort((a, b) => a.slot - b.slot);
}

export function synchronizeRotorLoadDraft(draft, saved, incoming) {
  const normalizedDraft = normalizeRotorLoads(draft);
  const normalizedSaved = normalizeRotorLoads(saved);
  const normalizedIncoming = normalizeRotorLoads(incoming);
  const draftIsDirty = JSON.stringify(normalizedDraft) !== JSON.stringify(normalizedSaved);
  return {
    draft: (draftIsDirty ? normalizedDraft : normalizedIncoming).map(load => ({ ...load })),
    saved: normalizedIncoming.map(load => ({ ...load })),
    preservedDraft: draftIsDirty
  };
}
