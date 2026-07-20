export function nextAvailableProfileId(profiles, maximumProfiles = 8) {
  if (!Number.isInteger(maximumProfiles) || maximumProfiles < 1) {
    throw new RangeError("Maximum profile count must be positive.");
  }
  if (profiles.length >= maximumProfiles) {
    return undefined;
  }
  const used = new Set(profiles.map(profile => Number(profile.id)));
  for (let id = 0; id < 0xffff; id++) {
    if (!used.has(id)) return id;
  }
  return undefined;
}
