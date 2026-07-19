export function resultDescription(code) {
  return ({
    0: "Command accepted",
    1: "Unsupported command",
    2: "Firmware/browser protocol mismatch",
    3: "Value is outside the firmware constraints",
    4: "Machine is not safely disarmed",
    5: "Could not save to device storage"
  })[code] ?? `Unknown firmware error ${code}`;
}

export function configurationPreparation(state, characterizationRunning, subject = "configuration") {
  if (characterizationRunning) {
    return { blocked: `Abort motor characterization before changing ${subject}.` };
  }
  if (state === 3) {
    return { blocked: `Clear and recheck the active firmware fault before changing ${subject}.` };
  }
  // Always issue STOP_RUN before configuration. Besides making the safety
  // transition explicit, this avoids a race with the one-second heartbeat
  // when the browser's last observed state is stale.
  return { firstCommand: "stop" };
}

export function profilePreparation(state, characterizationRunning) {
  return configurationPreparation(state, characterizationRunning, "the profile");
}
