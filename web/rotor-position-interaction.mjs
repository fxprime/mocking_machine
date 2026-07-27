export function rotorPositionInteractionState({
  connected,
  referenced,
  machineState,
  faults
}) {
  const healthy = faults === 0;
  const canDrag = Boolean(
      connected && referenced && healthy &&
      (machineState === 0 || machineState === 1));
  const canCommand = canDrag && machineState === 1;
  let hint = "Connect to set rotor position";
  if (connected && !referenced) {
    hint = "Rotate past the zero index before setting position";
  } else if (connected && !healthy) {
    hint = "Clear faults before setting position";
  } else if (connected && machineState === 0) {
    hint = "Drag to preview; arm output to move";
  } else if (canCommand) {
    hint = "Drag the yellow target to move";
  } else if (connected) {
    hint = "Stop the active run before setting position";
  }
  return {
    canDrag,
    canCommand,
    hint
  };
}
