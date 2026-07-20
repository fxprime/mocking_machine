export function machineStatusKey(state, faults) {
  return `${Number(state)}:${Number(faults) >>> 0}`;
}

export function shouldRefreshMachineUi(previousKey, state, faults) {
  return previousKey !== machineStatusKey(state, faults);
}
