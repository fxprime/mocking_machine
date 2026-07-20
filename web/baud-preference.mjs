export const SUPPORTED_BAUD_RATES = [115200, 230400, 460800, 921600];
export const BAUD_STORAGE_KEY = "mocking-machine.serial-baud";

export function readBaudPreference(storage, fallback = 115200) {
  try {
    const value = Number(storage?.getItem(BAUD_STORAGE_KEY));
    return SUPPORTED_BAUD_RATES.includes(value) ? value : fallback;
  } catch {
    return fallback;
  }
}

export function writeBaudPreference(storage, baud) {
  const value = Number(baud);
  if (!SUPPORTED_BAUD_RATES.includes(value)) return false;
  try {
    storage?.setItem(BAUD_STORAGE_KEY, String(value));
    return true;
  } catch {
    return false;
  }
}
