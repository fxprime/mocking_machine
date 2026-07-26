function hexadecimal(value) {
  return value ? `0x${String(value).toUpperCase().padStart(4, "0")}` : undefined;
}

function isLinuxUsbSerialPort(port) {
  const names = [port.portName, port.displayName].filter(Boolean);
  return names.some(name =>
    /^(?:\/dev\/)?tty(?:USB|ACM)\d+$/i.test(name) ||
    String(name).startsWith("/dev/serial/by-id/")
  ) || Boolean(port.vendorId || port.productId);
}

export function selectableSerialPorts(portList, platform = process.platform) {
  if (platform !== "linux") return [...portList];
  return portList.filter(isLinuxUsbSerialPort);
}

export function serialPortLabel(port, index) {
  const name = port.displayName || port.portName || `Serial device ${index + 1}`;
  const identifiers = [
    port.vendorId && `VID ${hexadecimal(port.vendorId)}`,
    port.productId && `PID ${hexadecimal(port.productId)}`
  ].filter(Boolean);
  return identifiers.length ? `${name} (${identifiers.join(", ")})` : name;
}

export function selectedSerialPortId(portList, response) {
  if (response <= 0 || response > portList.length) return "";
  return portList[response - 1].portId;
}
