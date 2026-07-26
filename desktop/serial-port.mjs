function hexadecimal(value) {
  return value ? `0x${String(value).toUpperCase().padStart(4, "0")}` : undefined;
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
