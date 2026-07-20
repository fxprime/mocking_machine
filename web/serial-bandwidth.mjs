export const MAXIMUM_CONFIGURED_STREAM_RATE_HZ = 500;
export const TELEMETRY_WIRE_BYTES = 84 + 12;
export const UART_BITS_PER_BYTE = 10;
export const MAXIMUM_STREAMING_UTILIZATION = 0.70;

export function maximumTelemetryStreamRateHz(baud) {
  if (!Number.isFinite(baud) || baud < 1) return 1;
  const calculated = Math.floor(
      baud * MAXIMUM_STREAMING_UTILIZATION / (TELEMETRY_WIRE_BYTES * UART_BITS_PER_BYTE));
  return Math.min(MAXIMUM_CONFIGURED_STREAM_RATE_HZ, Math.max(1, calculated));
}
