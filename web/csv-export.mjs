function integer(value) {
  if (typeof value === "bigint") return value.toString();
  const number = Number(value);
  return Number.isFinite(number) ? Math.trunc(number).toString() : "";
}

function decimal(value, places) {
  const number = Number(value);
  if (!Number.isFinite(number)) return "";
  const rounded = Number(number.toFixed(places));
  return (Object.is(rounded, -0) ? 0 : rounded).toFixed(places);
}

const columns = [
  ["timestamp_us", sample => integer(sample.timestamp)],
  ["profile_id", sample => integer(sample.profile)],
  ["desired_velocity_rad_s", sample => decimal(sample.desired, 4)],
  ["measured_velocity_rad_s", sample => decimal(sample.measured, 4)],
  ["controller_output", sample => decimal(sample.output, 6)],
  ["controller_p_delta", sample => decimal(sample.pTerm, 6)],
  ["controller_i_delta", sample => decimal(sample.iTerm, 6)],
  ["controller_d_delta", sample => decimal(sample.dTerm, 6)],
  ["current_a", sample => decimal(sample.current, 4)],
  ["supply_voltage_v", sample => decimal(sample.supplyVoltage, 4)],
  ["rotor_position_deg", sample => decimal(sample.rotorPosition, 3)],
  ["last_zero_timestamp_us", sample => integer(sample.zeroTime)],
  ["encoder_count", sample => integer(sample.count)],
  ["last_zero_encoder_count", sample => integer(sample.zeroCount)],
  ["zero_index_sequence", sample => integer(sample.zeroSequence)],
  ["zero_index_rejected_count", sample => integer(sample.zeroRejected)],
  ["load_setting_id", sample => integer(sample.load)],
  ["state", sample => integer(sample.state)],
  ["faults", sample => integer(sample.faults)]
];

export function createTelemetryCsv(samples) {
  const rows = [columns.map(([name]) => name).join(",")];
  for (const sample of samples) {
    rows.push(columns.map(([, format]) => format(sample)).join(","));
  }
  return rows.join("\n");
}

export function createLoadConfigurationCsv(settingId, loads) {
  const rows = ["load_setting_id,slot_id,position_deg,strength"];
  for (const load of loads) {
    rows.push([
      integer(settingId), integer(load.slot), integer(load.position), decimal(load.strength, 0)
    ].join(","));
  }
  return rows.join("\n");
}

export function defaultExportBaseName(date = new Date()) {
  return `mocking-machine-run-${date.toISOString().replace(/[:.]/g, "-")}`;
}

export function sanitizeExportBaseName(value, fallback = defaultExportBaseName()) {
  const sanitized = String(value ?? "")
    .trim()
    .replace(/\.csv$/i, "")
    .replace(/[<>:"/\\|?*\u0000-\u001f]/g, "-")
    .replace(/\s+/g, " ")
    .replace(/[. ]+$/g, "")
    .slice(0, 100);
  return sanitized && sanitized !== "." && sanitized !== ".." ? sanitized : fallback;
}
