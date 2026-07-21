function csvCell(value) {
  const text = String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

function parseRows(text) {
  const rows = [];
  let row = [];
  let cell = "";
  let quoted = false;
  for (let index = 0; index < text.length; ++index) {
    const character = text[index];
    if (quoted) {
      if (character === '"' && text[index + 1] === '"') {
        cell += '"';
        ++index;
      } else if (character === '"') {
        quoted = false;
      } else {
        cell += character;
      }
    } else if (character === '"') {
      if (cell.length) throw new Error("Unexpected quote in CSV field.");
      quoted = true;
    } else if (character === ",") {
      row.push(cell);
      cell = "";
    } else if (character === "\n") {
      row.push(cell.replace(/\r$/, ""));
      if (row.some(value => value.trim() !== "")) rows.push(row);
      row = [];
      cell = "";
    } else {
      cell += character;
    }
  }
  if (quoted) throw new Error("Unterminated quoted CSV field.");
  row.push(cell.replace(/\r$/, ""));
  if (row.some(value => value.trim() !== "")) rows.push(row);
  return rows;
}

export function createParameterCsv(settings, definitions) {
  const rows = [["parameter", "value"]];
  Object.entries(definitions)
    .filter(([, definition]) => definition.id !== undefined)
    .sort((left, right) => left[1].id - right[1].id)
    .forEach(([parameter]) => rows.push([
      parameter,
      typeof settings[parameter] === "boolean" ? (settings[parameter] ? 1 : 0) : settings[parameter]
    ]));
  return `${rows.map(row => row.map(csvCell).join(",")).join("\r\n")}\r\n`;
}

export function parseParameterCsv(text, definitions) {
  const rows = parseRows(String(text).replace(/^\uFEFF/, ""));
  if (!rows.length || rows[0].length < 2 || rows[0][0].trim() !== "parameter" ||
      rows[0][1].trim() !== "value") {
    throw new Error('CSV header must be "parameter,value".');
  }
  const seen = new Set();
  return rows.slice(1).map((row, index) => {
    const line = index + 2;
    const parameter = (row[0] ?? "").trim();
    const definition = definitions[parameter];
    if (!definition || definition.id === undefined) {
      throw new Error(`Line ${line}: unknown or read-only parameter "${parameter}".`);
    }
    if (seen.has(parameter)) throw new Error(`Line ${line}: duplicate parameter "${parameter}".`);
    seen.add(parameter);
    const rawValue = (row[1] ?? "").trim();
    if (!rawValue) throw new Error(`Line ${line}: value for "${parameter}" is empty.`);
    const value = Number(rawValue);
    if (!Number.isFinite(value)) throw new Error(`Line ${line}: value for "${parameter}" is not finite.`);
    if (value < definition.min || value > definition.max) {
      throw new Error(`Line ${line}: "${parameter}" must be between ${definition.min} and ${definition.max}.`);
    }
    if (definition.decimals === 0 && !Number.isInteger(value)) {
      throw new Error(`Line ${line}: "${parameter}" must be an integer.`);
    }
    if (parameter === "direction" && value !== -1 && value !== 1) {
      throw new Error(`Line ${line}: "direction" must be -1 or 1.`);
    }
    return { parameter, id: definition.id, value };
  });
}
