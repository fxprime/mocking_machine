function decimalPlaces(value) {
  const text = String(value).toLowerCase();
  const [coefficient, exponentText] = text.split("e");
  const fractionLength = coefficient.includes(".")
    ? coefficient.length - coefficient.indexOf(".") - 1
    : 0;
  const exponent = exponentText === undefined ? 0 : Number(exponentText);
  return Math.max(0, fractionLength - exponent);
}

export function snapNumberToStep(value, step, minimum = -Infinity,
                                 maximum = Infinity) {
  value = Number(value);
  step = Number(step);
  minimum = Number(minimum);
  maximum = Number(maximum);
  if (!Number.isFinite(value)) return Number.NaN;

  const lower = Number.isFinite(minimum) ? minimum : -Infinity;
  const upper = Number.isFinite(maximum) ? maximum : Infinity;
  const bounded = Math.min(upper, Math.max(lower, value));
  if (!Number.isFinite(step) || step <= 0) return bounded;

  const snapped = Math.round(bounded / step) * step;
  const clamped = Math.min(upper, Math.max(lower, snapped));
  const precision = Math.min(12, Math.max(
      decimalPlaces(step),
      Number.isFinite(lower) ? decimalPlaces(lower) : 0,
      Number.isFinite(upper) ? decimalPlaces(upper) : 0));
  return Number(clamped.toFixed(precision));
}

export function enableFlexibleNumberInput(input, supportedStep) {
  if (!input) return;
  if (supportedStep === undefined) supportedStep = input.step;
  const step = Number(supportedStep);
  if (!Number.isFinite(step) || step <= 0) return;
  input.dataset.supportedStep = String(step);
  input.step = "any";
}

export function commitFlexibleNumberInput(input) {
  const step = Number(input?.dataset?.supportedStep);
  if (!input || String(input.value).trim() === "") return Number.NaN;
  const value = Number(input?.value);
  if (!Number.isFinite(step) || step <= 0 ||
      !Number.isFinite(value)) {
    return Number.NaN;
  }
  const minimum = input.min === "" ? -Infinity : Number(input.min);
  const maximum = input.max === "" ? Infinity : Number(input.max);
  const snapped = snapNumberToStep(value, step, minimum, maximum);
  input.value = String(snapped);
  return snapped;
}
