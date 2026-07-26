const required = [22, 12, 0];
const current = process.versions.node.split(".").map(Number);
const supported = required.every((part, index) => {
  for (let position = 0; position <= index; ++position) {
    if (current[position] > required[position]) return true;
    if (current[position] < required[position]) return false;
  }
  return true;
});

if (!supported) {
  console.error(
    `Mocking Machine desktop requires Node.js 22.12.0 or newer; current version is ${process.versions.node}.\n` +
    "Run `nvm install`, `nvm use`, then `npm rebuild electron` before starting the app."
  );
  process.exit(1);
}
