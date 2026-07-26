import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { getLaunchAtLoginState, setLaunchAtLoginState } from "../desktop/login-item.mjs";
import { selectedSerialPortId, serialPortLabel } from "../desktop/serial-port.mjs";

const ports = [
  { portId: "one", displayName: "ESP32", vendorId: "10c4", productId: "ea60" },
  { portId: "two", portName: "/dev/cu.usbserial-2" }
];
assert.equal(serialPortLabel(ports[0], 0), "ESP32 (VID 0x10C4, PID 0xEA60)");
assert.equal(serialPortLabel(ports[1], 1), "/dev/cu.usbserial-2");
assert.equal(selectedSerialPortId(ports, 0), "");
assert.equal(selectedSerialPortId(ports, 1), "one");
assert.equal(selectedSerialPortId(ports, 2), "two");
assert.equal(selectedSerialPortId(ports, 3), "");

const calls = [];
const fakeApp = {
  isPackaged: true,
  openAtLogin: false,
  getLoginItemSettings() { return { openAtLogin: this.openAtLogin }; },
  setLoginItemSettings(settings) {
    this.openAtLogin = settings.openAtLogin;
    calls.push(settings);
  }
};
assert.deepEqual(getLaunchAtLoginState(fakeApp, "darwin"), { supported: true, enabled: false });
assert.deepEqual(setLaunchAtLoginState(fakeApp, true, "darwin"), { supported: true, enabled: true });
assert.deepEqual(calls, [{ openAtLogin: true }]);
fakeApp.isPackaged = false;
assert.deepEqual(getLaunchAtLoginState(fakeApp, "darwin"), { supported: false, enabled: false });
assert.throws(() => setLaunchAtLoginState(fakeApp, true, "darwin"), /installed macOS application/);

const mainSource = await readFile(new URL("../desktop/main.mjs", import.meta.url), "utf8");
const preloadSource = await readFile(new URL("../desktop/preload.cjs", import.meta.url), "utf8");
const appSource = await readFile(new URL("../web/app.js", import.meta.url), "utf8");
const indexSource = await readFile(new URL("../web/index.html", import.meta.url), "utf8");
const afterPackSource = await readFile(new URL("./after-pack.cjs", import.meta.url), "utf8");
const macEntitlements = await readFile(new URL("../build/entitlements.mac.plist", import.meta.url), "utf8");
const packageJson = JSON.parse(await readFile(new URL("../package.json", import.meta.url), "utf8"));
assert.match(mainSource, /nodeIntegration:\s*false/);
assert.match(mainSource, /contextIsolation:\s*true/);
assert.match(mainSource, /sandbox:\s*true/);
assert.match(mainSource, /permission === "serial" && ownsTrustedConsole/);
assert.match(mainSource, /Connecting synchronizes settings and telemetry only\. It does not arm or start the motor\./);
assert.doesNotMatch(preloadSource, /require\(["'](?:node:)?(?:fs|child_process|path|os)/);
assert.match(indexSource, /id="launchAtLogin"[^>]*type="checkbox"/);
assert.match(appSource, /initializeDesktopIntegration[\s\S]*getLaunchAtLogin\(\)[\s\S]*setLaunchAtLogin\(requested\)/);
assert.match(appSource, /Opens the console only; the motor remains disarmed\./);
assert.equal(packageJson.build.afterPack, "scripts/after-pack.cjs");
assert.match(afterPackSource, /signAsync\(\{[\s\S]*identity:\s*"-"[\s\S]*identityValidation:\s*false/);
assert.match(afterPackSource, /optionsForFile\(filePath\)[\s\S]*filePath\.endsWith\("\.app"\)[\s\S]*entitlements:\s*entitlementsPath/);
assert.match(afterPackSource, /codesign"[\s\S]*"--verify"[\s\S]*"--deep"[\s\S]*"--strict"/);
assert.match(macEntitlements, /com\.apple\.security\.cs\.disable-library-validation/);

console.log("Desktop shell checks passed.");
