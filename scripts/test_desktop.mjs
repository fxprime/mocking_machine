import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { getLaunchAtLoginState, setLaunchAtLoginState } from "../desktop/login-item.mjs";
import {
  selectableSerialPorts,
  selectedSerialPortId,
  serialPortLabel
} from "../desktop/serial-port.mjs";
import {
  performWindowAction,
  windowChromeState
} from "../desktop/window-chrome.mjs";
import {
  DesktopApiServer,
  normalizeApiConfiguration
} from "../desktop/api-server.mjs";

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

const linuxPorts = [
  { portId: "builtin", portName: "/dev/ttyS0" },
  { portId: "usb", portName: "/dev/ttyUSB0" },
  { portId: "acm", displayName: "ttyACM1" },
  { portId: "by-id", portName: "/dev/serial/by-id/usb-Espressif_ESP32" },
  { portId: "metadata", displayName: "CP2102", vendorId: "10c4" }
];
assert.deepEqual(
  selectableSerialPorts(linuxPorts, "linux").map(({ portId }) => portId),
  ["usb", "acm", "by-id", "metadata"]
);
assert.deepEqual(selectableSerialPorts(linuxPorts, "darwin"), linuxPorts);
assert.deepEqual(selectableSerialPorts(
  [{ portId: "builtin", portName: "/dev/ttyS1" }],
  "linux"
), []);

const windowCalls = [];
const fakeWindow = {
  maximized: false,
  isMaximized() { return this.maximized; },
  minimize() { windowCalls.push("minimize"); },
  maximize() { this.maximized = true; windowCalls.push("maximize"); },
  unmaximize() { this.maximized = false; windowCalls.push("unmaximize"); },
  close() { windowCalls.push("close"); }
};
assert.deepEqual(windowChromeState(fakeWindow, "linux"), {
  customTitleBar: true,
  maximized: false
});
assert.deepEqual(windowChromeState(fakeWindow, "darwin"), {
  customTitleBar: false,
  maximized: false
});
performWindowAction(fakeWindow, "minimize", "linux");
assert.equal(performWindowAction(fakeWindow, "toggle-maximize", "linux").maximized, true);
assert.equal(performWindowAction(fakeWindow, "toggle-maximize", "linux").maximized, false);
performWindowAction(fakeWindow, "close", "linux");
assert.deepEqual(windowCalls, ["minimize", "maximize", "unmaximize", "close"]);
assert.throws(() => performWindowAction(fakeWindow, "close", "darwin"), /Unsupported/);
assert.throws(() => performWindowAction(fakeWindow, "unknown", "linux"), /Unsupported/);

const normalizedApi = normalizeApiConfiguration({
  enabled: true,
  host: "public.example.com",
  port: 80,
  streamRateHz: 500,
  allowControl: true
}, () => "0123456789abcdef0123456789abcdef");
assert.deepEqual(normalizedApi, {
  enabled: true,
  host: "127.0.0.1",
  port: 8787,
  streamRateHz: 10,
  allowControl: true,
  token: "0123456789abcdef0123456789abcdef"
});

const apiCommands = [];
const apiServer = new DesktopApiServer({
  configuration: {
    enabled: true,
    host: "127.0.0.1",
    port: 0,
    streamRateHz: 10,
    allowControl: false,
    token: "0123456789abcdef0123456789abcdef"
  },
  dispatchCommand: async (command, argumentsValue) => {
    apiCommands.push({ command, arguments: argumentsValue });
    return { accepted: true };
  }
});
apiServer.updateSnapshot({
  connected: true,
  machine: { state: 0, faults: 0 },
  telemetry: { encoderCount: 9007199254740993n }
});
await apiServer.start();
const apiEndpoint = apiServer.state.endpoint;
assert.match(apiEndpoint, /^http:\/\/127\.0\.0\.1:\d+\/v1$/);
const healthResponse = await fetch(`${apiEndpoint}/health`);
assert.equal(healthResponse.status, 200);
assert.equal((await healthResponse.json()).machineConnected, true);
assert.equal((await fetch(`${apiEndpoint}/status`)).status, 401);
const authenticatedStatus = await fetch(`${apiEndpoint}/status`, {
  headers: { Authorization: "Bearer 0123456789abcdef0123456789abcdef" }
});
assert.equal(authenticatedStatus.status, 200);
assert.equal((await authenticatedStatus.json()).telemetry.encoderCount, "9007199254740993");
const disabledControl = await fetch(`${apiEndpoint}/commands/stop`, {
  method: "POST",
  headers: {
    Authorization: "Bearer 0123456789abcdef0123456789abcdef",
    "Content-Type": "application/json"
  },
  body: "{}"
});
assert.equal(disabledControl.status, 403);
await apiServer.reconfigure({ allowControl: true, port: 0 });
const controlEndpoint = apiServer.state.endpoint;
const controlResponse = await fetch(`${controlEndpoint}/commands/select-profile`, {
  method: "POST",
  headers: {
    Authorization: "Bearer 0123456789abcdef0123456789abcdef",
    "Content-Type": "application/json"
  },
  body: JSON.stringify({ profileId: 7 })
});
assert.equal(controlResponse.status, 200);
assert.deepEqual(apiCommands, [{ command: "select-profile", arguments: { profileId: 7 } }]);
await apiServer.stop();

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
fakeApp.openAtLogin = false;
assert.deepEqual(getLaunchAtLoginState(fakeApp, "win32"), { supported: true, enabled: false });
assert.deepEqual(setLaunchAtLoginState(fakeApp, true, "win32"), { supported: true, enabled: true });
assert.deepEqual(getLaunchAtLoginState(fakeApp, "linux"), { supported: false, enabled: false });
fakeApp.isPackaged = false;
assert.deepEqual(getLaunchAtLoginState(fakeApp, "darwin"), { supported: false, enabled: false });
assert.throws(() => setLaunchAtLoginState(fakeApp, true, "darwin"), /installed macOS and Windows applications/);

const mainSource = await readFile(new URL("../desktop/main.mjs", import.meta.url), "utf8");
const preloadSource = await readFile(new URL("../desktop/preload.cjs", import.meta.url), "utf8");
const appSource = await readFile(new URL("../web/app.js", import.meta.url), "utf8");
const indexSource = await readFile(new URL("../web/index.html", import.meta.url), "utf8");
const stylesSource = await readFile(new URL("../web/styles.css", import.meta.url), "utf8");
const afterPackSource = await readFile(new URL("./after-pack.cjs", import.meta.url), "utf8");
const macEntitlements = await readFile(new URL("../build/entitlements.mac.plist", import.meta.url), "utf8");
const packageJson = JSON.parse(await readFile(new URL("../package.json", import.meta.url), "utf8"));
const nvmVersion = (await readFile(new URL("../.nvmrc", import.meta.url), "utf8")).trim();
assert.match(mainSource, /nodeIntegration:\s*false/);
assert.match(mainSource, /import electronMain from "electron\/main";[\s\S]*const \{ app, BrowserWindow/);
assert.match(preloadSource, /require\("electron\/renderer"\)/);
assert.match(mainSource, /contextIsolation:\s*true/);
assert.match(mainSource, /sandbox:\s*true/);
assert.match(mainSource, /permission === "serial" && ownsTrustedConsole/);
assert.match(mainSource, /selectableSerialPorts\(portList\)/);
assert.match(mainSource, /\/dev\/ttyUSB\*/);
assert.match(mainSource, /frame:\s*!usesCustomTitleBar/);
assert.match(mainSource, /Menu\.setApplicationMenu\(null\)/);
assert.match(mainSource, /Connecting synchronizes settings and telemetry only\. It does not arm or start the motor\./);
assert.doesNotMatch(preloadSource, /require\(["'](?:node:)?(?:fs|child_process|path|os)/);
assert.match(preloadSource, /performWindowAction:\s*action => ipcRenderer\.invoke\("desktop:window-action", action\)/);
assert.match(preloadSource, /getApiConfiguration:\s*\(\) => ipcRenderer\.invoke\("desktop:get-api-configuration"\)/);
assert.match(preloadSource, /copyApiToken:\s*\(\) => ipcRenderer\.invoke\("desktop:copy-api-token"\)/,
  "The sandboxed renderer must use the desktop clipboard bridge");
assert.match(mainSource, /ipcMain\.handle\("desktop:copy-api-token"[\s\S]*clipboard\.writeText\(apiConfiguration\.token\)/,
  "Only the trusted console may copy the configured API token");
assert.match(appSource, /desktop\.copyApiToken[\s\S]*navigator\.clipboard\.writeText/,
  "The copy button must prefer Electron's clipboard and retain a browser fallback");
assert.match(preloadSource, /publishApiSnapshot:\s*snapshot => ipcRenderer\.send\("desktop:api-snapshot", snapshot\)/);
assert.match(indexSource, /id="windowTitleBar"[^>]*hidden/);
assert.match(indexSource, /data-window-action="minimize"[\s\S]*data-window-action="toggle-maximize"[\s\S]*data-window-action="close"/);
assert.match(stylesSource, /\.window-titlebar\s*\{[\s\S]*height:\s*44px[\s\S]*-webkit-app-region:\s*drag/);
assert.match(stylesSource, /\.window-controls\s*\{[\s\S]*-webkit-app-region:\s*no-drag/);
assert.match(stylesSource, /\.window-controls button\s*\{[\s\S]*width:\s*48px[\s\S]*min-height:\s*44px/);
assert.match(indexSource, /id="launchAtLogin"[^>]*type="checkbox"/);
assert.match(indexSource, /id="desktopApiPanel"[^>]*hidden/);
assert.match(indexSource, /id="desktopApiHost"[\s\S]*127\.0\.0\.1[\s\S]*0\.0\.0\.0/);
assert.match(indexSource, /id="desktopApiPort"[^>]*min="1024"[^>]*max="65535"/);
assert.match(indexSource, /id="desktopApiStreamRate"/);
assert.match(indexSource, /id="desktopApiAllowControl"/);
assert.match(appSource, /initializeDesktopIntegration[\s\S]*getLaunchAtLogin\(\)[\s\S]*setLaunchAtLogin\(requested\)/);
assert.match(appSource, /Opens the console only; the motor remains disarmed\./);
assert.equal(packageJson.build.afterPack, "scripts/after-pack.cjs");
assert.equal(nvmVersion, "22.12.0");
assert.equal(packageJson.scripts.prestart, "node scripts/check-node-version.mjs");
assert.equal(packageJson.scripts["dist:win"], "electron-builder --win --x64");
assert.equal(packageJson.scripts["dist:linux"], "electron-builder --linux --x64");
assert.equal(packageJson.build.win.signAndEditExecutable, false);
assert.deepEqual(packageJson.build.win.target.map(({ target }) => target), ["nsis", "portable", "zip"]);
assert.deepEqual(packageJson.build.linux.target.map(({ target }) => target), ["AppImage", "deb"]);
assert.notEqual(packageJson.build.nsis.artifactName, packageJson.build.portable.artifactName);
for (const artifactName of [
  packageJson.build.artifactName,
  packageJson.build.nsis.artifactName,
  packageJson.build.portable.artifactName,
  packageJson.build.linux.artifactName,
  packageJson.build.deb.artifactName
]) {
  assert.match(artifactName, /^\$\{name\}-/);
}
assert.match(afterPackSource, /signAsync\(\{[\s\S]*identity:\s*"-"[\s\S]*identityValidation:\s*false/);
assert.match(afterPackSource, /optionsForFile\(filePath\)[\s\S]*filePath\.endsWith\("\.app"\)[\s\S]*entitlements:\s*entitlementsPath/);
assert.match(afterPackSource, /codesign"[\s\S]*"--verify"[\s\S]*"--deep"[\s\S]*"--strict"/);
assert.match(macEntitlements, /com\.apple\.security\.cs\.disable-library-validation/);

console.log("Desktop shell checks passed.");
