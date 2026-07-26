import electronMain from "electron/main";
import { randomBytes, randomUUID } from "node:crypto";
import { mkdir, readFile, rename, writeFile } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import {
  DEFAULT_API_CONFIGURATION,
  DesktopApiServer,
  normalizeApiConfiguration
} from "./api-server.mjs";
import { getLaunchAtLoginState, setLaunchAtLoginState } from "./login-item.mjs";
import {
  selectableSerialPorts,
  selectedSerialPortId,
  serialPortLabel
} from "./serial-port.mjs";
import { performWindowAction, windowChromeState } from "./window-chrome.mjs";

const { app, BrowserWindow, clipboard, dialog, ipcMain, Menu, session, shell } = electronMain;
const desktopDirectory = path.dirname(fileURLToPath(import.meta.url));
const consolePath = path.join(desktopDirectory, "..", "web", "index.html");
const consoleUrl = pathToFileURL(consolePath).href;
let mainWindow;
let serialAccessConfigured = false;
let desktopApi;
let apiConfiguration;
const pendingApiCommands = new Map();

function apiConfigurationPath() {
  return path.join(app.getPath("userData"), "desktop-api.json");
}

function createApiToken() {
  return randomBytes(24).toString("hex");
}

async function loadApiConfiguration() {
  try {
    const stored = JSON.parse(await readFile(apiConfigurationPath(), "utf8"));
    return normalizeApiConfiguration(stored, createApiToken);
  } catch (error) {
    if (error.code !== "ENOENT" && !(error instanceof SyntaxError)) throw error;
    return normalizeApiConfiguration(DEFAULT_API_CONFIGURATION, createApiToken);
  }
}

async function saveApiConfiguration(configuration) {
  const target = apiConfigurationPath();
  const temporary = `${target}.tmp`;
  await mkdir(path.dirname(target), { recursive: true });
  await writeFile(temporary, `${JSON.stringify(configuration, null, 2)}\n`, {
    encoding: "utf8",
    mode: 0o600
  });
  await rename(temporary, target);
}

function sendApiState(state = desktopApi?.state) {
  if (state && mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send("desktop:api-state", state);
  }
}

function dispatchApiCommand(command, argumentsValue) {
  if (!mainWindow || mainWindow.isDestroyed()) {
    return Promise.reject(new Error("Desktop console is unavailable."));
  }
  const requestId = randomUUID();
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      pendingApiCommands.delete(requestId);
      reject(new Error(`Desktop command "${command}" timed out.`));
    }, 5000);
    pendingApiCommands.set(requestId, { resolve, reject, timer });
    mainWindow.webContents.send("desktop:api-command", {
      requestId,
      command,
      arguments: argumentsValue
    });
  });
}

function ownsTrustedConsole(webContents) {
  return webContents === mainWindow?.webContents &&
    webContents.getURL().split(/[?#]/, 1)[0] === consoleUrl;
}

function configureSerialAccess() {
  if (serialAccessConfigured) return;
  serialAccessConfigured = true;
  const applicationSession = session.defaultSession;

  applicationSession.setPermissionCheckHandler((webContents, permission) =>
    permission === "serial" && ownsTrustedConsole(webContents));
  applicationSession.setPermissionRequestHandler((webContents, permission, callback) =>
    callback(permission === "serial" && ownsTrustedConsole(webContents)));
  applicationSession.setDevicePermissionHandler(details =>
    details.deviceType === "serial");

  applicationSession.on("select-serial-port", async (event, portList, webContents, callback) => {
    event.preventDefault();
    if (!ownsTrustedConsole(webContents)) return callback("");
    const selectablePorts = selectableSerialPorts(portList);
    if (!selectablePorts.length) {
      await dialog.showMessageBox(mainWindow, {
        type: "warning",
        message: process.platform === "linux"
          ? "No USB serial devices found"
          : "No serial devices found",
        detail: process.platform === "linux"
          ? "Connect the ESP32 and check for /dev/ttyUSB* or /dev/ttyACM*, then try again."
          : "Connect the ESP32 by USB, then try again.",
        buttons: ["OK"]
      });
      return callback("");
    }

    const result = await dialog.showMessageBox(mainWindow, {
      type: "question",
      title: "Connect Mocking Machine",
      message: "Choose the ESP32 serial port",
      detail: "Connecting synchronizes settings and telemetry only. It does not arm or start the motor.",
      buttons: ["Cancel", ...selectablePorts.map(serialPortLabel)],
      cancelId: 0,
      defaultId: selectablePorts.length === 1 ? 1 : 0,
      noLink: true
    });
    callback(selectedSerialPortId(selectablePorts, result.response));
  });
}

function installApplicationMenu() {
  if (process.platform === "linux") {
    Menu.setApplicationMenu(null);
    return;
  }

  const template = [
    {
      label: app.name,
      submenu: [
        { role: "about" },
        { type: "separator" },
        { role: "services" },
        { type: "separator" },
        { role: "hide" },
        { role: "hideOthers" },
        { role: "unhide" },
        { type: "separator" },
        { role: "quit" }
      ]
    },
    { role: "editMenu" },
    { role: "viewMenu" },
    { role: "windowMenu" }
  ];
  Menu.setApplicationMenu(Menu.buildFromTemplate(template));
}

function createWindow() {
  const usesCustomTitleBar = process.platform === "linux";
  mainWindow = new BrowserWindow({
    title: "Mocking Machine",
    width: 1440,
    height: 900,
    minWidth: 900,
    minHeight: 650,
    backgroundColor: "#030817",
    frame: !usesCustomTitleBar,
    autoHideMenuBar: usesCustomTitleBar,
    show: false,
    webPreferences: {
      preload: path.join(desktopDirectory, "preload.cjs"),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true,
      webSecurity: true,
      devTools: !app.isPackaged
    }
  });

  mainWindow.webContents.setWindowOpenHandler(({ url }) => {
    if (url.startsWith("https://")) shell.openExternal(url);
    return { action: "deny" };
  });
  mainWindow.webContents.on("will-navigate", (event, url) => {
    if (url.split(/[?#]/, 1)[0] !== consoleUrl) event.preventDefault();
  });
  mainWindow.once("ready-to-show", () => mainWindow.show());
  const sendMaximizedState = () => {
    if (!mainWindow?.isDestroyed()) {
      mainWindow.webContents.send(
        "desktop:window-maximized",
        mainWindow.isMaximized()
      );
    }
  };
  mainWindow.on("maximize", sendMaximizedState);
  mainWindow.on("unmaximize", sendMaximizedState);
  mainWindow.on("closed", () => {
    mainWindow = undefined;
  });
  mainWindow.loadFile(consolePath);
}

ipcMain.handle("desktop:get-launch-at-login", () => getLaunchAtLoginState(app));
ipcMain.handle("desktop:set-launch-at-login", (_event, enabled) =>
  setLaunchAtLoginState(app, enabled));
ipcMain.handle("desktop:get-window-chrome", event => {
  if (!ownsTrustedConsole(event.sender)) throw new Error("Untrusted window.");
  return windowChromeState(mainWindow);
});
ipcMain.handle("desktop:window-action", (event, action) => {
  if (!ownsTrustedConsole(event.sender)) throw new Error("Untrusted window.");
  return performWindowAction(mainWindow, action);
});
ipcMain.handle("desktop:get-api-configuration", event => {
  if (!ownsTrustedConsole(event.sender)) throw new Error("Untrusted window.");
  return { configuration: apiConfiguration, state: desktopApi.state };
});
ipcMain.handle("desktop:copy-api-token", event => {
  if (!ownsTrustedConsole(event.sender)) throw new Error("Untrusted window.");
  clipboard.writeText(apiConfiguration.token);
});
ipcMain.handle("desktop:set-api-configuration", async (event, value) => {
  if (!ownsTrustedConsole(event.sender)) throw new Error("Untrusted window.");
  apiConfiguration = normalizeApiConfiguration({
    ...value,
    token: apiConfiguration.token
  }, createApiToken);
  await saveApiConfiguration(apiConfiguration);
  const state = await desktopApi.reconfigure(apiConfiguration);
  return { configuration: apiConfiguration, state };
});
ipcMain.on("desktop:api-snapshot", (event, snapshot) => {
  if (ownsTrustedConsole(event.sender)) desktopApi.updateSnapshot(snapshot);
});
ipcMain.on("desktop:api-command-result", (event, result) => {
  if (!ownsTrustedConsole(event.sender)) return;
  const pending = pendingApiCommands.get(result?.requestId);
  if (!pending) return;
  clearTimeout(pending.timer);
  pendingApiCommands.delete(result.requestId);
  if (result.ok) pending.resolve(result.value);
  else pending.reject(new Error(result.error || "Desktop API command failed."));
});

app.whenReady().then(async () => {
  apiConfiguration = await loadApiConfiguration();
  desktopApi = new DesktopApiServer({
    configuration: apiConfiguration,
    dispatchCommand: dispatchApiCommand,
    onStateChange: sendApiState
  });
  if (apiConfiguration.enabled) {
    desktopApi.start().catch(error => {
      console.error("Could not start desktop API server.", error);
    });
  }
  configureSerialAccess();
  installApplicationMenu();
  createWindow();
  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on("before-quit", () => {
  for (const pending of pendingApiCommands.values()) {
    clearTimeout(pending.timer);
    pending.reject(new Error("Desktop application is quitting."));
  }
  pendingApiCommands.clear();
  desktopApi?.stop().catch(() => {});
});
app.on("window-all-closed", () => app.quit());
