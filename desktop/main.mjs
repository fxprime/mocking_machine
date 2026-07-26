import { app, BrowserWindow, dialog, ipcMain, Menu, session, shell } from "electron";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import { getLaunchAtLoginState, setLaunchAtLoginState } from "./login-item.mjs";
import {
  selectableSerialPorts,
  selectedSerialPortId,
  serialPortLabel
} from "./serial-port.mjs";
import { performWindowAction, windowChromeState } from "./window-chrome.mjs";

const desktopDirectory = path.dirname(fileURLToPath(import.meta.url));
const consolePath = path.join(desktopDirectory, "..", "web", "index.html");
const consoleUrl = pathToFileURL(consolePath).href;
let mainWindow;
let serialAccessConfigured = false;

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

app.whenReady().then(() => {
  configureSerialAccess();
  installApplicationMenu();
  createWindow();
  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on("window-all-closed", () => app.quit());
