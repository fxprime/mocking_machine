const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("mockingMachineDesktop", Object.freeze({
  getLaunchAtLogin: () => ipcRenderer.invoke("desktop:get-launch-at-login"),
  setLaunchAtLogin: enabled => ipcRenderer.invoke("desktop:set-launch-at-login", Boolean(enabled)),
  getWindowChrome: () => ipcRenderer.invoke("desktop:get-window-chrome"),
  performWindowAction: action => ipcRenderer.invoke("desktop:window-action", action),
  onWindowMaximized: callback => {
    ipcRenderer.on("desktop:window-maximized", (_event, maximized) =>
      callback(Boolean(maximized)));
  }
}));
