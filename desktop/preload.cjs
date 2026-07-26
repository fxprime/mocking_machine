const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("mockingMachineDesktop", Object.freeze({
  getLaunchAtLogin: () => ipcRenderer.invoke("desktop:get-launch-at-login"),
  setLaunchAtLogin: enabled => ipcRenderer.invoke("desktop:set-launch-at-login", Boolean(enabled))
}));
