const { contextBridge, ipcRenderer } = require("electron/renderer");

contextBridge.exposeInMainWorld("mockingMachineDesktop", Object.freeze({
  getLaunchAtLogin: () => ipcRenderer.invoke("desktop:get-launch-at-login"),
  setLaunchAtLogin: enabled => ipcRenderer.invoke("desktop:set-launch-at-login", Boolean(enabled)),
  getWindowChrome: () => ipcRenderer.invoke("desktop:get-window-chrome"),
  performWindowAction: action => ipcRenderer.invoke("desktop:window-action", action),
  getApiConfiguration: () => ipcRenderer.invoke("desktop:get-api-configuration"),
  setApiConfiguration: configuration =>
    ipcRenderer.invoke("desktop:set-api-configuration", configuration),
  publishApiSnapshot: snapshot => ipcRenderer.send("desktop:api-snapshot", snapshot),
  onApiState: callback => {
    ipcRenderer.on("desktop:api-state", (_event, state) => callback(state));
  },
  onApiCommand: callback => {
    ipcRenderer.on("desktop:api-command", async (_event, request) => {
      try {
        const value = await callback(request.command, request.arguments);
        ipcRenderer.send("desktop:api-command-result", {
          requestId: request.requestId,
          ok: true,
          value
        });
      } catch (error) {
        ipcRenderer.send("desktop:api-command-result", {
          requestId: request.requestId,
          ok: false,
          error: error?.message ?? String(error)
        });
      }
    });
  },
  onWindowMaximized: callback => {
    ipcRenderer.on("desktop:window-maximized", (_event, maximized) =>
      callback(Boolean(maximized)));
  }
}));
