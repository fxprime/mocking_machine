import { createServer } from "node:http";
import { timingSafeEqual } from "node:crypto";

export const DEFAULT_API_CONFIGURATION = Object.freeze({
  enabled: false,
  host: "127.0.0.1",
  port: 8787,
  streamRateHz: 10,
  allowControl: false,
  token: ""
});

const allowedHosts = new Set(["127.0.0.1", "0.0.0.0"]);
const maximumRequestBytes = 16 * 1024;

export function normalizeApiConfiguration(value = {}, tokenFactory = () => "") {
  const port = Number(value.port);
  const streamRateHz = Number(value.streamRateHz);
  const token = typeof value.token === "string" && value.token.length >= 24
    ? value.token
    : tokenFactory();
  return {
    enabled: value.enabled === true,
    host: allowedHosts.has(value.host) ? value.host : DEFAULT_API_CONFIGURATION.host,
    port: Number.isInteger(port) && port >= 1024 && port <= 65535
      ? port
      : DEFAULT_API_CONFIGURATION.port,
    streamRateHz: Number.isInteger(streamRateHz) && streamRateHz >= 1 && streamRateHz <= 50
      ? streamRateHz
      : DEFAULT_API_CONFIGURATION.streamRateHz,
    allowControl: value.allowControl === true,
    token
  };
}

function jsonSafe(value) {
  if (typeof value === "bigint") return value.toString();
  if (Array.isArray(value)) return value.map(jsonSafe);
  if (value && typeof value === "object") {
    return Object.fromEntries(Object.entries(value).map(([key, item]) => [key, jsonSafe(item)]));
  }
  return value;
}

function sendJson(response, status, body) {
  const encoded = JSON.stringify(jsonSafe(body));
  response.writeHead(status, {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Headers": "Authorization, Content-Type, X-API-Key",
    "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
    "Cache-Control": "no-store",
    "Content-Type": "application/json; charset=utf-8",
    "Content-Length": Buffer.byteLength(encoded)
  });
  response.end(encoded);
}

function suppliedToken(request, url) {
  const authorization = request.headers.authorization;
  if (authorization?.startsWith("Bearer ")) return authorization.slice(7);
  if (typeof request.headers["x-api-key"] === "string") return request.headers["x-api-key"];
  return url.searchParams.get("token") ?? "";
}

function tokensMatch(expected, supplied) {
  const expectedBytes = Buffer.from(expected);
  const suppliedBytes = Buffer.from(supplied);
  return expectedBytes.length > 0 &&
    expectedBytes.length === suppliedBytes.length &&
    timingSafeEqual(expectedBytes, suppliedBytes);
}

async function readJson(request) {
  const chunks = [];
  let size = 0;
  for await (const chunk of request) {
    size += chunk.byteLength;
    if (size > maximumRequestBytes) throw new RangeError("Request body exceeds 16 KiB");
    chunks.push(chunk);
  }
  if (size === 0) return {};
  return JSON.parse(Buffer.concat(chunks).toString("utf8"));
}

export class DesktopApiServer {
  #configuration;
  #dispatchCommand;
  #server;
  #snapshot = {
    connected: false,
    machine: null,
    settings: null,
    telemetry: null,
    updatedAt: null
  };
  #streamClients = new Set();
  #streamTimer;
  #stateListener;
  #lastError = "";

  constructor({ configuration, dispatchCommand, onStateChange } = {}) {
    this.#configuration = { ...DEFAULT_API_CONFIGURATION, ...configuration };
    this.#dispatchCommand = dispatchCommand ?? (() => Promise.reject(new Error("No command bridge")));
    this.#stateListener = onStateChange;
  }

  get configuration() {
    return { ...this.#configuration };
  }

  get state() {
    const address = this.#server?.address();
    const port = typeof address === "object" && address ? address.port : this.#configuration.port;
    return {
      enabled: this.#configuration.enabled,
      running: Boolean(this.#server?.listening),
      host: this.#configuration.host,
      port,
      streamRateHz: this.#configuration.streamRateHz,
      allowControl: this.#configuration.allowControl,
      endpoint: this.#server?.listening
        ? `http://${this.#configuration.host}:${port}/v1`
        : "",
      lastError: this.#lastError
    };
  }

  updateSnapshot(snapshot) {
    this.#snapshot = {
      ...this.#snapshot,
      ...jsonSafe(snapshot),
      updatedAt: new Date().toISOString()
    };
  }

  async reconfigure(configuration) {
    await this.stop();
    this.#configuration = { ...this.#configuration, ...configuration };
    if (this.#configuration.enabled) await this.start();
    else this.#notifyState();
    return this.state;
  }

  async start() {
    if (this.#server?.listening || !this.#configuration.enabled) return this.state;
    this.#lastError = "";
    this.#server = createServer((request, response) => {
      this.#handleRequest(request, response).catch(error => {
        const status = error instanceof RangeError || error instanceof SyntaxError ? 400 : 500;
        if (!response.headersSent) sendJson(response, status, { error: error.message });
        else response.destroy(error);
      });
    });
    this.#server.on("error", error => {
      this.#lastError = error.message;
      this.#notifyState();
    });
    try {
      await new Promise((resolve, reject) => {
        const onError = error => {
          this.#server.off("listening", onListening);
          reject(error);
        };
        const onListening = () => {
          this.#server.off("error", onError);
          resolve();
        };
        this.#server.once("error", onError);
        this.#server.once("listening", onListening);
        this.#server.listen(this.#configuration.port, this.#configuration.host);
      });
      this.#streamTimer = setInterval(
        () => this.#broadcastSnapshot(),
        Math.round(1000 / this.#configuration.streamRateHz)
      );
      this.#streamTimer.unref?.();
      this.#notifyState();
      return this.state;
    } catch (error) {
      this.#lastError = error.message;
      this.#server = undefined;
      this.#notifyState();
      throw error;
    }
  }

  async stop() {
    clearInterval(this.#streamTimer);
    this.#streamTimer = undefined;
    for (const response of this.#streamClients) response.end();
    this.#streamClients.clear();
    const server = this.#server;
    this.#server = undefined;
    if (server) {
      await new Promise(resolve => server.close(() => resolve()));
    }
    this.#notifyState();
  }

  #notifyState() {
    this.#stateListener?.(this.state);
  }

  #authorized(request, url) {
    return tokensMatch(this.#configuration.token, suppliedToken(request, url));
  }

  #broadcastSnapshot() {
    if (this.#streamClients.size === 0) return;
    const event = `event: snapshot\ndata: ${JSON.stringify(this.#snapshot)}\n\n`;
    for (const response of [...this.#streamClients]) {
      if (response.destroyed) this.#streamClients.delete(response);
      else response.write(event);
    }
  }

  async #handleRequest(request, response) {
    const url = new URL(request.url ?? "/", "http://localhost");
    if (request.method === "OPTIONS") {
      response.writeHead(204, {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Headers": "Authorization, Content-Type, X-API-Key",
        "Access-Control-Allow-Methods": "GET, POST, OPTIONS"
      });
      response.end();
      return;
    }
    if (request.method === "GET" && url.pathname === "/v1/health") {
      sendJson(response, 200, {
        ok: true,
        service: "mocking-machine-desktop-api",
        apiVersion: 1,
        machineConnected: this.#snapshot.connected === true
      });
      return;
    }
    if (!this.#authorized(request, url)) {
      sendJson(response, 401, { error: "A valid bearer token or X-API-Key is required." });
      return;
    }
    if (request.method === "GET" && url.pathname === "/v1/status") {
      sendJson(response, 200, this.#snapshot);
      return;
    }
    if (request.method === "GET" && url.pathname === "/v1/settings") {
      sendJson(response, 200, { settings: this.#snapshot.settings });
      return;
    }
    if (request.method === "GET" && url.pathname === "/v1/telemetry") {
      sendJson(response, 200, { telemetry: this.#snapshot.telemetry });
      return;
    }
    if (request.method === "GET" && url.pathname === "/v1/stream") {
      response.writeHead(200, {
        "Access-Control-Allow-Origin": "*",
        "Cache-Control": "no-cache, no-transform",
        "Connection": "keep-alive",
        "Content-Type": "text/event-stream; charset=utf-8"
      });
      response.write(`event: snapshot\ndata: ${JSON.stringify(this.#snapshot)}\n\n`);
      this.#streamClients.add(response);
      request.on("close", () => this.#streamClients.delete(response));
      return;
    }
    if (request.method === "POST" && url.pathname.startsWith("/v1/commands/")) {
      if (!this.#configuration.allowControl) {
        sendJson(response, 403, {
          error: "Remote control is disabled in the desktop Parameters page."
        });
        return;
      }
      const command = decodeURIComponent(url.pathname.slice("/v1/commands/".length));
      const argumentsValue = await readJson(request);
      try {
        const result = await this.#dispatchCommand(command, argumentsValue);
        sendJson(response, 200, { ok: true, command, result });
      } catch (error) {
        sendJson(response, 409, { ok: false, command, error: error.message });
      }
      return;
    }
    sendJson(response, 404, { error: "Unknown API endpoint." });
  }
}
