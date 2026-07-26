import { Emitter } from "./emitter.mjs";
import { toUint8Array } from "./codec.mjs";

export class StreamTransport extends Emitter {
  #readable;
  #writable;
  #reader;
  #writer;
  #reading = false;

  constructor({ readable, writable }) {
    super();
    if (!readable || !writable) throw new TypeError("readable and writable streams are required");
    this.#readable = readable;
    this.#writable = writable;
  }

  async open() {
    if (this.#reading) return;
    this.#reader = this.#readable.getReader();
    this.#writer = this.#writable.getWriter();
    this.#reading = true;
    this.#readLoop();
  }

  async write(bytes) {
    if (!this.#writer) throw new Error("transport is not open");
    await this.#writer.write(toUint8Array(bytes));
  }

  async close() {
    this.#reading = false;
    try { await this.#reader?.cancel(); } catch {}
    try { this.#reader?.releaseLock(); } catch {}
    try { this.#writer?.releaseLock(); } catch {}
    this.#reader = undefined;
    this.#writer = undefined;
  }

  async #readLoop() {
    try {
      while (this.#reading) {
        const { value, done } = await this.#reader.read();
        if (done) break;
        if (value) this.emit("data", toUint8Array(value));
      }
    } catch (error) {
      if (this.#reading) this.emit("error", error);
    } finally {
      this.#reading = false;
      this.emit("close");
    }
  }
}

export class WebSerialTransport extends Emitter {
  #port;
  #stream;
  #baudRate;
  #bufferSize;
  #filters;

  constructor({ port, baudRate = 115200, bufferSize = 4096, filters = [] } = {}) {
    super();
    this.#port = port;
    this.#baudRate = baudRate;
    this.#bufferSize = bufferSize;
    this.#filters = filters;
  }

  get port() {
    return this.#port;
  }

  async open() {
    if (!this.#port) {
      if (!globalThis.navigator?.serial) {
        throw new Error("Web Serial is unavailable; use a supported browser on HTTPS or localhost");
      }
      this.#port = await globalThis.navigator.serial.requestPort({ filters: this.#filters });
    }
    await this.#port.open({ baudRate: this.#baudRate, bufferSize: this.#bufferSize });
    this.#stream = new StreamTransport({
      readable: this.#port.readable,
      writable: this.#port.writable
    });
    this.#stream.on("data", bytes => this.emit("data", bytes));
    this.#stream.on("error", error => this.emit("error", error));
    this.#stream.on("close", () => this.emit("close"));
    await this.#stream.open();
  }

  write(bytes) {
    return this.#stream?.write(bytes) ?? Promise.reject(new Error("transport is not open"));
  }

  async close() {
    await this.#stream?.close();
    this.#stream = undefined;
    await this.#port?.close();
  }
}

// Adapts callback/event based serial libraries without making this package depend on one.
export class CallbackTransport extends Emitter {
  #write;
  #open;
  #close;
  #subscribe;
  #unsubscribe;

  constructor({ write, subscribe, open, close }) {
    super();
    if (typeof write !== "function" || typeof subscribe !== "function") {
      throw new TypeError("write and subscribe functions are required");
    }
    this.#write = write;
    this.#subscribe = subscribe;
    this.#open = open;
    this.#close = close;
  }

  async open() {
    this.#unsubscribe = this.#subscribe(
      bytes => this.emit("data", toUint8Array(bytes)),
      error => this.emit("error", error)
    );
    try {
      await this.#open?.();
    } catch (error) {
      await this.#unsubscribe?.();
      this.#unsubscribe = undefined;
      throw error;
    }
  }

  async write(bytes) {
    await this.#write(toUint8Array(bytes));
  }

  async close() {
    await this.#unsubscribe?.();
    this.#unsubscribe = undefined;
    await this.#close?.();
  }
}
