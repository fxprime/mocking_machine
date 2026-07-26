export class Emitter {
  #listeners = new Map();

  on(event, listener) {
    if (typeof listener !== "function") throw new TypeError("listener must be a function");
    const listeners = this.#listeners.get(event) ?? new Set();
    listeners.add(listener);
    this.#listeners.set(event, listeners);
    return () => this.off(event, listener);
  }

  once(event, listener) {
    const unsubscribe = this.on(event, value => {
      unsubscribe();
      listener(value);
    });
    return unsubscribe;
  }

  off(event, listener) {
    const listeners = this.#listeners.get(event);
    listeners?.delete(listener);
    if (listeners?.size === 0) this.#listeners.delete(event);
  }

  emit(event, value) {
    for (const listener of [...(this.#listeners.get(event) ?? [])]) listener(value);
  }

  clear() {
    this.#listeners.clear();
  }
}
