/**
 * The fleet: the simulated swarm demo and the real robots, behind one command interface.
 * One instance per server process (kept on globalThis so Next.js dev reloads do not duplicate it).
 */
import { encodeCells } from "../sim/world";
import { BASE_ID, Swarm } from "../sim/swarm";
import { RealRobot } from "./real";
import type { FleetEvent, RobotView, Snapshot, WorldInfo } from "./types";

const TICK_MS = 100;
const SIM_DT = 0.1;
export const MAX_SPEED_UP = 20;
const DEMO_INCIDENT_RATE = 0.003; // per robot-second while moving (× terrain exposure): a hidden stump or hole

export type Action = "goal" | "manual" | "auto" | "teleop" | "stop";

export interface CommandRequest {
  targets: string[];
  action: Action;
  x?: number;
  y?: number;
  th?: number;
  v?: number;
  w?: number;
}

export class FleetManager {
  swarm: Swarm | null = null;
  paused = false;
  speed = 1;
  incidents = true;
  readonly real = new Map<string, RealRobot>();
  private events: FleetEvent[] = [];
  private realAttention = new Map<string, string | null>();
  private timer: ReturnType<typeof setInterval>;
  private realSeq = 0;

  constructor() {
    this.timer = setInterval(() => this.tick(), TICK_MS);
  }

  // ---------------------------------------------------------------- demo
  startDemo(robots = 10, seed = 7, incidents = true): void {
    this.swarm = new Swarm(seed);
    this.swarm.incidentRate = incidents ? DEMO_INCIDENT_RATE : 0;
    this.incidents = incidents;
    this.swarm.spawn(robots);
    this.paused = false;
    this.log("демо запущено: рой на базе, укажите цель или нажмите «Рой → к цели»", "info");
  }

  stopDemo(): void {
    this.swarm = null;
    this.log("демо остановлено", "info");
  }

  worldInfo(): WorldInfo | null {
    const w = this.swarm?.world;
    if (!w) return null;
    return { width: w.width, height: w.height, res: w.res, base: w.base, start: w.start, destination: w.destination, cells: encodeCells(w) };
  }

  fog(): string | null {
    const k = this.swarm?.baseKnowledge;
    return k ? Buffer.from(k.cells.buffer, k.cells.byteOffset, k.cells.byteLength).toString("base64") : null;
  }

  private tick(): void {
    if (this.swarm && !this.paused) {
      const steps = Math.max(1, Math.min(MAX_SPEED_UP, Math.round(this.speed)));
      for (let k = 0; k < steps; k++) this.swarm.step(SIM_DT);
    }
    for (const r of this.real.values()) {
      const a = r.view().attention;
      if (a !== (this.realAttention.get(r.id) ?? null)) {
        if (a) this.log(`${r.name}: ${a}`, "alert", r.id);
        this.realAttention.set(r.id, a);
      }
    }
  }

  // ---------------------------------------------------------------- real robots
  addReal(name: string, url: string): RealRobot {
    const clean = url.replace(/\/+$/, "");
    if (!/^https?:\/\/[\w.\-:[\]]+$/.test(clean)) throw new Error("адрес вида http://192.168.1.58:8090");
    const id = `real-${++this.realSeq}`;
    const r = new RealRobot(id, name || clean, clean);
    this.real.set(id, r);
    this.log(`добавлен робот ${r.name} (${clean})`, "info");
    return r;
  }

  removeReal(id: string): void {
    this.real.get(id)?.dispose();
    this.real.delete(id);
  }

  // ---------------------------------------------------------------- commands
  async command(req: CommandRequest): Promise<{ ok: boolean; message: string }> {
    const sim = req.targets.filter((id) => id.startsWith("sim-"));
    const real = req.targets.map((id) => this.real.get(id)).filter((r): r is RealRobot => !!r);
    const msgs: string[] = [];
    let ok = true;
    if (sim.length) {
      const sw = this.swarm;
      if (!sw) return { ok: false, message: "демо не запущено" };
      if (req.action === "goal") {
        if (req.x === undefined || req.y === undefined) return { ok: false, message: "нет точки цели" };
        sw.goalForGroup(sim, { x: req.x, y: req.y });
      } else {
        for (const id of sim) {
          if (req.action === "manual") sw.command(id, { kind: "manual" });
          else if (req.action === "auto") sw.command(id, { kind: "auto" });
          else if (req.action === "stop") sw.command(id, { kind: "goal", goal: null });
          else if (req.action === "teleop") sw.command(id, { kind: "teleop", v: req.v ?? 0, w: req.w ?? 0 });
        }
      }
      msgs.push(`${sim.length} сим.`);
    }
    for (const r of real) {
      let res: { ok: boolean; message: string };
      if (req.action === "goal") res = await r.goal(req.x ?? 0, req.y ?? 0, req.th ?? 0);
      else if (req.action === "teleop") res = await r.teleop(req.v ?? 0, req.w ?? 0);
      else if (req.action === "stop" || req.action === "manual") res = await r.cancel();
      else res = { ok: true, message: "ok" };
      ok &&= res.ok;
      msgs.push(`${r.name}: ${res.message}`);
    }
    if (req.action !== "teleop" && req.action !== "manual" && req.targets.length) {
      this.log(`оператор: ${ACTION_TEXT[req.action]} → ${req.targets.length} робот(ов)`, "info");
    }
    return { ok, message: msgs.join("; ") || "нет роботов" };
  }

  /** STOP ALL: every simulated robot drops its goal; every real robot gets its emergency stop. */
  async stopAll(): Promise<void> {
    if (this.swarm) for (const r of this.swarm.robots) this.swarm.command(r.id, { kind: "goal", goal: null });
    await Promise.all([...this.real.values()].map((r) => r.estop(true)));
    this.log("СТОП ВСЕМ", "alert");
  }

  // ---------------------------------------------------------------- snapshot
  snapshot(truth: boolean): Snapshot {
    const robots: RobotView[] = [];
    const sw = this.swarm;
    if (sw) {
      const pending = sw.pendingCommands();
      for (const o of sw.operatorView()) {
        const r = sw.robots.find((x) => x.id === o.id)!;
        const b = o.beacon;
        robots.push({
          id: o.id, name: o.name, kind: "sim", role: sw.role(o.id), known: !!b,
          x: b?.x ?? r.x, y: b?.y ?? r.y, th: b?.th ?? r.th, v: b?.v ?? 0,
          mode: b?.mode ?? "idle", arrived: b?.arrived ?? false, battery: b?.battery ?? null,
          goal: b?.goal ?? null, path: b?.path ?? [], linkAge: o.age, attention: o.attention,
          pending: pending[o.id] ?? null,
          truth: truth ? { x: r.x, y: r.y, th: r.th } : undefined,
        });
      }
    }
    for (const r of this.real.values()) robots.push(r.view());
    const nodes = sw?.nodes() ?? [];
    return {
      now: Date.now(),
      demo: sw ? {
        running: true, paused: this.paused, speed: this.speed, t: sw.t, robots: sw.robots.length,
        arrived: sw.robots.filter((r) => r.arrived).length, incidents: this.incidents,
        destination: sw.world.destination, base: sw.world.base,
        links: sw.mesh.links(nodes).map(([a, b]) => [a, b] as [string, string]),
        mesh: { ...sw.mesh.stats }, knownCells: sw.baseKnowledge.knownCount(),
      } : null,
      robots,
      events: [...(sw?.events ?? []).map((e) => ({ ...e })), ...this.events].sort((a, b) => a.t - b.t).slice(-40),
    };
  }

  private log(text: string, level: FleetEvent["level"], robot?: string): void {
    this.events.push({ t: this.swarm?.t ?? 0, text, level, robot });
    if (this.events.length > 100) this.events.shift();
  }

  dispose(): void {
    clearInterval(this.timer);
    for (const r of this.real.values()) r.dispose();
  }
}

const ACTION_TEXT: Record<Action, string> = {
  goal: "цель", manual: "ручное управление", auto: "авторежим", teleop: "джойстик", stop: "стоп",
};

// when this module was evaluated: after a dev hot reload the newer module replaces the stale instance
// (which would keep running the old simulation code); an older module still alive in a long-lived
// request (the SSE stream) must reuse the newer instance, not replace it back
const LOADED_AT = performance.timeOrigin + performance.now();

const g = globalThis as unknown as { __ugvFleet?: FleetManager & { loadedAt?: number } };
export function fleet(): FleetManager {
  const cur = g.__ugvFleet;
  if (!cur || (cur.loadedAt ?? 0) < LOADED_AT) {
    cur?.dispose();
    const f = new FleetManager() as FleetManager & { loadedAt?: number };
    f.loadedAt = LOADED_AT;
    g.__ugvFleet = f;
  }
  return g.__ugvFleet!;
}

export { BASE_ID };
