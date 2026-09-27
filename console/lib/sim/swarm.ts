/**
 * The swarm demo: robots, terrain, mesh and the operator base.
 *
 * Every 0.5 s each robot broadcasts a beacon (pose, state, help request, last command it applied) and
 * the terrain it learnt since the previous broadcast. Neighbours merge that terrain into their own
 * knowledge (and replan); the base merges it into the operator's picture. Operator commands leave the
 * base as mesh packets and are re-sent until the robot's beacon acknowledges them.
 */
import { Knowledge } from "./knowledge";
import { Mesh, MeshNode, Packet } from "./mesh";
import { HelpReason, Mode, Point, SimRobot } from "./robot";
import { Cell, World, cellAt, generateWorld, idx, passable, rng } from "./world";

export const BASE_ID = "base";
const MESH_PERIOD = 0.5; // s
const COMMAND_RETRY = 2.0; // s
const RELAY_MARGIN = 6; // m of radio range kept in reserve when a relay anchors
export const STALE_AFTER = 15; // s without a beacon at the base → "no link"

export interface Beacon {
  id: string;
  t: number;
  x: number;
  y: number;
  th: number;
  v: number;
  mode: Mode;
  help: HelpReason;
  arrived: boolean;
  battery: number;
  goal: Point | null;
  ack: number; // last command sequence number applied
  path: Point[];
}

type Command =
  | { seq: number; kind: "goal"; goal: Point | null; area?: { x: number; y: number; r: number } }
  | { seq: number; kind: "manual" }
  | { seq: number; kind: "auto" }
  | { seq: number; kind: "teleop"; v: number; w: number };

/** Omit that distributes over a union (plain Omit collapses the variants). */
type DistOmit<T, K extends PropertyKey> = T extends unknown ? Omit<T, K> : never;
export type CommandInput = DistOmit<Command, "seq">;

interface Pending {
  cmd: Command;
  sentT: number;
}

export interface SwarmEvent {
  t: number;
  robot?: string;
  text: string;
  level: "info" | "warn" | "alert";
}

export class Swarm {
  readonly world: World;
  readonly robots: SimRobot[] = [];
  readonly mesh: Mesh;
  readonly baseKnowledge: Knowledge;
  readonly baseView = new Map<string, Beacon>();
  readonly events: SwarmEvent[] = [];
  t = 0;
  private rand: () => number;
  private meshT = 0;
  private cmdSeq = 0;
  private pending = new Map<string, Pending>(); // robot → latest unacknowledged command
  private applied = new Map<string, number>(); // robot → last applied seq (robot side)
  private lastHelp = new Map<string, HelpReason>();
  private roles = new Map<string, "relay" | "unit">();
  private relayOrder: string[] = [];
  private anchored = new Map<string, Point>();
  private lastGood = new Map<string, Point>();

  /** per-robot probability per second of a demo incident while crossing rough ground / undergrowth */
  incidentRate = 0;

  constructor(seed = 7, world?: World) {
    this.world = world ?? generateWorld(seed);
    this.mesh = new Mesh(this.world, undefined, seed + 1);
    this.baseKnowledge = new Knowledge(this.world.width, this.world.height);
    this.rand = rng(seed + 2);
  }

  addRobot(name: string, x: number, y: number, th = 0): SimRobot {
    const id = `sim-${this.robots.length + 1}`;
    const r = new SimRobot(id, name, this.robots.length, this.world, x, y, th);
    this.robots.push(r);
    this.applied.set(id, -1);
    return r;
  }

  /** Spawn n robots in a grid at the start area. */
  spawn(n: number): void {
    const { start } = this.world;
    const cols = Math.ceil(Math.sqrt(n));
    for (let k = 0; k < n; k++) {
      const row = Math.floor(k / cols), col = k % cols;
      this.addRobot(`UGV-${String(k + 1).padStart(2, "0")}`, start.x - 2 + row * 1.6, start.y + (col - (cols - 1) / 2) * 1.6);
    }
    this.log(`рой: ${n} роботов на старте`, "info");
  }

  nodes(): MeshNode[] {
    return [{ id: BASE_ID, x: this.world.base.x, y: this.world.base.y }, ...this.robots.map((r) => ({ id: r.id, x: r.x, y: r.y }))];
  }

  // ---------------------------------------------------------------- operator commands (via the mesh)
  command(robotId: string, cmd: CommandInput): void {
    const full = { ...cmd, seq: this.cmdSeq++ } as Command;
    if (full.kind === "teleop") {
      // teleop is continuous: fire and forget, no retries
      this.mesh.send(BASE_ID, "command", full, robotId);
      return;
    }
    this.pending.set(robotId, { cmd: full, sentT: this.t });
    this.mesh.send(BASE_ID, "command", full, robotId);
  }

  /**
   * Goal for a group. If the goal is beyond the base's radio reach, some robots become relays and
   * stop along the base → goal line so the mesh chain back to the operator stays connected; the rest
   * take hex slots around the goal so they do not crowd one spot.
   */
  goalForGroup(ids: string[], p: Point): void {
    // relays ("breadcrumbs"): as many as the straight-line estimate needs; they travel with the swarm
    // and anchor themselves where the link to the previous chain link gets weak (hills included)
    const need = relayPoints(this.world.base, p, this.mesh.linkRange({ id: BASE_ID, x: 0, y: 0 }, { id: "r", x: 0, y: 0 }),
      this.mesh.cfg.range, ids.length).length;
    const byDistToBase = [...ids].sort((a, b) => this.dist(a, this.world.base) - this.dist(b, this.world.base));
    this.relayOrder = byDistToBase.slice(0, need);
    this.anchored.clear();
    this.lastGood.clear();
    const slots = hexSlots(p, ids.length, 2.0);
    const r = Math.max(0, ...slots.map((s) => Math.hypot(s.x - p.x, s.y - p.y))) + 1.5;
    // nearest robot to the goal takes the central slot, and so on
    const order = [...ids].sort((a, b) => this.dist(a, p) - this.dist(b, p));
    order.forEach((id, k) => {
      this.roles.set(id, this.relayOrder.includes(id) ? "relay" : "unit");
      this.command(id, { kind: "goal", goal: slots[k], area: { x: p.x, y: p.y, r } });
    });
    if (need) this.log(`цель дальше зоны связи базы: ${need} робот(а) будут по пути вставать ретрансляторами`, "info");
  }

  /** Relays anchor where their link to the previous chain link (base or relay) gets weak. */
  private deployRelays(): void {
    const node = (id: string): MeshNode => {
      if (id === BASE_ID) return { id, x: this.world.base.x, y: this.world.base.y };
      const r = this.robots.find((x) => x.id === id)!;
      return { id, x: r.x, y: r.y };
    };
    for (let k = 0; k < this.relayOrder.length; k++) {
      const id = this.relayOrder[k];
      if (this.anchored.has(id)) continue;
      const up = k === 0 ? BASE_ID : this.relayOrder[k - 1];
      if (up !== BASE_ID && !this.anchored.has(up)) continue; // anchor in order, nearest to the base first
      const r = this.robots.find((x) => x.id === id);
      if (!r || r.mode !== "auto" || !r.goal) continue;
      const b = this.world.base;
      const upNode = node(up);
      // only a relay that has got beyond the previous chain link extends the chain
      if (up !== BASE_ID && Math.hypot(r.x - b.x, r.y - b.y) < Math.hypot(upNode.x - b.x, upNode.y - b.y) + 5) continue;
      const m = this.mesh.margin(node(id), upNode);
      if (m > RELAY_MARGIN) {
        this.lastGood.set(id, { x: r.x, y: r.y });
        continue;
      }
      const here = m >= 0 ? { x: r.x, y: r.y } : this.lastGood.get(id) ?? { x: r.x, y: r.y };
      const spot = this.anchorSpot(r, here, upNode);
      this.anchored.set(id, spot);
      r.setGoal(spot, this.t, { x: spot.x, y: spot.y, r: 1.5 });   // the robot's own decision
      this.log(`${r.name} встал ретранслятором (звено ${k + 1}), держит связь с ${up === BASE_ID ? "базой" : this.robots.find((x) => x.id === up)!.name}`, "info", id);
    }
  }

  /**
   * Where to park a relay near `around`: still linked to the upstream node, as far as possible from
   * known obstacles (not in a ford or a gap between trees — it would block the swarm) and from robots.
   */
  private anchorSpot(r: SimRobot, around: Point, upstream: MeshNode): Point {
    const w = this.world, k = r.knowledge;
    const clearance = (x: number, y: number): number => {
      const cx = Math.floor(x / w.res), cy = Math.floor(y / w.res);
      let best = 3;
      for (let dy = -6; dy <= 6; dy++)
        for (let dx = -6; dx <= 6; dx++) {
          const c = k.get(idx(w, Math.min(w.width - 1, Math.max(0, cx + dx)), Math.min(w.height - 1, Math.max(0, cy + dy))));
          if (c !== -1 && !passable(c as Cell)) best = Math.min(best, Math.hypot(dx, dy) * w.res);
        }
      return best;
    };
    let best = around, bestScore = -Infinity;
    for (let rad = 0; rad <= 4; rad += 1)
      for (let a = 0; a < 12; a++) {
        if (rad === 0 && a) break;
        const p = { x: around.x + rad * Math.cos((a * Math.PI) / 6), y: around.y + rad * Math.sin((a * Math.PI) / 6) };
        const c = cellAt(w, p.x, p.y);
        const known = k.get(idx(w, Math.floor(p.x / w.res), Math.floor(p.y / w.res)));
        if (c === null || known === -1 || !passable(known as Cell)) continue;
        if (this.mesh.margin({ id: r.id, ...p }, upstream) < 2) continue;
        const robots = Math.min(3, ...this.robots.filter((o) => o !== r).map((o) => Math.hypot(o.x - p.x, o.y - p.y)));
        const score = clearance(p.x, p.y) + 0.5 * robots - 0.15 * rad;
        if (score > bestScore) { bestScore = score; best = p; }
      }
    return best;
  }

  /** A moving robot stuck behind a parked one for > 3 s: the parked one pulls aside. */
  private makeWay(): void {
    // the operator is driving a robot and a neighbour is in the way: auto robots nearby step aside
    for (const m of this.robots) {
      if (m.mode !== "manual" || m.lastBlock !== "robot") continue;
      for (const p of this.robots) {
        if (p === m || p.mode !== "auto" || p.trapped || Math.hypot(p.x - m.x, p.y - m.y) > 1.2) continue;
        p.makeWay(Math.atan2(p.y - m.y, p.x - m.x), this.t);
      }
    }
    for (const r of this.robots) {
      if (r.mode !== "auto" || r.arrived || r.stuckFor(this.t) < 3) continue;
      for (const p of this.robots) {
        if (p === r || !p.arrived || p.mode !== "auto") continue;
        const d = Math.hypot(p.x - r.x, p.y - r.y);
        const bearing = Math.atan2(p.y - r.y, p.x - r.x) - r.th;
        if (d < 1.6 && Math.cos(bearing) > 0.3) {
          const away = Math.atan2(p.y - r.y, p.x - r.x) + (Math.sin(bearing) >= 0 ? 1.2 : -1.2); // aside, not ahead
          p.makeWay(away, this.t);
        }
      }
    }
  }

  role(id: string): "relay" | "unit" {
    return this.roles.get(id) ?? "unit";
  }

  private dist(id: string, p: Point): number {
    const v = this.baseView.get(id) ?? this.robots.find((r) => r.id === id);
    return v ? Math.hypot(v.x - p.x, v.y - p.y) : Infinity;
  }

  // ---------------------------------------------------------------- simulation
  step(dt: number): void {
    this.t += dt;
    for (const r of this.robots) {
      const others = this.robots.filter((o) => o !== r && Math.hypot(o.x - r.x, o.y - r.y) < 6)
        .map((o) => ({ id: o.id, x: o.x, y: o.y, th: o.th, priority: o.priority,
                       parked: o.arrived || o.mode !== "auto" || o.trapped }));
      r.step(dt, this.t, this.world, this.rand, others, this.incidentRate);
      if (r.help !== (this.lastHelp.get(r.id) ?? null)) {
        if (r.help) this.log(`${r.name}: ${helpText(r.help)}`, "alert", r.id);
        this.lastHelp.set(r.id, r.help);
      }
    }
    if (this.t - this.meshT >= MESH_PERIOD) {
      this.meshT = this.t;
      this.deployRelays();
      this.makeWay();
      this.meshRound();
    }
  }

  private meshRound(): void {
    for (const r of this.robots) {
      const beacon: Beacon = {
        id: r.id, t: this.t, x: r.x, y: r.y, th: r.th, v: r.v, mode: r.mode, help: r.help, arrived: r.arrived,
        battery: r.battery, goal: r.goal, ack: this.applied.get(r.id) ?? -1, path: r.path.slice(0, 40),
      };
      this.mesh.send(r.id, "beacon", beacon);
      const fresh = r.knowledge.takeFresh();
      if (fresh.length) this.mesh.send(r.id, "obstacles", fresh);
    }
    // re-send unacknowledged commands
    for (const [id, p] of this.pending) {
      const ack = this.baseView.get(id)?.ack ?? -1;
      if (ack >= p.cmd.seq) this.pending.delete(id);
      else if (this.t - p.sentT >= COMMAND_RETRY) {
        p.sentT = this.t;
        this.mesh.send(BASE_ID, "command", p.cmd, id);
      }
    }
    const nodes = this.nodes();
    for (let hop = 0; hop < 3; hop++) this.mesh.tick(nodes, (node, pkt) => this.deliver(node, pkt));
  }

  private deliver(node: string, pkt: Packet): void {
    if (node === BASE_ID) {
      if (pkt.kind === "beacon") {
        const b = pkt.payload as Beacon;
        const prev = this.baseView.get(b.id);
        if (!prev || prev.t < b.t) this.baseView.set(b.id, b);
      } else if (pkt.kind === "obstacles") {
        this.baseKnowledge.merge(pkt.payload as number[]);
      }
      return;
    }
    const r = this.robots.find((x) => x.id === node);
    if (!r) return;
    if (pkt.kind === "obstacles") {
      r.knowledge.merge(pkt.payload as number[]);
    } else if (pkt.kind === "command" && pkt.dst === r.id) {
      this.apply(r, pkt.payload as Command);
    }
  }

  private apply(r: SimRobot, cmd: Command): void {
    if (cmd.kind === "teleop") {
      if (r.mode === "manual") r.manual = { v: cmd.v, w: cmd.w, t: this.t };
      return;
    }
    if ((this.applied.get(r.id) ?? -1) >= cmd.seq) return;
    this.applied.set(r.id, cmd.seq);
    if (cmd.kind === "goal") r.setGoal(cmd.goal, this.t, cmd.area ?? null);
    else if (cmd.kind === "manual") r.takeManual();
    else if (cmd.kind === "auto") r.releaseManual(this.t);
  }

  /** Demo control: a random moving robot gets stuck on a hidden obstacle. Returns its name. */
  injectIncident(): string | null {
    const moving = this.robots.filter((r) => r.mode === "auto" && !r.arrived && !r.trapped && this.role(r.id) !== "relay");
    if (!moving.length) return null;
    const r = moving[Math.floor(this.rand() * moving.length)];
    r.trapped = true;
    r.help = "hidden";
    return r.name;
  }

  log(text: string, level: SwarmEvent["level"], robot?: string): void {
    this.events.push({ t: this.t, text, level, robot });
    if (this.events.length > 200) this.events.shift();
  }

  // ---------------------------------------------------------------- what the operator sees
  /** Robots the base has heard from, with link age and the reasons they need a human. */
  operatorView(): { id: string; name: string; beacon: Beacon | null; age: number | null; attention: string | null }[] {
    return this.robots.map((r) => {
      const b = this.baseView.get(r.id) ?? null;
      const age = b ? this.t - b.t : null;
      let attention: string | null = null;
      if (b?.help) attention = helpText(b.help);
      else if (age === null || age > STALE_AFTER) attention = "нет связи с роботом";
      return { id: r.id, name: r.name, beacon: b, age, attention };
    });
  }

  /** Commands still waiting for an acknowledgement (per robot). */
  pendingCommands(): Record<string, string> {
    const out: Record<string, string> = {};
    for (const [id, p] of this.pending) out[id] = p.cmd.kind;
    return out;
  }
}

export function helpText(h: HelpReason): string {
  switch (h) {
    case "mud": return "застрял в болоте";
    case "hidden": return "застрял на скрытом препятствии";
    case "no_path": return "нет пути к цели";
    case "no_progress": return "нет продвижения";
    default: return "";
  }
}

/**
 * Relay spots on the base → goal line so every hop stays within radio range (with a 25 % margin for
 * hills and detours). Never more than a third of the swarm.
 */
export function relayPoints(base: Point, goal: Point, baseRange: number, range: number, swarm: number): Point[] {
  const d = Math.hypot(goal.x - base.x, goal.y - base.y);
  const first = baseRange * 0.75, hop = range * 0.75;
  if (d <= first) return [];
  const n = Math.min(Math.floor(swarm / 3), Math.ceil((d - first) / hop));
  const pts: Point[] = [];
  for (let k = 0; k < n; k++) {
    const s = Math.min(d - hop * 0.5, first + k * hop) / d;
    pts.push({ x: base.x + (goal.x - base.x) * s, y: base.y + (goal.y - base.y) * s });
  }
  return pts;
}

/** n points on a hexagonal lattice around p, nearest first. */
export function hexSlots(p: Point, n: number, spacing: number): Point[] {
  const pts: Point[] = [];
  for (let ring = 0; pts.length < n; ring++) {
    if (ring === 0) { pts.push({ ...p }); continue; }
    for (let side = 0; side < 6; side++)
      for (let k = 0; k < ring; k++) {
        const a0 = (Math.PI / 3) * side, a1 = (Math.PI / 3) * (side + 1);
        const c0 = { x: Math.cos(a0) * ring, y: Math.sin(a0) * ring };
        const c1 = { x: Math.cos(a1) * ring, y: Math.sin(a1) * ring };
        const f = k / ring;
        pts.push({ x: p.x + (c0.x + (c1.x - c0.x) * f) * spacing, y: p.y + (c0.y + (c1.y - c0.y) * f) * spacing });
      }
  }
  return pts.slice(0, n);
}
