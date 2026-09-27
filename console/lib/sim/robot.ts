/**
 * One simulated UGV of the swarm: same limits as the real cart (0.55 m/s, tank turns, 0.62 × 0.56 m),
 * an 8 m lidar, its own terrain knowledge, A* on that knowledge, yielding to neighbours, and the
 * reasons to call the operator.
 */
import { Knowledge, observe } from "./knowledge";
import { aStar, costGrid, nearestFree, smooth } from "./planner";
import { CELL_INFO, Cell, World, cellAt, idx, passable } from "./world";

export const MAX_SPEED = 0.55; // m/s (2 km/h)
export const MAX_TURN = 0.9; // rad/s
export const LIDAR_RANGE = 8;
export const BODY_RADIUS = 0.35; // m, for robot-robot spacing
export const ARRIVE_RADIUS = 0.8;
const FREED_AFTER = 0.3; // m the operator must drive a snagged robot to free it

export type Mode = "idle" | "auto" | "manual";
export type HelpReason = "mud" | "hidden" | "no_path" | "no_progress" | null;

export interface Point {
  x: number;
  y: number;
}

export interface Neighbour {
  id: string;
  x: number;
  y: number;
  th: number;
  priority: number; // lower = more important
  parked: boolean; // arrived / idle / manual: a static obstacle for everyone
}

export class SimRobot {
  x: number;
  y: number;
  th: number;
  v = 0;
  w = 0;
  mode: Mode = "idle";
  goal: Point | null = null;
  area: { x: number; y: number; r: number } | null = null; // the whole swarm's destination area
  path: Point[] = [];
  arrived = false;
  help: HelpReason = null;
  trapped = false;
  battery = 100;
  odometer = 0;
  waiting = false; // yielding to a neighbour
  manual = { v: 0, w: 0, t: -1 };
  lastBlock: "terrain" | "robot" | null = null; // why the last move was refused (debug / UI)
  private manualOdo = 0; // metres driven by the operator since taking over
  readonly knowledge: Knowledge;
  private plannedVersion = -1;
  private planT = -1e9;
  private bestDist = Infinity;
  private progressT = 0;
  private sightT = -1;
  private blockedSince = -1; // the body could not move although commanded
  private unstick: { until: number; heading: number } | null = null;
  private waitingSince = -1;
  private holdUntil = -1; // giving way after a retreat

  constructor(readonly id: string, readonly name: string, readonly priority: number, world: World,
              x: number, y: number, th = 0) {
    this.x = x;
    this.y = y;
    this.th = th;
    this.knowledge = new Knowledge(world.width, world.height);
  }

  /** How long this robot has been blocked or waiting (s). */
  stuckFor(t: number): number {
    const b = this.blockedSince >= 0 ? t - this.blockedSince : 0;
    const w = this.waitingSince >= 0 ? t - this.waitingSince : 0;
    const noProgress = this.mode === "auto" && !this.arrived ? t - this.progressT - 7 : 0; // own unstick moves reset b
    return Math.max(b, w, noProgress);
  }

  /** Parked robot: pull 0.8 m aside along `heading` (another robot needs the spot). */
  makeWay(heading: number, t: number): void {
    if (!this.unstick) this.unstick = { until: t + 3.5, heading };
  }

  setGoal(p: Point | null, t: number, area: { x: number; y: number; r: number } | null = null): void {
    this.goal = p;
    this.area = area;
    this.path = [];
    this.arrived = false;
    this.plannedVersion = -1;
    this.bestDist = Infinity;
    this.progressT = t;
    if (this.help === "no_path" || this.help === "no_progress") this.help = null;
    this.mode = p ? "auto" : "idle";
  }

  takeManual(): void {
    this.mode = "manual";
    this.manual = { v: 0, w: 0, t: -1 };
    this.manualOdo = this.odometer;
  }

  releaseManual(t: number): void {
    this.mode = this.goal && !this.arrived ? "auto" : "idle";
    if (!this.trapped) this.help = null;
    this.plannedVersion = -1;
    this.bestDist = Infinity;
    this.progressT = t;
  }

  /** Advance dt seconds. `others` = true positions of the other robots (the lidar sees robots nearby). */
  step(dt: number, t: number, world: World, rand: () => number, others: Neighbour[], incidentRate = 0): void {
    if (t - this.sightT >= 0.5) {
      observe(world, this.knowledge, this.x, this.y, LIDAR_RANGE);
      this.sightT = t;
    }
    const here = cellAt(world, this.x, this.y) ?? Cell.Ground;
    let v = 0, w = 0;
    this.waiting = false;

    if (this.mode === "auto" && this.arrived && this.unstick) {
      // parked, but asked to make way for a robot behind: pull aside, then park again
      if (t > this.unstick.until) this.unstick = null;
      else {
        const e = wrap(this.unstick.heading - this.th);
        v = Math.abs(e) > 0.3 ? 0 : 0.25;
        w = Math.max(-MAX_TURN, Math.min(MAX_TURN, 2 * e));
      }
    } else if (this.mode === "manual") {
      if (t - this.manual.t < 0.6) {
        v = Math.max(-MAX_SPEED * 0.5, Math.min(MAX_SPEED, this.manual.v));
        w = Math.max(-MAX_TURN, Math.min(MAX_TURN, this.manual.w));
      }
      // the operator drove it out: out of the mud, or off a hidden snag
      if (this.trapped && (this.help === "mud" ? here !== Cell.Mud : this.odometer - this.manualOdo > FREED_AFTER)) {
        this.trapped = false;
        this.help = null;
      }
    } else if (this.mode === "auto" && this.goal) {
      if (here === Cell.Mud && !this.trapped && rand() < 0.04 * dt) {
        this.trapped = true;
        this.help = "mud";
      }
      // demo incidents: a root / a hole hidden in rough ground or undergrowth
      const exposure = here === Cell.Forest ? 1.5 : here === Cell.Rough ? 1 : here === Cell.Grass ? 0.4 : 0; // stumps, holes
      if (incidentRate > 0 && !this.trapped && exposure > 0 && Math.abs(this.v) > 0.05
          && rand() < incidentRate * exposure * dt) {
        this.trapped = true;
        this.help = "hidden";
      }
      if (!this.trapped) [v, w] = this.autopilot(t, world, others);
    }

    // physics: terrain speed, no driving into obstacles or other robots
    const terrain = CELL_INFO[here].speed || 0.2;
    const trappedFactor = this.trapped && this.mode !== "manual" ? 0 : 1; // the operator drives it off the snag
    v *= terrain * trappedFactor;
    this.th = wrap(this.th + w * dt);
    const nx = this.x + Math.cos(this.th) * v * dt, ny = this.y + Math.sin(this.th) * v * dt;
    const probeX = nx + Math.cos(this.th) * Math.sign(v) * 0.3, probeY = ny + Math.sin(this.th) * Math.sign(v) * 0.3;
    const probe = cellAt(world, probeX, probeY);
    // a robot may always move away from one it touches; only closing in below 2 radii is a collision
    const hitsRobot = others.some((o) => {
      const dNew = Math.hypot(o.x - nx, o.y - ny);
      return dNew < 2 * BODY_RADIUS && dNew < Math.hypot(o.x - this.x, o.y - this.y);
    });
    this.lastBlock = probe === null || !passable(probe) ? "terrain" : hitsRobot ? "robot" : null;
    if (probe !== null && passable(probe) && !hitsRobot) {
      this.odometer += Math.abs(v * dt);
      this.x = nx;
      this.y = ny;
      if (Math.abs(v) > 0.01) this.blockedSince = -1;
    } else {
      if (Math.abs(v) > 0.01 && this.blockedSince < 0) this.blockedSince = t;
      v = 0;
    }
    this.v = v;
    this.w = w;
    if (this.waiting) { if (this.waitingSince < 0) this.waitingSince = t; } else this.waitingSince = -1;
    this.battery = Math.max(0, this.battery - (Math.abs(v) > 0.01 ? 0.004 : 0.0005) * dt);
  }

  private autopilot(t: number, world: World, others: Neighbour[]): [number, number] {
    const goal = this.goal!;
    const dist = Math.hypot(goal.x - this.x, goal.y - this.y);
    const blockedFor = this.blockedSince >= 0 ? t - this.blockedSince : 0;
    const crowdAtGoal = others.some((o) => o.parked && Math.hypot(o.x - goal.x, o.y - goal.y) < 2.5);
    const stalled = t - this.progressT > 8 || blockedFor > 3;
    const inArea = this.area !== null && Math.hypot(this.area.x - this.x, this.area.y - this.y) < this.area.r;
    const nextToParked = others.some((o) => o.parked && Math.hypot(o.x - this.x, o.y - this.y) < 2.5);
    if (dist < ARRIVE_RADIUS || (stalled && ((dist < 3.0 && crowdAtGoal) || (inArea && nextToParked)))) {
      // my slot is taken or walled in by the parked swarm: inside the destination area is good enough
      this.arrived = true;
      this.path = [];
      if (this.help === "no_progress" || this.help === "no_path") this.help = null;
      return [0, 0];
    }
    // progress watchdog. Queueing behind a higher-priority robot (a ford, a gap) is not a problem for
    // the operator; being stuck for a minute without anyone in front is.
    const queued = others.some((o) => !o.parked && o.priority < this.priority && Math.hypot(o.x - this.x, o.y - this.y) < 4);
    if (queued) this.progressT = Math.max(this.progressT, t - 30);
    if (dist < this.bestDist - 1.0) {
      this.bestDist = dist;
      this.progressT = t;
      if (this.help === "no_progress") this.help = null;   // moving again
    } else if (t - this.progressT > 60 && !this.help) {
      this.help = "no_progress";
    }
    // robots I must plan around: parked ones, and moving ones with a higher priority
    const blockers = others.filter((o) => (o.parked || o.priority < this.priority) && Math.hypot(o.x - this.x, o.y - this.y) < 6);
    const crowded = blockers.some((o) => Math.hypot(o.x - this.x, o.y - this.y) < 3);
    // (re)plan when new terrain is known, periodically, near blocking robots, or without a path
    const stale = (this.knowledge.version !== this.plannedVersion && t - this.planT > 2) || t - this.planT > 5;
    const blocked = this.blockedSince >= 0 && t - this.blockedSince > 0.5; // e.g. a shortcut clips a river bank
    if (!this.path.length || stale || blocked || (crowded && t - this.planT > 1) || (this.help === "no_path" && t - this.planT > 5)) {
      this.blockedSince = -1;
      this.plan(t, world, blockers);
    }
    if (!this.path.length) return [0, 0];

    // right of way: stuck next to a higher-priority robot → retreat away from it, then hold and let it pass.
    // The highest-priority robot of any jam never yields, so every jam dissolves (priorities are a total order).
    const higher = others.filter((o) => !o.parked && o.priority < this.priority && Math.hypot(o.x - this.x, o.y - this.y) < 1.8);
    if (!this.unstick && t >= this.holdUntil && higher.length && this.stuckFor(t) > 2) {
      this.unstick = { until: t + 2.5, heading: freestHeading(this, world, higher) };
      this.holdUntil = t + 6;
      this.blockedSince = -1;
    }
    if (!this.unstick && t < this.holdUntil) {
      this.waiting = true;
      return [0, 0];
    }
    // unstick: blocked for a while (a tree, a parked robot) → turn to the freest direction and pull away
    if (blockedFor > 3 && !this.unstick) {
      this.unstick = { until: t + 2.5, heading: freestHeading(this, world, others) };
      this.blockedSince = -1;
    }
    if (this.unstick) {
      if (t > this.unstick.until) {
        this.unstick = null;
        this.path = [];
        return [0, 0];
      }
      const e = wrap(this.unstick.heading - this.th);
      return [Math.abs(e) > 0.3 ? 0 : 0.25, Math.max(-MAX_TURN, Math.min(MAX_TURN, 2 * e))];
    }

    // pure pursuit: look-ahead point 1.2 m along the path line (keeps the robot on the line, e.g. mid-ford)
    const wp = lookAhead(this.path, this.x, this.y, 1.2);
    while (this.path.length > 1 && Math.hypot(this.path[0].x - this.x, this.path[0].y - this.y) < 0.5) this.path.shift();
    const err = wrap(Math.atan2(wp.y - this.y, wp.x - this.x) - this.th);
    let w = Math.max(-MAX_TURN, Math.min(MAX_TURN, 1.8 * err));
    let v = Math.abs(err) > 0.8 ? 0 : MAX_SPEED * Math.cos(err) ** 2; // tank turn first when far off

    // neighbours: the lower priority gives way; parked robots are just obstacles (planned around)
    for (const o of others) {
      const d = Math.hypot(o.x - this.x, o.y - this.y);
      if (d > 3 || o.parked) continue;
      const bearing = wrap(Math.atan2(o.y - this.y, o.x - this.x) - this.th);
      const ahead = Math.abs(bearing) < 1.0;
      if (!ahead || o.priority > this.priority) continue;   // behind me, or I have the right of way
      const headOn = Math.abs(wrap(Math.atan2(this.y - o.y, this.x - o.x) - o.th)) < 1.0;
      if (headOn && d < 1.4) {
        v = -0.15; // face to face in a narrow spot: back off and let it pass
        w = bearing > 0 ? -0.3 : 0.3;
        this.waiting = true;
      } else if (d < 1.2) {
        v = 0;
        this.waiting = true;
      } else if (d < 2.2) {
        v = Math.min(v, 0.15);
        this.waiting = true;
      }
    }
    return [v, Math.max(-MAX_TURN, Math.min(MAX_TURN, w))];
  }

  private plan(t: number, world: World, blockers: Neighbour[] = []): void {
    this.planT = t;
    this.plannedVersion = this.knowledge.version;
    const base = costGrid(this.knowledge, 1);
    // first try around the blocking robots; if they box us in, plan through them and rely on yielding
    const cells = this.search(world, withRobots(base, blockers, world)) ?? this.search(world, base);
    if (!cells) {
      this.path = [];
      this.help = "no_path";
      return;
    }
    if (this.help === "no_path") this.help = null;
    this.path = smooth(cells, base, world.width).map((i) => ({
      x: ((i % world.width) + 0.5) * world.res,
      y: (Math.floor(i / world.width) + 0.5) * world.res,
    }));
  }

  private search(world: World, cost: Float32Array): number[] | null {
    const cx = Math.floor(this.x / world.res), cy = Math.floor(this.y / world.res);
    const g = this.goal!;
    const goal = nearestFree(cost, world.width, world.height,
      idx(world, Math.floor(g.x / world.res), Math.floor(g.y / world.res)), 4);
    return goal >= 0 ? aStar(cost, world.width, world.height, idx(world, cx, cy), goal) : null;
  }
}

/** Robots as temporary round obstacles (0.6 m) in a copy of the cost grid. */
function withRobots(cost: Float32Array, robots: Neighbour[], world: World): Float32Array {
  if (!robots.length) return cost;
  const out = cost.slice();
  const r = Math.ceil(0.6 / world.res);
  for (const o of robots) {
    const ox = Math.floor(o.x / world.res), oy = Math.floor(o.y / world.res);
    for (let dy = -r; dy <= r; dy++)
      for (let dx = -r; dx <= r; dx++) {
        const x = ox + dx, y = oy + dy;
        if (dx * dx + dy * dy <= r * r && x >= 0 && y >= 0 && x < world.width && y < world.height) out[y * world.width + x] = Infinity;
      }
  }
  return out;
}

/** Point `dist` metres ahead along the polyline from the projection of (x, y) onto it. */
export function lookAhead(path: Point[], x: number, y: number, dist: number): Point {
  if (path.length === 1) return path[0];
  let best = 0, bestD = Infinity, bestT = 0;
  for (let i = 0; i + 1 < path.length; i++) {
    const a = path[i], b = path[i + 1];
    const dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy || 1e-9;
    const tt = Math.max(0, Math.min(1, ((x - a.x) * dx + (y - a.y) * dy) / len2));
    const d = Math.hypot(a.x + dx * tt - x, a.y + dy * tt - y);
    if (d < bestD) { bestD = d; best = i; bestT = tt; }
  }
  let remaining = dist;
  let px = path[best].x + (path[best + 1].x - path[best].x) * bestT;
  let py = path[best].y + (path[best + 1].y - path[best].y) * bestT;
  for (let i = best; i + 1 < path.length; i++) {
    const b = path[i + 1];
    const seg = Math.hypot(b.x - px, b.y - py);
    if (seg >= remaining) return { x: px + ((b.x - px) * remaining) / seg, y: py + ((b.y - py) * remaining) / seg };
    remaining -= seg;
    px = b.x;
    py = b.y;
  }
  return path[path.length - 1];
}

/** Heading (of 16) whose next 1 m is passable and farthest from other robots. */
function freestHeading(r: { x: number; y: number; th: number }, world: World, others: Neighbour[]): number {
  let best = r.th + Math.PI, bestScore = -Infinity;
  for (let k = 0; k < 16; k++) {
    const a = r.th + (k * Math.PI) / 8;
    let ok = true;
    for (const d of [0.4, 0.7, 1.0]) {
      const c = cellAt(world, r.x + Math.cos(a) * d, r.y + Math.sin(a) * d);
      if (c === null || !passable(c)) ok = false;
    }
    if (!ok) continue;
    const ex = r.x + Math.cos(a), ey = r.y + Math.sin(a);
    const clearance = Math.min(3, ...others.map((o) => Math.hypot(o.x - ex, o.y - ey)));
    const score = clearance - 0.1 * Math.abs(wrap(a - r.th));
    if (score > bestScore) { bestScore = score; best = a; }
  }
  return best;
}

export function wrap(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}
