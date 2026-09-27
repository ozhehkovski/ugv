/**
 * Mesh radio between robots and the operator base: limited range, hills cast radio shadow, packet
 * loss per hop, flooding with TTL and duplicate suppression, one hop per mesh tick. The operator only
 * knows what reaches the base; commands reach a robot only if there is a multi-hop route.
 */
import { Cell, World, cellAt, rng } from "./world";

export interface MeshNode {
  id: string;
  x: number;
  y: number;
}

export type PacketKind = "beacon" | "obstacles" | "command";

export interface Packet {
  key: string; // unique: src/seq
  src: string;
  dst?: string; // unicast target (still flooded); undefined = broadcast
  kind: PacketKind;
  ttl: number;
  hops: number;
  payload: unknown;
}

export interface MeshConfig {
  range: number; // m in the open, robot ↔ robot
  mastRange?: number; // m for the operator base mast (a link uses the mean of both ends)
  mastId?: string;
  hillPenalty: number; // m of range lost per hill cell (0.5 m) standing between the two ends
  loss: number; // per-hop loss probability
  ttl: number;
}

export const DEFAULT_MESH: MeshConfig = { range: 35, mastRange: 70, mastId: "base", hillPenalty: 2, loss: 0.05, ttl: 8 };
const SHADOW_CLEAR = 3; // m around each antenna where terrain does not shadow the link

export type Deliver = (node: string, packet: Packet) => void;

export class Mesh {
  private outbox = new Map<string, Packet[]>();
  private seen = new Map<string, Set<string>>();
  private seq = 0;
  private rand: () => number;
  stats = { sent: 0, delivered: 0, lost: 0 };

  constructor(readonly world: World, readonly cfg: MeshConfig = DEFAULT_MESH, seed = 1) {
    this.rand = rng(seed);
  }

  /** Effective link: distance within the range left after the hills on the line. */
  rangeOf(id: string): number {
    return id === this.cfg.mastId && this.cfg.mastRange ? this.cfg.mastRange : this.cfg.range;
  }

  linkRange(a: MeshNode, b: MeshNode): number {
    return (this.rangeOf(a.id) + this.rangeOf(b.id)) / 2;
  }

  /** Hill cells that stand BETWEEN a and b (a hill under an antenna does not shadow it). */
  private shadow(a: MeshNode, b: MeshNode): number {
    const d = Math.hypot(a.x - b.x, a.y - b.y);
    let hills = 0;
    const steps = Math.max(1, Math.ceil(d / this.world.res));
    for (let s = 1; s < steps; s++) {
      const along = (s / steps) * d;
      if (along < SHADOW_CLEAR || d - along < SHADOW_CLEAR) continue;
      const t = s / steps;
      if (cellAt(this.world, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t) === Cell.Hill) hills++;
    }
    return hills;
  }

  /** Metres of radio range left on the a–b link (negative = no link). What a robot sees as signal strength. */
  margin(a: MeshNode, b: MeshNode): number {
    return this.linkRange(a, b) - this.cfg.hillPenalty * this.shadow(a, b) - Math.hypot(a.x - b.x, a.y - b.y);
  }

  linked(a: MeshNode, b: MeshNode): boolean {
    const d = Math.hypot(a.x - b.x, a.y - b.y);
    if (d > this.linkRange(a, b)) return false;   // cheap reject before the shadow scan
    return this.margin(a, b) >= 0;
  }

  links(nodes: MeshNode[]): [string, string][] {
    const out: [string, string][] = [];
    for (let i = 0; i < nodes.length; i++)
      for (let j = i + 1; j < nodes.length; j++)
        if (this.linked(nodes[i], nodes[j])) out.push([nodes[i].id, nodes[j].id]);
    return out;
  }

  /** Nodes reachable from `from` over any number of hops (ignores loss). */
  reachable(nodes: MeshNode[], from: string): Set<string> {
    const byId = new Map(nodes.map((n) => [n.id, n]));
    const seen = new Set([from]);
    const queue = [from];
    while (queue.length) {
      const cur = byId.get(queue.shift()!)!;
      for (const n of nodes)
        if (!seen.has(n.id) && this.linked(cur, n)) {
          seen.add(n.id);
          queue.push(n.id);
        }
    }
    return seen;
  }

  send(src: string, kind: PacketKind, payload: unknown, dst?: string): void {
    const p: Packet = { key: `${src}/${this.seq++}`, src, dst, kind, ttl: this.cfg.ttl, hops: 0, payload };
    this.markSeen(src, p.key);
    this.queue(src, p);
    this.stats.sent++;
  }

  /** One hop: every queued packet goes to every current neighbour of its holder. */
  tick(nodes: MeshNode[], deliver: Deliver): void {
    const pending = this.outbox;
    this.outbox = new Map();
    const byId = new Map(nodes.map((n) => [n.id, n]));
    for (const [holder, packets] of pending) {
      const from = byId.get(holder);
      if (!from) continue;
      for (const to of nodes) {
        if (to.id === holder || !this.linked(from, to)) continue;
        for (const p of packets) {
          if (this.hasSeen(to.id, p.key)) continue;
          if (this.rand() < this.cfg.loss) {
            this.stats.lost++;
            continue;
          }
          this.markSeen(to.id, p.key);
          const got = { ...p, hops: p.hops + 1, ttl: p.ttl - 1 };
          if (!p.dst || p.dst === to.id) {
            deliver(to.id, got);
            this.stats.delivered++;
          }
          if (got.ttl > 0 && p.dst !== to.id) this.queue(to.id, got);
        }
      }
    }
  }

  private queue(node: string, p: Packet): void {
    const q = this.outbox.get(node);
    if (q) q.push(p);
    else this.outbox.set(node, [p]);
  }

  private hasSeen(node: string, key: string): boolean {
    return this.seen.get(node)?.has(key) ?? false;
  }

  private markSeen(node: string, key: string): void {
    let s = this.seen.get(node);
    if (!s) this.seen.set(node, (s = new Set()));
    s.add(key);
    if (s.size > 20000) {
      // forget the oldest half: TTL makes very old keys irrelevant
      const keep = [...s].slice(-10000);
      this.seen.set(node, new Set(keep));
    }
  }
}
