/**
 * A* on a robot's own knowledge grid (8-connected). Cells next to impassable ones are expensive so the
 * 0.56 m-wide body keeps clear of trees, rocks, water and the ravine edge. Unknown cells are allowed
 * at a mild premium: the swarm explores, but prefers terrain it has already seen.
 */
import { Knowledge, UNKNOWN } from "./knowledge";
import { CELL_INFO, Cell } from "./world";

export const UNKNOWN_COST = 1.4;

export function cellCost(k: Knowledge, i: number): number {
  const c = k.get(i);
  if (c === UNKNOWN) return UNKNOWN_COST;
  return CELL_INFO[c as Cell].cost;
}

export const INFLATED_COST = 20;

/** Cost grid; Infinity = blocked. Cells next to obstacles get INFLATED_COST: the planner avoids them
 * but a robot that is already there can still plan its way out (like a Nav2 inflation layer). */
export function costGrid(k: Knowledge, inflate = 1): Float32Array {
  const { width, height } = k;
  const g = new Float32Array(width * height);
  for (let i = 0; i < g.length; i++) g[i] = cellCost(k, i);
  if (inflate <= 0) return g;
  const out = g.slice();
  for (let y = 0; y < height; y++)
    for (let x = 0; x < width; x++) {
      if (g[y * width + x] !== Infinity) continue;
      for (let dy = -inflate; dy <= inflate; dy++)
        for (let dx = -inflate; dx <= inflate; dx++) {
          const nx = x + dx, ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < width && ny < height && out[ny * width + nx] !== Infinity)
            out[ny * width + nx] = Math.max(out[ny * width + nx], INFLATED_COST);
        }
    }
  return out;
}

class Heap {
  private a: [number, number][] = [];
  get size(): number {
    return this.a.length;
  }
  push(p: number, v: number): void {
    const a = this.a;
    a.push([p, v]);
    let i = a.length - 1;
    while (i > 0) {
      const j = (i - 1) >> 1;
      if (a[j][0] <= a[i][0]) break;
      [a[i], a[j]] = [a[j], a[i]];
      i = j;
    }
  }
  pop(): number {
    const a = this.a;
    const top = a[0][1];
    const last = a.pop()!;
    if (a.length) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < a.length && a[l][0] < a[m][0]) m = l;
        if (r < a.length && a[r][0] < a[m][0]) m = r;
        if (m === i) break;
        [a[i], a[m]] = [a[m], a[i]];
        i = m;
      }
    }
    return top;
  }
}

/**
 * Path of cell indices from start to goal, or null. The goal may be in an inflated cell (e.g. a
 * slot near a wall): it is then searched to the nearest free cell within 3 cells.
 */
export function aStar(cost: Float32Array, width: number, height: number, start: number, goal: number,
                      maxExpand = 200_000): number[] | null {
  const n = width * height;
  const g = new Float32Array(n).fill(Infinity);
  const came = new Int32Array(n).fill(-1);
  const closed = new Uint8Array(n);
  const gx = goal % width, gy = Math.floor(goal / width);
  const h = (i: number) => {
    const dx = Math.abs((i % width) - gx), dy = Math.abs(Math.floor(i / width) - gy);
    return (dx + dy + (Math.SQRT2 - 2) * Math.min(dx, dy)) * 0.95;
  };
  const heap = new Heap();
  g[start] = 0;
  heap.push(h(start), start);
  let expanded = 0;
  while (heap.size && expanded < maxExpand) {
    const cur = heap.pop();
    if (closed[cur]) continue;
    if (cur === goal) {
      const path = [cur];
      let p = cur;
      while (came[p] !== -1) path.push((p = came[p]));
      return path.reverse();
    }
    closed[cur] = 1;
    expanded++;
    const cx = cur % width, cy = Math.floor(cur / width);
    for (let dy = -1; dy <= 1; dy++)
      for (let dx = -1; dx <= 1; dx++) {
        if (!dx && !dy) continue;
        const nx = cx + dx, ny = cy + dy;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
        const ni = ny * width + nx;
        const c = cost[ni];
        if (c === Infinity && ni !== start) continue;
        if (dx && dy && (cost[cy * width + nx] === Infinity || cost[ny * width + cx] === Infinity)) continue; // no corner cutting
        const ng = g[cur] + (dx && dy ? Math.SQRT2 : 1) * (c === Infinity ? 1 : c);
        if (ng < g[ni]) {
          g[ni] = ng;
          came[ni] = cur;
          heap.push(ng + h(ni), ni);
        }
      }
  }
  return null;
}

/** Nearest cell with finite cost to `i` within `radius` cells (Chebyshev), or -1. */
export function nearestFree(cost: Float32Array, width: number, height: number, i: number, radius = 3): number {
  if (cost[i] !== Infinity) return i;
  const x0 = i % width, y0 = Math.floor(i / width);
  let best = -1, bestD = Infinity;
  for (let dy = -radius; dy <= radius; dy++)
    for (let dx = -radius; dx <= radius; dx++) {
      const x = x0 + dx, y = y0 + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) continue;
      const j = y * width + x;
      const d = dx * dx + dy * dy;
      if (cost[j] !== Infinity && d < bestD) { best = j; bestD = d; }
    }
  return best;
}

/** Drop intermediate cells while the straight segment stays on cells no worse than the path's. */
export function smooth(path: number[], cost: Float32Array, width: number): number[] {
  if (path.length <= 2) return path;
  const out = [path[0]];
  let anchor = 0;
  const terrain = (i: number) => (cost[i] >= INFLATED_COST ? 1.2 : cost[i]);
  const clear = (a: number, b: number) => {
    const ax = a % width, ay = Math.floor(a / width), bx = b % width, by = Math.floor(b / width);
    const steps = Math.max(Math.abs(bx - ax), Math.abs(by - ay)) * 2;
    const maxCost = Math.max(terrain(a), terrain(b), 1.2) + 1e-3;
    for (let s = 1; s < steps; s++) {
      const x = Math.round(ax + ((bx - ax) * s) / steps), y = Math.round(ay + ((by - ay) * s) / steps);
      if (cost[y * width + x] > maxCost) return false;   // includes cells next to obstacles (INFLATED_COST)
    }
    return true;
  };
  for (let i = 2; i < path.length; i++) {
    if (!clear(path[anchor], path[i])) {
      out.push(path[i - 1]);
      anchor = i - 1;
    }
  }
  out.push(path[path.length - 1]);
  return out;
}
