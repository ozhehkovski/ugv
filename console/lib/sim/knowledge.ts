/**
 * What one robot knows about the terrain: -1 unknown, else the observed Cell.
 * Filled by its own lidar and by obstacle reports received over the mesh.
 */
import { CELL_INFO, Cell, World, idx, inBounds } from "./world";

export const UNKNOWN = -1;

export class Knowledge {
  readonly cells: Int8Array;
  /** cells learnt since the last mesh broadcast (indices) */
  fresh: number[] = [];
  version = 0;

  constructor(readonly width: number, readonly height: number) {
    this.cells = new Int8Array(width * height).fill(UNKNOWN);
  }

  get(i: number): number {
    return this.cells[i];
  }

  /** Learn a cell; returns true when it changed what the robot knew. */
  learn(i: number, c: Cell): boolean {
    if (this.cells[i] === c) return false;
    this.cells[i] = c;
    this.fresh.push(i);
    this.version++;
    return true;
  }

  /** Merge a report (flat [index, cell, index, cell, ...]); returns how many cells were new. */
  merge(report: ArrayLike<number>): number {
    let n = 0;
    for (let k = 0; k + 1 < report.length; k += 2) {
      const i = report[k], c = report[k + 1];
      if (this.cells[i] !== c) {
        this.cells[i] = c;
        this.version++;
        n++;
      }
    }
    return n;
  }

  /** Take the cells learnt since the last call, as a flat report. */
  takeFresh(limit = 4000): number[] {
    const out: number[] = [];
    const take = this.fresh.splice(0, limit);
    for (const i of take) out.push(i, this.cells[i]);
    return out;
  }

  knownCount(): number {
    let n = 0;
    for (let i = 0; i < this.cells.length; i++) if (this.cells[i] !== UNKNOWN) n++;
    return n;
  }
}

/** Lidar: cast rays from (x, y) up to range; trees and rocks stop a ray. */
export function observe(world: World, k: Knowledge, x: number, y: number, range: number, rays = 90): number {
  let learnt = 0;
  const step = world.res * 0.5;
  for (let r = 0; r < rays; r++) {
    const a = (2 * Math.PI * r) / rays;
    const dx = Math.cos(a) * step, dy = Math.sin(a) * step;
    let px = x, py = y;
    for (let d = 0; d <= range; d += step) {
      const cx = Math.floor(px / world.res), cy = Math.floor(py / world.res);
      if (!inBounds(world, cx, cy)) break;
      const i = idx(world, cx, cy);
      const c = world.cells[i] as Cell;
      if (k.learn(i, c)) learnt++;
      if (CELL_INFO[c].blocksSight) break;
      px += dx;
      py += dy;
    }
  }
  return learnt;
}
