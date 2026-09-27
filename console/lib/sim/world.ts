/**
 * Cross-country terrain for the swarm demo: a deterministic grid generated from a seed.
 * 0.5 m cells, 120 × 80 m. Base (start) on the left, destination on the right, a river with
 * fords, a ravine with a narrow pass, forests, rock fields, mud and hills that block radio.
 */

export enum Cell {
  Ground = 0,
  Grass = 1,
  Rough = 2,
  Mud = 3,
  Forest = 4, // passable undergrowth between trees
  Tree = 5, // obstacle, blocks the lidar
  Rock = 6, // obstacle, blocks the lidar
  Water = 7, // impassable, the lidar sees over it
  Ford = 8, // shallow crossing
  Ravine = 9, // impassable drop
  Hill = 10, // passable, slows, blocks radio
}

export interface CellInfo {
  name: string;
  speed: number; // fraction of the robot's top speed; 0 = impassable
  cost: number; // planner cost per metre
  blocksSight: boolean;
  color: string;
}

export const CELL_INFO: Record<Cell, CellInfo> = {
  [Cell.Ground]: { name: "грунт", speed: 1.0, cost: 1.0, blocksSight: false, color: "#8a7a5c" },
  [Cell.Grass]: { name: "трава", speed: 0.9, cost: 1.15, blocksSight: false, color: "#6d8f4e" },
  [Cell.Rough]: { name: "кочки", speed: 0.6, cost: 1.8, blocksSight: false, color: "#7d6b4a" },
  [Cell.Mud]: { name: "болото", speed: 0.3, cost: 4.0, blocksSight: false, color: "#4f4a2e" },
  [Cell.Forest]: { name: "подлесок", speed: 0.5, cost: 2.2, blocksSight: false, color: "#3f6b35" },
  [Cell.Tree]: { name: "дерево", speed: 0, cost: Infinity, blocksSight: true, color: "#1f3d1c" },
  [Cell.Rock]: { name: "камень", speed: 0, cost: Infinity, blocksSight: true, color: "#6b6b6b" },
  [Cell.Water]: { name: "вода", speed: 0, cost: Infinity, blocksSight: false, color: "#2f5f8a" },
  [Cell.Ford]: { name: "брод", speed: 0.35, cost: 3.0, blocksSight: false, color: "#5a86a8" },
  [Cell.Ravine]: { name: "овраг", speed: 0, cost: Infinity, blocksSight: false, color: "#2b2118" },
  [Cell.Hill]: { name: "холм", speed: 0.7, cost: 1.5, blocksSight: false, color: "#9a8a60" },
};

export const passable = (c: Cell): boolean => CELL_INFO[c].speed > 0;

export interface World {
  width: number; // cells
  height: number;
  res: number; // m per cell
  cells: Uint8Array; // row-major, y up (row 0 = y 0)
  base: { x: number; y: number }; // operator base station (m)
  start: { x: number; y: number };
  destination: { x: number; y: number };
  seed: number;
}

/** Small fast deterministic PRNG (mulberry32). */
export function rng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Smooth 2D value noise in [0, 1). */
function valueNoise(width: number, height: number, scale: number, rand: () => number): Float32Array {
  const gw = Math.ceil(width / scale) + 2;
  const gh = Math.ceil(height / scale) + 2;
  const grid = new Float32Array(gw * gh).map(() => rand());
  const out = new Float32Array(width * height);
  const fade = (t: number) => t * t * (3 - 2 * t);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const gx = x / scale, gy = y / scale;
      const x0 = Math.floor(gx), y0 = Math.floor(gy);
      const tx = fade(gx - x0), ty = fade(gy - y0);
      const v = (i: number, j: number) => grid[j * gw + i];
      const a = v(x0, y0) * (1 - tx) + v(x0 + 1, y0) * tx;
      const b = v(x0, y0 + 1) * (1 - tx) + v(x0 + 1, y0 + 1) * tx;
      out[y * width + x] = a * (1 - ty) + b * ty;
    }
  }
  return out;
}

export const idx = (w: World, cx: number, cy: number): number => cy * w.width + cx;
export const inBounds = (w: World, cx: number, cy: number): boolean =>
  cx >= 0 && cy >= 0 && cx < w.width && cy < w.height;
export const cellAt = (w: World, x: number, y: number): Cell | null => {
  const cx = Math.floor(x / w.res), cy = Math.floor(y / w.res);
  return inBounds(w, cx, cy) ? (w.cells[idx(w, cx, cy)] as Cell) : null;
};

export function generateWorld(seed = 7, width = 240, height = 160, res = 0.5): World {
  const rand = rng(seed);
  const cells = new Uint8Array(width * height);
  const w: World = {
    width, height, res, cells, seed,
    base: { x: 4, y: height * res * 0.5 },
    start: { x: 9, y: height * res * 0.5 },
    destination: { x: width * res - 10, y: height * res * 0.55 },
  };
  const set = (cx: number, cy: number, c: Cell) => { if (inBounds(w, cx, cy)) cells[idx(w, cx, cy)] = c; };
  const blob = (cx: number, cy: number, r: number, fn: (x: number, y: number, d: number) => void) => {
    for (let y = Math.floor(cy - r); y <= cy + r; y++)
      for (let x = Math.floor(cx - r); x <= cx + r; x++) {
        const d = Math.hypot(x - cx, y - cy);
        if (d <= r && inBounds(w, x, y)) fn(x, y, d);
      }
  };

  // ground / grass / rough from noise
  const n = valueNoise(width, height, 18, rand);
  for (let i = 0; i < cells.length; i++) cells[i] = n[i] < 0.38 ? Cell.Ground : n[i] < 0.7 ? Cell.Grass : Cell.Rough;

  // hills (radio shadow)
  for (let k = 0; k < 4; k++) {
    const hx = width * (0.25 + 0.5 * rand()), hy = height * (0.15 + 0.7 * rand());
    blob(hx, hy, 8 + 6 * rand(), (x, y) => set(x, y, Cell.Hill));
  }
  // forests with trees
  for (let k = 0; k < 6; k++) {
    const fx = width * (0.15 + 0.7 * rand()), fy = height * rand();
    blob(fx, fy, 7 + 8 * rand(), (x, y) => set(x, y, rand() < 0.16 ? Cell.Tree : Cell.Forest));
  }
  // rock fields
  for (let k = 0; k < 5; k++) {
    const rx = width * (0.2 + 0.65 * rand()), ry = height * rand();
    blob(rx, ry, 4 + 5 * rand(), (x, y) => { if (rand() < 0.35) set(x, y, Cell.Rock); });
  }
  // mud patches
  for (let k = 0; k < 7; k++) {
    const mx = width * (0.2 + 0.65 * rand()), my = height * rand();
    blob(mx, my, 3 + 5 * rand(), (x, y) => set(x, y, Cell.Mud));
  }
  // ravine: an almost horizontal crack across the lower third, with one narrow pass (4 cells = 2 m)
  const ravY = Math.floor(height * 0.3);
  const passX = Math.floor(width * (0.35 + 0.1 * rand()));
  for (let x = Math.floor(width * 0.22); x < Math.floor(width * 0.58); x++) {
    const yy = ravY + Math.round(3 * Math.sin(x / 9));
    if (Math.abs(x - passX) <= 2) continue;
    for (let t = -1; t <= 1; t++) set(x, yy + t, Cell.Ravine);
  }
  // river: meanders top → bottom at ~65 % width, 3 fords
  const riverX = width * 0.65;
  const fords = [0.2, 0.52, 0.83].map((f) => Math.floor(height * (f + 0.06 * (rand() - 0.5))));
  for (let y = 0; y < height; y++) {
    const cx = Math.round(riverX + 7 * Math.sin(y / 14) + 3 * Math.sin(y / 5));
    const half = 2 + (y % 40 < 20 ? 1 : 0);
    const ford = fords.some((f) => Math.abs(y - f) <= 2);
    for (let x = cx - half; x <= cx + half; x++) set(x, y, ford ? Cell.Ford : Cell.Water);
  }
  // keep base, start and destination areas clear
  const clear = (p: { x: number; y: number }, r: number) =>
    blob(p.x / res, p.y / res, r / res, (x, y) => set(x, y, Cell.Ground));
  clear(w.base, 5);
  clear(w.start, 8);
  clear(w.destination, 7);
  return w;
}

/** Terrain as base64 for the browser (one byte per cell). */
export function encodeCells(w: World): string {
  return Buffer.from(w.cells).toString("base64");
}
