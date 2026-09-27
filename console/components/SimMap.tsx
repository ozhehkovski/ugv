"use client";

import { useCallback, useEffect, useRef } from "react";
import type { RobotView, Snapshot, WorldInfo } from "@/lib/fleet/types";
import { CELL_INFO, Cell } from "@/lib/sim/world";

export type MapMode = "select" | "goal";

interface Props {
  world: WorldInfo;
  cells: Uint8Array;
  fog: Int8Array | null;
  showFog: boolean;
  showTruth: boolean;
  snap: Snapshot;
  selected: Set<string>;
  focusId: string | null;
  mode: MapMode;
  onSelect: (ids: string[], additive: boolean) => void;
  onGoal: (x: number, y: number) => void;
}

interface View {
  scale: number; // px per metre
  ox: number; // screen px of world x = 0
  oy: number; // screen px of world y = 0 (y grows up in the world, down on screen)
}

const hex = (c: string): [number, number, number] => [parseInt(c.slice(1, 3), 16), parseInt(c.slice(3, 5), 16), parseInt(c.slice(5, 7), 16)];

export function robotColor(r: RobotView, now: number): string {
  if (r.attention && r.attention !== "нет связи с роботом") return Math.floor(now / 400) % 2 ? "#e5484d" : "#ff8a8d";
  if (r.linkAge === null || r.linkAge > 15) return "#6b7480";
  if (r.mode === "manual") return "#f0b429";
  if (r.arrived) return "#2fbf71";
  if (r.mode === "auto") return "#4c8dff";
  return "#b8c2cf";
}

export default function SimMap(props: Props) {
  const { world, cells, fog, showFog, showTruth, snap, selected, focusId, mode, onSelect, onGoal } = props;
  const canvas = useRef<HTMLCanvasElement>(null);
  const terrain = useRef<HTMLCanvasElement | null>(null);
  const fogImg = useRef<HTMLCanvasElement | null>(null);
  const view = useRef<View | null>(null);
  const drag = useRef<{ x: number; y: number; moved: boolean; box: boolean; cx: number; cy: number } | null>(null);
  const latest = useRef(props);
  latest.current = props;

  // terrain → offscreen image (1 px per cell), once per world
  useEffect(() => {
    const c = document.createElement("canvas");
    c.width = world.width;
    c.height = world.height;
    const ctx = c.getContext("2d")!;
    const img = ctx.createImageData(world.width, world.height);
    const lut = Object.values(Cell).filter((v) => typeof v === "number").map((v) => hex(CELL_INFO[v as Cell].color));
    for (let y = 0; y < world.height; y++)
      for (let x = 0; x < world.width; x++) {
        const [r, g, b] = lut[cells[y * world.width + x]] ?? [255, 0, 255];
        const o = ((world.height - 1 - y) * world.width + x) * 4; // flip: row 0 = y 0 at the bottom
        img.data[o] = r; img.data[o + 1] = g; img.data[o + 2] = b; img.data[o + 3] = 255;
      }
    ctx.putImageData(img, 0, 0);
    terrain.current = c;
    view.current = null; // refit
  }, [world, cells]);

  // fog of war: what the base does not know
  useEffect(() => {
    if (!fog) { fogImg.current = null; return; }
    const c = document.createElement("canvas");
    c.width = world.width;
    c.height = world.height;
    const ctx = c.getContext("2d")!;
    const img = ctx.createImageData(world.width, world.height);
    for (let y = 0; y < world.height; y++)
      for (let x = 0; x < world.width; x++) {
        if (fog[y * world.width + x] !== -1) continue;
        const o = ((world.height - 1 - y) * world.width + x) * 4;
        img.data[o] = 8; img.data[o + 1] = 10; img.data[o + 2] = 14; img.data[o + 3] = 170;
      }
    ctx.putImageData(img, 0, 0);
    fogImg.current = c;
  }, [fog, world]);

  const toScreen = (v: View, x: number, y: number): [number, number] => [v.ox + x * v.scale, v.oy - y * v.scale];
  const toWorld = (v: View, sx: number, sy: number): [number, number] => [(sx - v.ox) / v.scale, (v.oy - sy) / v.scale];

  const draw = useCallback(() => {
    const cv = canvas.current;
    if (!cv) return;
    const p = latest.current;
    const rect = cv.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(rect.width * dpr) || cv.height !== Math.round(rect.height * dpr)) {
      cv.width = Math.round(rect.width * dpr);
      cv.height = Math.round(rect.height * dpr);
    }
    const ctx = cv.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = rect.width, H = rect.height;
    const wm = p.world.width * p.world.res, hm = p.world.height * p.world.res;
    if (!view.current) {
      const scale = Math.min(W / wm, H / hm) * 0.96;
      view.current = { scale, ox: (W - wm * scale) / 2, oy: (H + hm * scale) / 2 };
    }
    const v = view.current;
    ctx.fillStyle = "#0b0e12";
    ctx.fillRect(0, 0, W, H);
    ctx.imageSmoothingEnabled = false;
    const [tx, ty] = toScreen(v, 0, hm);
    if (terrain.current) ctx.drawImage(terrain.current, tx, ty, wm * v.scale, hm * v.scale);
    if (p.showFog && fogImg.current) ctx.drawImage(fogImg.current, tx, ty, wm * v.scale, hm * v.scale);

    const demo = p.snap.demo;
    const sims = p.snap.robots.filter((r) => r.kind === "sim");
    const pos = new Map<string, [number, number]>(sims.map((r) => [r.id, [r.x, r.y]]));
    if (demo) pos.set("base", [demo.base.x, demo.base.y]);
    // mesh links
    if (demo) {
      ctx.strokeStyle = "rgba(94, 234, 212, 0.45)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (const [a, b] of demo.links) {
        const pa = pos.get(a), pb = pos.get(b);
        if (!pa || !pb) continue;
        const [x1, y1] = toScreen(v, pa[0], pa[1]), [x2, y2] = toScreen(v, pb[0], pb[1]);
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
      }
      ctx.stroke();
      // base station
      const [bx, by] = toScreen(v, demo.base.x, demo.base.y);
      ctx.fillStyle = "#5eead4";
      ctx.beginPath(); ctx.moveTo(bx, by - 12); ctx.lineTo(bx - 8, by + 7); ctx.lineTo(bx + 8, by + 7); ctx.closePath(); ctx.fill();
      ctx.fillStyle = "#e6e9ee"; ctx.font = "11px sans-serif"; ctx.fillText("база оператора", bx + 11, by + 4);
      // destination
      const [dx, dy] = toScreen(v, demo.destination.x, demo.destination.y);
      ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2; ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.arc(dx, dy, 7 * v.scale, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = "#ffffff"; ctx.fillText("точка назначения", dx + 7 * v.scale + 4, dy + 4);
    }
    // paths + goals
    for (const r of sims) {
      const sel = p.selected.has(r.id) || p.focusId === r.id;
      if (sel && r.path.length) {
        ctx.strokeStyle = "rgba(76,141,255,0.85)"; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
        ctx.beginPath();
        const [sx, sy] = toScreen(v, r.x, r.y);
        ctx.moveTo(sx, sy);
        for (const q of r.path) { const [a, b] = toScreen(v, q.x, q.y); ctx.lineTo(a, b); }
        ctx.stroke(); ctx.setLineDash([]);
      }
      if (r.goal && !r.arrived) {
        const [gx, gy] = toScreen(v, r.goal.x, r.goal.y);
        ctx.strokeStyle = sel ? "#4c8dff" : "rgba(255,255,255,0.35)"; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(gx - 4, gy - 4); ctx.lineTo(gx + 4, gy + 4); ctx.moveTo(gx + 4, gy - 4); ctx.lineTo(gx - 4, gy + 4); ctx.stroke();
      }
    }
    // robots
    const now = Date.now();
    const size = Math.max(7, 0.62 * v.scale);
    for (const r of sims) {
      if (p.showTruth && r.truth) {
        const [gx, gy] = toScreen(v, r.truth.x, r.truth.y);
        ctx.strokeStyle = "rgba(255,255,255,0.5)"; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(gx, gy, size * 0.7, 0, Math.PI * 2); ctx.stroke();
      }
      if (!r.known) continue;
      const [sx, sy] = toScreen(v, r.x, r.y);
      const col = robotColor(r, now);
      ctx.save();
      ctx.translate(sx, sy);
      ctx.rotate(-r.th);
      ctx.fillStyle = col;
      ctx.beginPath(); ctx.moveTo(size, 0); ctx.lineTo(-size * 0.7, size * 0.6); ctx.lineTo(-size * 0.4, 0); ctx.lineTo(-size * 0.7, -size * 0.6); ctx.closePath(); ctx.fill();
      ctx.restore();
      if (r.role === "relay") {            // relay: an antenna ring
        ctx.strokeStyle = "#5eead4"; ctx.lineWidth = 1.5; ctx.setLineDash([2, 3]);
        ctx.beginPath(); ctx.arc(sx, sy, size * 2.2, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
      }
      if (p.selected.has(r.id) || p.focusId === r.id) {
        ctx.strokeStyle = "#ffffff"; ctx.lineWidth = p.focusId === r.id ? 2.5 : 1.5;
        ctx.beginPath(); ctx.arc(sx, sy, size * 1.35, 0, Math.PI * 2); ctx.stroke();
      }
      if (r.attention && r.attention !== "нет связи с роботом") {
        ctx.strokeStyle = "rgba(229,72,77,0.8)"; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(sx, sy, size * (1.8 + 0.4 * Math.sin(now / 200)), 0, Math.PI * 2); ctx.stroke();
      }
      ctx.fillStyle = "#e6e9ee"; ctx.font = "11px sans-serif";
      ctx.fillText(r.name.replace("UGV-", ""), sx + size + 2, sy - size);
    }
    // selection box
    const d = drag.current;
    if (d?.box && d.moved) {
      ctx.strokeStyle = "#4c8dff"; ctx.lineWidth = 1; ctx.setLineDash([4, 3]);
      ctx.strokeRect(Math.min(d.x, d.cx), Math.min(d.y, d.cy), Math.abs(d.cx - d.x), Math.abs(d.cy - d.y));
      ctx.setLineDash([]);
    }
  }, []);

  // redraw on every animation frame (robots pulse, snapshots arrive 4×/s)
  useEffect(() => {
    let raf = 0;
    const loop = () => { draw(); raf = requestAnimationFrame(loop); };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, [draw]);

  const local = (e: React.PointerEvent | React.WheelEvent): [number, number] => {
    const r = canvas.current!.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  };

  const hitRobot = (sx: number, sy: number): string | null => {
    const v = view.current;
    if (!v) return null;
    let best: string | null = null, bestD = 16;
    for (const r of latest.current.snap.robots) {
      if (r.kind !== "sim" || !r.known) continue;
      const [x, y] = toScreen(v, r.x, r.y);
      const d = Math.hypot(x - sx, y - sy);
      if (d < bestD) { bestD = d; best = r.id; }
    }
    return best;
  };

  return (
    <canvas
      ref={canvas}
      style={{ cursor: mode === "goal" ? "crosshair" : "grab" }}
      onPointerDown={(e) => {
        const [x, y] = local(e);
        drag.current = { x, y, cx: x, cy: y, moved: false, box: e.shiftKey && mode === "select" };
        (e.target as HTMLCanvasElement).setPointerCapture(e.pointerId);
      }}
      onPointerMove={(e) => {
        const d = drag.current;
        if (!d) return;
        const [x, y] = local(e);
        if (Math.hypot(x - d.x, y - d.y) > 5) d.moved = true;
        if (d.moved && !d.box && view.current) {
          view.current.ox += x - d.cx;
          view.current.oy += y - d.cy;
        }
        d.cx = x; d.cy = y;
      }}
      onPointerUp={(e) => {
        const d = drag.current;
        drag.current = null;
        if (!d || !view.current) return;
        const [x, y] = local(e);
        if (d.box && d.moved) {
          const v = view.current;
          const ids = latest.current.snap.robots.filter((r) => {
            if (r.kind !== "sim" || !r.known) return false;
            const [sx, sy] = toScreen(v, r.x, r.y);
            return sx >= Math.min(d.x, x) && sx <= Math.max(d.x, x) && sy >= Math.min(d.y, y) && sy <= Math.max(d.y, y);
          }).map((r) => r.id);
          onSelect(ids, e.shiftKey);
          return;
        }
        if (d.moved) return;
        if (mode === "goal") {
          const [wx, wy] = toWorld(view.current, x, y);
          onGoal(wx, wy);
          return;
        }
        const hit = hitRobot(x, y);
        onSelect(hit ? [hit] : [], e.shiftKey);
      }}
      onWheel={(e) => {
        const v = view.current;
        if (!v) return;
        const [x, y] = local(e);
        const k = e.deltaY < 0 ? 1.15 : 1 / 1.15;
        v.ox = x - (x - v.ox) * k;
        v.oy = y - (y - v.oy) * k;
        v.scale *= k;
      }}
    />
  );
}
