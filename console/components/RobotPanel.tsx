"use client";

import { useEffect, useRef, useState } from "react";
import type { RobotView } from "@/lib/fleet/types";
import { command } from "./Console";

const V_MAX = 0.5;
const W_MAX = 0.8;

/** Selected robot: status, take / release control, joystick + WASD, camera and map for real robots. */
export default function RobotPanel({ robot, onClose, onMessage }: { robot: RobotView; onClose: () => void; onMessage: (m: string) => void }) {
  const manual = robot.mode === "manual";
  const reachable = robot.linkAge !== null && robot.linkAge < 5;
  const joy = useRef<HTMLDivElement>(null);
  const knob = useRef<HTMLDivElement>(null);
  const cmd = useRef<{ v: number; w: number } | null>(null);
  const keys = useRef(new Set<string>());

  // teleop at 10 Hz while the joystick or keys are held (robot side times out in 0.3–0.6 s)
  useEffect(() => {
    const id = setInterval(() => {
      let c = cmd.current;
      if (!c && keys.current.size) {
        const k = keys.current;
        c = { v: (k.has("w") ? V_MAX : 0) - (k.has("s") ? V_MAX / 2 : 0), w: (k.has("a") ? W_MAX : 0) - (k.has("d") ? W_MAX : 0) };
      }
      if (c && manual) void command([robot.id], "teleop", { v: c.v, w: c.w });
    }, 100);
    return () => clearInterval(id);
  }, [robot.id, manual]);

  useEffect(() => {
    const map: Record<string, string> = { KeyW: "w", KeyS: "s", KeyA: "a", KeyD: "d", ArrowUp: "w", ArrowDown: "s", ArrowLeft: "a", ArrowRight: "d" };
    const down = (e: KeyboardEvent) => {
      if (!manual || !(e.code in map) || (e.target as HTMLElement).tagName === "INPUT") return;
      e.preventDefault();
      keys.current.add(map[e.code]);
    };
    const up = (e: KeyboardEvent) => { if (e.code in map) keys.current.delete(map[e.code]); };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    return () => { window.removeEventListener("keydown", down); window.removeEventListener("keyup", up); keys.current.clear(); };
  }, [manual]);

  const joyMove = (e: React.PointerEvent) => {
    const r = joy.current!.getBoundingClientRect(), R = r.width / 2;
    let dx = (e.clientX - r.left - R) / R, dy = (e.clientY - r.top - R) / R;
    const m = Math.hypot(dx, dy);
    if (m > 1) { dx /= m; dy /= m; }
    knob.current!.style.left = `${R + dx * R - 26}px`;
    knob.current!.style.top = `${R + dy * R - 26}px`;
    cmd.current = { v: -dy * V_MAX, w: -dx * W_MAX };
  };
  const joyEnd = () => {
    cmd.current = null;
    if (knob.current) { knob.current.style.left = "49px"; knob.current.style.top = "49px"; }
    if (manual) void command([robot.id], "teleop", { v: 0, w: 0 });
  };

  const act = async (action: string, text: string) => {
    const res = await command([robot.id], action);
    onMessage(res.ok ? `${robot.name}: ${text}` : `${robot.name}: ${res.message}`);
  };

  return (
    <div className="panel">
      <div className="row" style={{ justifyContent: "space-between" }}>
        <b style={{ fontSize: 14 }}>{robot.name}</b>
        <button onClick={onClose}>✕</button>
      </div>
      <div className="kv">
        <span>режим</span><b>{manual ? "ручное управление" : robot.mode === "auto" ? "авторежим" : "ожидает"}{robot.arrived ? " · на месте" : ""}</b>
        <span>связь</span><b>{robot.linkAge === null ? "нет" : robot.linkAge < 3 ? "есть" : `${Math.round(robot.linkAge)} с назад`}</b>
        <span>скорость</span><b>{(robot.v * 3.6).toFixed(1)} км/ч</b>
        {robot.battery !== null && (<><span>батарея</span><b>{Math.round(robot.battery)}%</b></>)}
        {robot.status && (<><span>состояние</span><b>{robot.status}</b></>)}
        {robot.attention && (<><span>внимание</span><b style={{ color: "var(--bad)" }}>{robot.attention}</b></>)}
      </div>
      {robot.kind === "real" && robot.camera && <img className="cam" alt="камера" src={`/api/real/${robot.id}/camera`} />}
      {robot.kind === "real" && <RealMap robot={robot} onMessage={onMessage} />}
      <div className="row">
        {!manual ? (
          <button className="warn" disabled={!reachable} onClick={() => void act("manual", "ручное управление")}>Взять управление</button>
        ) : (
          <button className="primary" onClick={() => void act("auto", "авторежим")}>Вернуть в авторежим</button>
        )}
        <button onClick={() => void act("stop", "стоп, цель снята")}>Стоп</button>
        {!reachable && <span className="stat">нет связи: команда дойдёт, когда робот вернётся в зону mesh</span>}
      </div>
      {manual && (
        <>
          <div ref={joy} className="joy" onPointerDown={(e) => { joy.current!.setPointerCapture(e.pointerId); joyMove(e); }}
               onPointerMove={(e) => { if (cmd.current) joyMove(e); }} onPointerUp={joyEnd} onPointerCancel={joyEnd}>
            <div ref={knob} className="knob" />
          </div>
          <div className="stat" style={{ textAlign: "center" }}>джойстик или W A S D · робот едет, пока держите</div>
        </>
      )}
    </div>
  );
}

/** A real robot's own SLAM map with its pose; click = navigation goal on that map. */
function RealMap({ robot, onMessage }: { robot: RobotView; onMessage: (m: string) => void }) {
  const cv = useRef<HTMLCanvasElement>(null);
  const [img, setImg] = useState<HTMLImageElement | null>(null);
  const version = robot.map?.version;

  useEffect(() => {
    if (version === undefined) return;
    const i = new Image();
    i.onload = () => setImg(i);
    i.src = `/api/real/${robot.id}/map?v=${version}`;
  }, [robot.id, version]);

  const fit = () => {
    const c = cv.current, m = robot.map;
    if (!c || !m) return null;
    const W = c.clientWidth, H = c.clientHeight;
    const s = Math.min(W / m.width, H / m.height);
    return { s, ox: (W - m.width * s) / 2, oy: (H - m.height * s) / 2, m };
  };

  useEffect(() => {
    const c = cv.current;
    const f = fit();
    if (!c || !f) return;
    c.width = c.clientWidth;
    c.height = c.clientHeight;
    const ctx = c.getContext("2d")!;
    ctx.fillStyle = "#11151a";
    ctx.fillRect(0, 0, c.width, c.height);
    if (img) { ctx.imageSmoothingEnabled = false; ctx.drawImage(img, f.ox, f.oy, f.m.width * f.s, f.m.height * f.s); }
    const px = (x: number, y: number): [number, number] => [
      f.ox + ((x - f.m.origin[0]) / f.m.resolution) * f.s,
      f.oy + (f.m.height - (y - f.m.origin[1]) / f.m.resolution) * f.s,
    ];
    if (robot.path.length) {
      ctx.strokeStyle = "#4c8dff"; ctx.setLineDash([5, 4]); ctx.beginPath();
      robot.path.forEach((p, i) => { const [a, b] = px(p.x, p.y); if (i) ctx.lineTo(a, b); else ctx.moveTo(a, b); });
      ctx.stroke(); ctx.setLineDash([]);
    }
    if (robot.known) {
      const [x, y] = px(robot.x, robot.y);
      ctx.fillStyle = "#2fbf71"; ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = "#2fbf71"; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y);
      ctx.lineTo(x + 14 * Math.cos(robot.th), y - 14 * Math.sin(robot.th)); ctx.stroke();
    }
  });

  if (!robot.map) return <div className="stat">карта робота ещё не получена</div>;
  return (
    <canvas ref={cv} className="realmap" title="клик — поехать сюда" onClick={async (e) => {
      const f = fit();
      if (!f) return;
      const r = cv.current!.getBoundingClientRect();
      const x = f.m.origin[0] + ((e.clientX - r.left - f.ox) / f.s) * f.m.resolution;
      const y = f.m.origin[1] + (f.m.height - (e.clientY - r.top - f.oy) / f.s) * f.m.resolution;
      const th = Math.atan2(y - robot.y, x - robot.x);
      const res = await command([robot.id], "goal", { x, y, th });
      onMessage(res.ok ? `${robot.name}: цель (${x.toFixed(1)}, ${y.toFixed(1)})` : `${robot.name}: ${res.message}`);
    }} />
  );
}
