"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { RobotView, Snapshot, WorldInfo } from "@/lib/fleet/types";
import SimMap, { MapMode, robotColor } from "./SimMap";
import RobotPanel from "./RobotPanel";

async function api(path: string, body?: unknown): Promise<{ ok: boolean; message?: string }> {
  try {
    const r = await fetch(path, {
      method: body === undefined ? "GET" : "POST",
      headers: { "Content-Type": "application/json" },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
    return (await r.json().catch(() => ({ ok: r.ok }))) as { ok: boolean; message?: string };
  } catch (e) {
    return { ok: false, message: e instanceof Error ? e.message : String(e) };
  }
}

export const command = (targets: string[], action: string, extra: Record<string, number> = {}) =>
  api("/api/command", { targets, action, ...extra });

const decode = (b64: string): Uint8Array => Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));

function fmtTime(t: number): string {
  const m = Math.floor(t / 60), s = Math.floor(t % 60);
  return `${m}:${String(s).padStart(2, "0")}`;
}

function badge(r: RobotView): { text: string; cls: string } {
  if (r.attention) return { text: r.attention, cls: "bad" };
  if (r.mode === "manual") return { text: "ручное", cls: "manual" };
  if (r.arrived) return { text: "на месте", cls: "ok" };
  if (r.mode === "auto") return { text: "авто", cls: "ok" };
  return { text: "ожидает", cls: "" };
}

export default function Console() {
  const [snap, setSnap] = useState<Snapshot | null>(null);
  const [online, setOnline] = useState(false);
  const [world, setWorld] = useState<{ info: WorldInfo; cells: Uint8Array } | null>(null);
  const [fog, setFog] = useState<Int8Array | null>(null);
  const [showFog, setShowFog] = useState(true);
  const [showTruth, setShowTruth] = useState(false);
  const [selected, setSelected] = useState<Set<string>>(new Set());
  const [focusId, setFocusId] = useState<string | null>(null);
  const [mode, setMode] = useState<MapMode>("select");
  const [msg, setMsg] = useState("");
  const addDlg = useRef<HTMLDialogElement>(null);
  const seenAlerts = useRef(new Set<string>());

  // live fleet state
  useEffect(() => {
    const es = new EventSource(`/api/stream${showTruth ? "?truth=1" : ""}`);
    es.onmessage = (e) => { setSnap(JSON.parse(e.data) as Snapshot); setOnline(true); };
    es.onerror = () => setOnline(false);
    return () => es.close();
  }, [showTruth]);

  // terrain once per demo run; fog of war every 1.5 s
  const demoKey = snap?.demo ? `${snap.demo.robots}` : null;
  useEffect(() => {
    if (!demoKey) { setWorld(null); return; }
    void fetch("/api/demo/world").then(async (r) => {
      if (!r.ok) return;
      const info = (await r.json()) as WorldInfo;
      setWorld({ info, cells: decode(info.cells) });
    });
  }, [demoKey, snap?.demo?.destination.x]);
  useEffect(() => {
    if (!world || !showFog) return;
    const load = () => void fetch("/api/demo/fog").then(async (r) => {
      if (r.ok) setFog(new Int8Array(decode(((await r.json()) as { cells: string }).cells).buffer));
    });
    load();
    const id = setInterval(load, 1500);
    return () => clearInterval(id);
  }, [world, showFog]);

  const robots = snap?.robots ?? [];
  // the queue holds what the operator can act on; robots out of radio reach are summarised
  const attention = robots.filter((r) => r.attention && r.attention !== "нет связи с роботом");
  const offline = robots.filter((r) => r.attention === "нет связи с роботом");
  const focus = robots.find((r) => r.id === focusId) ?? null;

  // a new robot needing help: beep once and put it in focus if nothing is focused
  useEffect(() => {
    for (const r of attention) {
      const key = `${r.id}:${r.attention}`;
      if (seenAlerts.current.has(key)) continue;
      seenAlerts.current.add(key);
      beep();
      if (!focusId) setFocusId(r.id);
    }
    for (const key of [...seenAlerts.current]) {
      const [id, why] = key.split(/:(.*)/s);
      if (!robots.some((r) => r.id === id && r.attention === why)) seenAlerts.current.delete(key);
    }
  }, [attention, focusId, robots]);

  const select = useCallback((ids: string[], additive: boolean) => {
    setSelected((prev) => {
      const next = new Set(additive ? prev : []);
      for (const id of ids) {
        if (additive && next.has(id)) next.delete(id);
        else next.add(id);
      }
      return next;
    });
    if (ids.length === 1) setFocusId(ids[0]);
  }, []);

  const goalTargets = (): string[] => {
    const sim = robots.filter((r) => r.kind === "sim");
    if (selected.size) return [...selected].filter((id) => id.startsWith("sim-"));
    return sim.map((r) => r.id);
  };

  const onGoal = useCallback(async (x: number, y: number) => {
    const targets = goalTargets();
    setMode("select");
    const res = await command(targets, "goal", { x, y });
    setMsg(res.ok ? `цель → ${targets.length} робот(ам)` : `ошибка: ${res.message}`);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selected, robots]);

  const demo = snap?.demo ?? null;

  return (
    <div className="app">
      <header className="topbar">
        <h1>UGV · пульт роя</h1>
        <span className="stat">{online ? "сервер ok" : <b style={{ color: "var(--bad)" }}>нет связи с пультом</b>}</span>
        {!demo ? (
          <button className="primary" onClick={() => void api("/api/demo", { action: "start", robots: 10, incidents: true })}>
            Демо: рой из 10 роботов
          </button>
        ) : (
          <>
            <button className="primary" onClick={() => void api("/api/demo", { action: "to_destination" })}>Рой → к точке назначения</button>
            <button onClick={() => void api("/api/demo", { action: demo.paused ? "resume" : "pause" })}>{demo.paused ? "▶ продолжить" : "⏸ пауза"}</button>
            <span className="seg">
              {[1, 5, 10, 20].map((s) => (
                <button key={s} className={demo.speed === s ? "on" : ""} onClick={() => void api("/api/demo", { action: "speed", speed: s })}>{s}×</button>
              ))}
            </span>
            <span className="stat">время <b>{fmtTime(demo.t)}</b></span>
            <span className="stat">на месте <b>{demo.arrived}/{demo.robots}</b></span>
            <span className="stat">mesh <b>{demo.links.length}</b> связей · потери {demo.mesh.sent ? Math.round((100 * demo.mesh.lost) / Math.max(1, demo.mesh.delivered + demo.mesh.lost)) : 0}%</span>
            <button title="Для показа: случайный едущий робот застрянет на скрытом препятствии"
                    onClick={async () => { const r = await api("/api/demo", { action: "incident" }); setMsg(r.message ?? ""); }}>
              Подкинуть проблему
            </button>
            <button onClick={() => { if (confirm("Остановить демо?")) void api("/api/demo", { action: "stop" }); }}>Завершить демо</button>
          </>
        )}
        <div className="spacer" />
        <button onClick={() => addDlg.current?.showModal()}>+ Добавить робота</button>
        <button className="danger" onClick={() => void api("/api/stop", {})}>СТОП ВСЕМ</button>
      </header>

      <div className="main">
        <div className="mapwrap">
          {world && snap ? (
            <>
              <SimMap world={world.info} cells={world.cells} fog={fog} showFog={showFog} showTruth={showTruth} snap={snap}
                      selected={selected} focusId={focusId} mode={mode} onSelect={select} onGoal={(x, y) => void onGoal(x, y)} />
              <div className="maptools">
                <button className={mode === "goal" ? "on" : ""} onClick={() => setMode(mode === "goal" ? "select" : "goal")}>
                  {mode === "goal" ? "Отмена" : `Цель ${selected.size ? `(${selected.size})` : "(всем)"}`}
                </button>
                <button className={showFog ? "on" : ""} onClick={() => setShowFog(!showFog)}>Туман: что знает база</button>
                <button className={showTruth ? "on" : ""} onClick={() => setShowTruth(!showTruth)}>Истинные позиции</button>
                <button onClick={() => setSelected(new Set(robots.filter((r) => r.kind === "sim").map((r) => r.id)))}>Выбрать всех</button>
              </div>
              {mode === "goal" && <div className="maphint">Кликните по карте: цель для {selected.size ? `${selected.size} выбранных` : "всего роя"}</div>}
              <div className="legend">синий — авто · жёлтый — ручное · зелёный — на месте · красный — нужен оператор · серый — нет связи ·
                бирюзовые линии — mesh, пунктирное кольцо — ретранслятор · клик — выбрать, Shift+рамка — группа, колесо — масштаб</div>
            </>
          ) : (
            <div className="empty">
              <h2>Демо не запущено</h2>
              <div>Рой из 10 роботов на пересечённой местности: лес, камни, болото, река с бродами, овраг, холмы без связи.</div>
              <button className="primary" onClick={() => void api("/api/demo", { action: "start", robots: 10, incidents: true })}>Запустить демо</button>
              {robots.some((r) => r.kind === "real") && <div>Настоящие роботы — в списке справа.</div>}
            </div>
          )}
        </div>

        <aside className="side">
          <div className="section">
            <h3>Нужен оператор ({attention.length})</h3>
            {attention.length === 0 && <div className="stat">все роботы справляются сами</div>}
            {offline.length > 0 && (
              <div className="stat" style={{ marginBottom: 6 }}>
                вне связи: <b>{offline.map((r) => r.name).join(", ")}</b> — позиции на карте последние известные
              </div>
            )}
            {attention.map((r) => (
              <div className="attention" key={r.id} onClick={() => setFocusId(r.id)}>
                <div>
                  <b>{r.name}</b>
                  <div className="why">{r.attention}</div>
                </div>
                {r.attention !== "нет связи с роботом" && r.mode !== "manual" && (
                  <button className="warn" onClick={(e) => { e.stopPropagation(); setFocusId(r.id); void command([r.id], "manual"); }}>
                    Взять управление
                  </button>
                )}
              </div>
            ))}
          </div>
          <div className="section scroll list">
            <h3>Роботы ({robots.length})</h3>
            {robots.map((r) => {
              const b = badge(r);
              return (
                <div key={r.id} className={`card ${focusId === r.id ? "focus" : ""}`}
                     onClick={(e) => { select([r.id], e.shiftKey); setFocusId(r.id); }}>
                  <span className="dot" style={{ background: robotColor(r, 0) }} />
                  <span className="name">{r.name}{r.role === "relay" ? " · ретранслятор" : ""}{selected.has(r.id) ? " ✓" : ""}</span>
                  <span className={`badge ${b.cls}`}>{b.text}</span>
                  <span className="sub">
                    {r.kind === "real" ? "настоящий · " : ""}
                    {r.linkAge === null ? "ещё не выходил на связь" : r.linkAge > 3 ? `связь ${Math.round(r.linkAge)} с назад` : "на связи"}
                    {r.battery !== null ? ` · ${Math.round(r.battery)}%` : ""}
                    {r.pending ? ` · команда «${r.pending}» в пути` : ""}
                  </span>
                </div>
              );
            })}
          </div>
          {focus && <RobotPanel robot={focus} onClose={() => setFocusId(null)} onMessage={setMsg} />}
          <div className="section scroll events">
            <h3>События {msg && <span style={{ textTransform: "none", color: "var(--accent)" }}> · {msg}</span>}</h3>
            {[...(snap?.events ?? [])].reverse().map((e, i) => (
              <div key={i} className={`ev ${e.level}`}><span className="t">{fmtTime(e.t)}</span>{e.text}</div>
            ))}
          </div>
        </aside>
      </div>

      <dialog ref={addDlg}>
        <form method="dialog" onSubmit={async (e) => {
          const f = new FormData(e.currentTarget);
          const res = await api("/api/robots", { name: f.get("name"), url: f.get("url") });
          setMsg(res.ok ? "робот добавлен" : `ошибка: ${res.message}`);
        }}>
          <h3 style={{ margin: 0 }}>Добавить настоящего робота</h3>
          <label>Название<br /><input name="name" defaultValue="UGV-Jetson" style={{ width: "100%" }} /></label>
          <label>Адрес веб-панели робота<br /><input name="url" defaultValue="http://192.168.1.58:8090" style={{ width: "100%" }} /></label>
          <div className="row" style={{ display: "flex", gap: 8, justifyContent: "flex-end" }}>
            <button value="cancel" formNoValidate>Отмена</button>
            <button className="primary" value="ok">Добавить</button>
          </div>
        </form>
      </dialog>
    </div>
  );
}

let audio: AudioContext | null = null;
function beep(): void {
  try {
    audio ??= new AudioContext();
    const o = audio.createOscillator(), g = audio.createGain();
    o.frequency.value = 880;
    g.gain.value = 0.05;
    o.connect(g).connect(audio.destination);
    o.start();
    o.stop(audio.currentTime + 0.15);
  } catch {
    /* audio is optional (autoplay policy) */
  }
}
