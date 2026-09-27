/**
 * A real UGV: the console talks to the robot's own web panel API (ros2_ws/src/ugv_webui):
 *   GET /api/state · POST /api/goal · /api/goal/cancel · /api/teleop · /api/estop · map.png · camera.mjpg
 */
import type { RobotView } from "./types";

const POLL_MS = 400;
const TIMEOUT_MS = 1500;

interface RobotState {
  estop: boolean;
  source: string;
  safety: string;
  drive: { level: number | null; message: string };
  drive_age: number | null;
  pose: { x: number; y: number; th: number } | null;
  pose_frame: string | null;
  odom: { v: number; w: number };
  map: { version: number; resolution: number; width: number; height: number; origin: [number, number, number] } | null;
  map_status: string;
  nav: { state: string; goal: [number, number, number] | null; remaining: number | null; message: string };
  plan: [number, number][];
  explore: string;
  follow?: { state?: string; enabled?: boolean };
  camera_age: number | null;
  wall_event?: string;
}

export class RealRobot {
  state: RobotState | null = null;
  lastOk = 0;
  error: string | null = null;
  private timer: ReturnType<typeof setInterval>;
  private busy = false;

  constructor(readonly id: string, readonly name: string, readonly url: string) {
    this.timer = setInterval(() => void this.poll(), POLL_MS);
    void this.poll();
  }

  dispose(): void {
    clearInterval(this.timer);
  }

  private async poll(): Promise<void> {
    if (this.busy) return;
    this.busy = true;
    try {
      const r = await fetch(`${this.url}/api/state`, { signal: AbortSignal.timeout(TIMEOUT_MS), cache: "no-store" });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      this.state = (await r.json()) as RobotState;
      this.lastOk = Date.now();
      this.error = null;
    } catch (e) {
      this.error = e instanceof Error ? e.message : String(e);
    } finally {
      this.busy = false;
    }
  }

  async post(path: string, body: unknown): Promise<{ ok: boolean; message: string }> {
    try {
      const r = await fetch(`${this.url}${path}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
        signal: AbortSignal.timeout(path === "/api/goal" ? 15000 : TIMEOUT_MS),
      });
      const data = (await r.json().catch(() => ({}))) as { success?: boolean; ok?: boolean; message?: string; error?: string };
      const ok = r.ok && (data.success ?? data.ok ?? true);
      return { ok, message: data.message ?? data.error ?? (ok ? "ok" : `HTTP ${r.status}`) };
    } catch (e) {
      return { ok: false, message: e instanceof Error ? e.message : String(e) };
    }
  }

  goal(x: number, y: number, th: number) { return this.post("/api/goal", { pose: [x, y, th] }); }
  cancel() { return this.post("/api/goal/cancel", {}); }
  estop(on: boolean) { return this.post("/api/estop", { engage: on }); }
  teleop(v: number, w: number) { return this.post("/api/teleop", { v, w }); }

  view(): RobotView {
    const s = this.state;
    const age = this.lastOk ? (Date.now() - this.lastOk) / 1000 : null;
    const online = age !== null && age < 3;
    let attention: string | null = null;
    if (!online) attention = "нет связи с роботом";
    else if (s?.estop) attention = "аварийный стоп";
    else if (s?.map_status?.split("|")[3]?.includes("relocalize")) attention = "робота переносили — укажите позицию";
    else if (s?.nav.state === "aborted") attention = "не смог доехать до цели";
    else if (s?.drive.level !== undefined && s?.drive.level !== null && s.drive.level >= 2) attention = `привод: ${s.drive.message}`;
    else if (s?.follow?.state === "lost") attention = "потерял человека";
    const moving = s?.nav.state === "active" || s?.explore?.startsWith("exploring") || !!s?.follow?.enabled;
    const manual = s?.source === "teleop";
    return {
      id: this.id,
      name: this.name,
      kind: "real",
      known: !!s?.pose,
      x: s?.pose?.x ?? 0,
      y: s?.pose?.y ?? 0,
      th: s?.pose?.th ?? 0,
      v: s?.odom.v ?? 0,
      mode: manual ? "manual" : moving ? "auto" : "idle",
      arrived: s?.nav.state === "succeeded",
      battery: null,
      goal: s?.nav.goal ? { x: s.nav.goal[0], y: s.nav.goal[1] } : null,
      path: (s?.plan ?? []).map(([x, y]) => ({ x, y })),
      linkAge: age,
      attention,
      pending: null,
      url: this.url,
      map: s?.map ?? null,
      camera: s?.camera_age !== null && s?.camera_age !== undefined && s.camera_age < 2,
      estop: s?.estop ?? false,
      status: s ? `${s.drive.message} · ${s.safety.split(" ")[0]} · nav ${s.nav.state}` : this.error ?? "…",
    };
  }
}
