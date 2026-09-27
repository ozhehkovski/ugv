import { NextResponse } from "next/server";
import { MAX_SPEED_UP, fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

interface DemoRequest {
  action: "start" | "stop" | "pause" | "resume" | "speed" | "to_destination" | "incident";
  robots?: number;
  seed?: number;
  incidents?: boolean;
  speed?: number;
}

export async function POST(req: Request): Promise<NextResponse> {
  let body: DemoRequest;
  try {
    body = (await req.json()) as DemoRequest;
  } catch {
    return NextResponse.json({ ok: false, message: "invalid JSON" }, { status: 400 });
  }
  const f = fleet();
  switch (body.action) {
    case "start": {
      const n = Math.max(1, Math.min(30, Math.round(body.robots ?? 10)));
      f.startDemo(n, Math.round(body.seed ?? 7), body.incidents ?? true);
      break;
    }
    case "stop": f.stopDemo(); break;
    case "pause": f.paused = true; break;
    case "resume": f.paused = false; break;
    case "speed": f.speed = Math.max(1, Math.min(MAX_SPEED_UP, Number(body.speed) || 1)); break;
    case "to_destination": {
      const sw = f.swarm;
      if (!sw) return NextResponse.json({ ok: false, message: "демо не запущено" }, { status: 409 });
      await f.command({ targets: sw.robots.map((r) => r.id), action: "goal", ...sw.world.destination });
      break;
    }
    case "incident": {
      const name = f.swarm?.injectIncident() ?? null;
      return NextResponse.json({ ok: !!name, message: name ? `${name}: застрял на скрытом препятствии` : "нет едущих роботов" });
    }
    default:
      return NextResponse.json({ ok: false, message: "unknown action" }, { status: 400 });
  }
  return NextResponse.json({ ok: true });
}
