import { NextResponse } from "next/server";
import { CommandRequest, fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

const ACTIONS = new Set(["goal", "manual", "auto", "teleop", "stop"]);

export async function POST(req: Request): Promise<NextResponse> {
  let body: CommandRequest;
  try {
    body = (await req.json()) as CommandRequest;
  } catch {
    return NextResponse.json({ ok: false, message: "invalid JSON" }, { status: 400 });
  }
  if (!Array.isArray(body.targets) || !ACTIONS.has(body.action)) {
    return NextResponse.json({ ok: false, message: "targets[] and a valid action are required" }, { status: 400 });
  }
  for (const k of ["x", "y", "th", "v", "w"] as const) {
    if (body[k] !== undefined && !Number.isFinite(body[k])) {
      return NextResponse.json({ ok: false, message: `${k} must be a number` }, { status: 400 });
    }
  }
  const res = await fleet().command(body);
  return NextResponse.json(res, { status: res.ok ? 200 : 409 });
}
