import { NextResponse } from "next/server";
import { fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

/** Add a real robot by the address of its web panel, e.g. http://192.168.1.58:8090 */
export async function POST(req: Request): Promise<NextResponse> {
  let body: { name?: string; url?: string };
  try {
    body = (await req.json()) as { name?: string; url?: string };
  } catch {
    return NextResponse.json({ ok: false, message: "invalid JSON" }, { status: 400 });
  }
  try {
    const r = fleet().addReal(String(body.name ?? "").slice(0, 40), String(body.url ?? ""));
    return NextResponse.json({ ok: true, id: r.id });
  } catch (e) {
    return NextResponse.json({ ok: false, message: e instanceof Error ? e.message : String(e) }, { status: 400 });
  }
}
