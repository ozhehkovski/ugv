import { fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

/** The real robot's SLAM map image, proxied (the browser may not reach the robot directly). */
export async function GET(_req: Request, ctx: { params: Promise<{ id: string }> }): Promise<Response> {
  const { id } = await ctx.params;
  const r = fleet().real.get(id);
  if (!r) return new Response("unknown robot", { status: 404 });
  try {
    const up = await fetch(`${r.url}/api/map.png`, { signal: AbortSignal.timeout(3000), cache: "no-store" });
    return new Response(up.body, { status: up.status, headers: { "Content-Type": "image/png", "Cache-Control": "no-store" } });
  } catch (e) {
    return new Response(e instanceof Error ? e.message : String(e), { status: 502 });
  }
}
