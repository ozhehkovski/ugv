import { fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

/** The real robot's MJPEG camera stream, proxied. */
export async function GET(req: Request, ctx: { params: Promise<{ id: string }> }): Promise<Response> {
  const { id } = await ctx.params;
  const r = fleet().real.get(id);
  if (!r) return new Response("unknown robot", { status: 404 });
  try {
    const up = await fetch(`${r.url}/camera.mjpg`, { signal: req.signal, cache: "no-store" });
    return new Response(up.body, {
      status: up.status,
      headers: { "Content-Type": up.headers.get("content-type") ?? "multipart/x-mixed-replace; boundary=frame", "Cache-Control": "no-store" },
    });
  } catch (e) {
    return new Response(e instanceof Error ? e.message : String(e), { status: 502 });
  }
}
