import { fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";
export const runtime = "nodejs";

/** Server-sent events: a fleet snapshot 4 times per second. ?truth=1 adds simulation ground truth. */
export async function GET(req: Request): Promise<Response> {
  const truth = new URL(req.url).searchParams.get("truth") === "1";
  const enc = new TextEncoder();
  let timer: ReturnType<typeof setInterval> | undefined;
  const stream = new ReadableStream<Uint8Array>({
    start(controller) {
      const push = () => {
        try {
          controller.enqueue(enc.encode(`data: ${JSON.stringify(fleet().snapshot(truth))}\n\n`));
        } catch {
          clearInterval(timer);
        }
      };
      push();
      timer = setInterval(push, 250);
    },
    cancel() {
      clearInterval(timer);
    },
  });
  return new Response(stream, {
    headers: { "Content-Type": "text/event-stream", "Cache-Control": "no-store", Connection: "keep-alive" },
  });
}
