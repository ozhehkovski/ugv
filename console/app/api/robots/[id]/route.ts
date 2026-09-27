import { NextResponse } from "next/server";
import { fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

export async function DELETE(_req: Request, ctx: { params: Promise<{ id: string }> }): Promise<NextResponse> {
  const { id } = await ctx.params;
  fleet().removeReal(id);
  return NextResponse.json({ ok: true });
}
