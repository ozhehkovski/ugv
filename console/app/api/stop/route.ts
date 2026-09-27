import { NextResponse } from "next/server";
import { fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

export async function POST(): Promise<NextResponse> {
  await fleet().stopAll();
  return NextResponse.json({ ok: true });
}
