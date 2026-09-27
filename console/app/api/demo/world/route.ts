import { NextResponse } from "next/server";
import { fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

export function GET(): NextResponse {
  const w = fleet().worldInfo();
  return w ? NextResponse.json(w) : NextResponse.json({ message: "демо не запущено" }, { status: 404 });
}
