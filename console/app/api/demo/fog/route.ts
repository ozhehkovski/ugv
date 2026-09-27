import { NextResponse } from "next/server";
import { fleet } from "@/lib/fleet/manager";

export const dynamic = "force-dynamic";

/** What the operator base knows about the terrain (-1 unknown), as base64 Int8. */
export function GET(): NextResponse {
  const cells = fleet().fog();
  return cells ? NextResponse.json({ cells }) : NextResponse.json({ message: "демо не запущено" }, { status: 404 });
}
