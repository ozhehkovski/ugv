/** Data shared by the server (fleet manager) and the browser (console UI). */

export interface Pt {
  x: number;
  y: number;
}

export type RobotKind = "sim" | "real";

export interface RobotView {
  id: string;
  name: string;
  kind: RobotKind;
  role?: "relay" | "unit"; // swarm demo: relays hold the mesh chain back to the base
  /** what the operator knows (over the mesh for sim robots, over HTTP for real ones) */
  known: boolean;
  x: number;
  y: number;
  th: number;
  v: number;
  mode: "idle" | "auto" | "manual";
  arrived: boolean;
  battery: number | null;
  goal: Pt | null;
  path: Pt[];
  linkAge: number | null; // s since the last report reached the operator
  attention: string | null; // why the operator is needed (null = all fine)
  pending: string | null; // command not yet acknowledged
  /** ground truth (simulation only, for the "show truth" overlay) */
  truth?: { x: number; y: number; th: number };
  /** real robot extras */
  url?: string;
  map?: { resolution: number; width: number; height: number; origin: [number, number, number]; version: number } | null;
  camera?: boolean;
  estop?: boolean;
  status?: string;
}

export interface FleetEvent {
  t: number;
  robot?: string;
  text: string;
  level: "info" | "warn" | "alert";
}

export interface DemoInfo {
  running: boolean;
  paused: boolean;
  speed: number;
  t: number;
  robots: number;
  arrived: number;
  incidents: boolean;
  destination: Pt;
  base: Pt;
  links: [string, string][]; // current mesh links (base = "base")
  mesh: { sent: number; delivered: number; lost: number };
  knownCells: number;
}

export interface Snapshot {
  now: number;
  demo: DemoInfo | null;
  robots: RobotView[];
  events: FleetEvent[];
}

export interface WorldInfo {
  width: number;
  height: number;
  res: number;
  base: Pt;
  start: Pt;
  destination: Pt;
  cells: string; // base64, one byte per cell (lib/sim/world Cell)
}
