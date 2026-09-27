import { describe, expect, it } from "vitest";
import { Knowledge } from "@/lib/sim/knowledge";
import { Mesh, Packet } from "@/lib/sim/mesh";
import { aStar, costGrid } from "@/lib/sim/planner";
import { Swarm, hexSlots, relayPoints } from "@/lib/sim/swarm";
import { Cell, World, cellAt, generateWorld, idx, passable } from "@/lib/sim/world";

function fullKnowledge(w: World): Knowledge {
  const k = new Knowledge(w.width, w.height);
  w.cells.forEach((c, i) => k.learn(i, c as Cell));
  return k;
}
const cellIndex = (w: World, p: { x: number; y: number }) => idx(w, Math.floor(p.x / w.res), Math.floor(p.y / w.res));

describe("world", () => {
  it("is deterministic and keeps start, base and destination clear", () => {
    const a = generateWorld(7), b = generateWorld(7), c = generateWorld(8);
    expect(Buffer.from(a.cells).equals(Buffer.from(b.cells))).toBe(true);
    expect(Buffer.from(a.cells).equals(Buffer.from(c.cells))).toBe(false);
    for (const p of [a.base, a.start, a.destination]) expect(passable(cellAt(a, p.x, p.y)!)).toBe(true);
  });

  it("has a river, a ravine and still a way through (fords, the pass)", () => {
    const w = generateWorld(7);
    const counts = new Map<number, number>();
    w.cells.forEach((c) => counts.set(c, (counts.get(c) ?? 0) + 1));
    for (const c of [Cell.Water, Cell.Ford, Cell.Ravine, Cell.Mud, Cell.Tree, Cell.Hill]) expect(counts.get(c) ?? 0).toBeGreaterThan(0);
    const path = aStar(costGrid(fullKnowledge(w), 1), w.width, w.height, cellIndex(w, w.start), cellIndex(w, w.destination));
    expect(path).not.toBeNull();
    const crosses = path!.some((i) => w.cells[i] === Cell.Ford);
    expect(crosses).toBe(true);
  });
});

describe("mesh", () => {
  const flat = (): World => {
    const w = generateWorld(1, 200, 20);
    w.cells.fill(Cell.Ground);
    return w;
  };

  it("links by range, hills cut the range", () => {
    const w = flat();
    const m = new Mesh(w, { range: 30, hillPenalty: 4, loss: 0, ttl: 8 });
    expect(m.linked({ id: "a", x: 5, y: 5 }, { id: "b", x: 30, y: 5 })).toBe(true);
    expect(m.linked({ id: "a", x: 5, y: 5 }, { id: "b", x: 40, y: 5 })).toBe(false);
    for (let x = 30; x < 42; x++) w.cells[idx(w, x, 10)] = Cell.Hill; // a 6 m hill between them
    expect(m.linked({ id: "a", x: 5, y: 5 }, { id: "b", x: 30, y: 5 })).toBe(false);
  });

  it("relays across hops, respects dst and never delivers twice", () => {
    const w = flat();
    const m = new Mesh(w, { range: 30, hillPenalty: 4, loss: 0, ttl: 8 });
    const nodes = [{ id: "base", x: 5, y: 5 }, { id: "relay", x: 30, y: 5 }, { id: "far", x: 55, y: 5 }];
    expect(m.linked(nodes[0], nodes[2])).toBe(false);
    const got: [string, Packet][] = [];
    m.send("base", "command", { hello: 1 }, "far");
    m.send("base", "beacon", { b: 1 });
    for (let k = 0; k < 4; k++) m.tick(nodes, (n, p) => got.push([n, p]));
    const cmd = got.filter(([, p]) => p.kind === "command");
    expect(cmd.map(([n]) => n)).toEqual(["far"]);
    expect(cmd[0][1].hops).toBe(2);
    const beacons = got.filter(([, p]) => p.kind === "beacon").map(([n]) => n).sort();
    expect(beacons).toEqual(["far", "relay"]);
  });
});

describe("planner", () => {
  it("goes around a wall and not through it", () => {
    const k = new Knowledge(20, 20);
    for (let i = 0; i < 400; i++) k.learn(i, Cell.Ground);
    for (let y = 0; y < 15; y++) k.learn(y * 20 + 10, Cell.Rock);
    const path = aStar(costGrid(k, 1), 20, 20, 2 * 20 + 2, 2 * 20 + 17)!;
    expect(path).not.toBeNull();
    expect(path.some((i) => Math.floor(i / 20) >= 16)).toBe(true); // around the end of the wall
    expect(path.every((i) => k.get(i) !== Cell.Rock)).toBe(true);
  });
});

describe("swarm", () => {
  it("relay points keep every hop in range, none when the goal is near", () => {
    expect(relayPoints({ x: 0, y: 0 }, { x: 30, y: 0 }, 52.5, 35, 10)).toEqual([]);
    const pts = relayPoints({ x: 0, y: 0 }, { x: 106, y: 0 }, 52.5, 35, 10);
    expect(pts.length).toBe(3);
    const chain = [{ x: 0, y: 0 }, ...pts, { x: 106, y: 0 }];
    expect(chain[1].x).toBeLessThanOrEqual(52.5 * 0.75 + 1e-6);
    for (let k = 1; k + 1 < chain.length; k++) expect(chain[k + 1].x - chain[k].x).toBeLessThanOrEqual(35 * 0.75 + 1e-6);
  });

  it("hex slots are distinct and spaced", () => {
    const s = hexSlots({ x: 0, y: 0 }, 10, 1.6);
    expect(s).toHaveLength(10);
    for (let i = 0; i < s.length; i++)
      for (let j = i + 1; j < s.length; j++) expect(Math.hypot(s[i].x - s[j].x, s[i].y - s[j].y)).toBeGreaterThan(1.5);
  });

  it("10 robots cross the terrain and all reach the destination", () => {
    const sw = new Swarm(7);
    sw.spawn(10);
    sw.goalForGroup(sw.robots.map((r) => r.id), sw.world.destination);
    let allT = -1;
    for (let t = 0; t < 900 && allT < 0; t += 0.1) { // up to 15 simulated minutes
      sw.step(0.1);
      if (sw.robots.every((r) => r.arrived)) allT = t;
    }
    expect(allT).toBeGreaterThan(0);
    // the relay chain keeps everyone in touch with the operator at the end
    for (let t = 0; t < 10; t += 0.1) sw.step(0.1);
    expect(sw.operatorView().every((o) => o.age !== null && o.age < 5)).toBe(true);
    expect(sw.robots.filter((r) => sw.role(r.id) === "relay").length).toBeGreaterThan(0);
    expect(allT).toBeLessThan(600);
    expect(sw.robots.every((r) => r.arrived && !r.help)).toBe(true);
    // the base heard from every robot and knows a good part of the terrain
    expect(sw.baseView.size).toBe(10);
    expect(sw.baseKnowledge.knownCount()).toBeGreaterThan(3000); // only what reached the base over the mesh
    // robots never overlap
    for (let i = 0; i < 10; i++)
      for (let j = i + 1; j < 10; j++) {
        const a = sw.robots[i], b = sw.robots[j];
        expect(Math.hypot(a.x - b.x, a.y - b.y)).toBeGreaterThan(0.6);
      }
  }, 120_000);
});


describe("operator rescue", () => {
  it("a trapped robot asks for help; manual control over the mesh frees it; auto resumes", () => {
    const sw = new Swarm(7);
    const r = sw.addRobot("UGV-01", sw.world.start.x, sw.world.start.y);
    sw.command(r.id, { kind: "goal", goal: { x: sw.world.start.x + 20, y: sw.world.start.y } });
    for (let t = 0; t < 5; t += 0.1) sw.step(0.1);
    expect(r.mode).toBe("auto");
    r.trapped = true;                       // a hidden snag
    r.help = "hidden";
    for (let t = 0; t < 2; t += 0.1) sw.step(0.1);
    expect(sw.operatorView()[0].attention).toBe("застрял на скрытом препятствии");
    const x0 = r.x;
    for (let t = 0; t < 3; t += 0.1) sw.step(0.1);
    expect(r.x).toBeCloseTo(x0, 5);          // it does not move by itself
    sw.command(r.id, { kind: "manual" });
    for (let t = 0; t < 2; t += 0.1) sw.step(0.1);
    expect(r.mode).toBe("manual");
    for (let t = 0; t < 12; t += 0.1) {      // the operator drives forward (teleop every 0.1 s)
      sw.command(r.id, { kind: "teleop", v: 0.5, w: 0 });
      sw.step(0.1);
    }
    expect(r.trapped).toBe(false);
    for (let t = 0; t < 1.5; t += 0.1) sw.step(0.1);
    expect(sw.operatorView()[0].attention).toBeNull(); // the alert clears once it is free, still in manual
    sw.command(r.id, { kind: "auto" });
    for (let t = 0; t < 120 && !r.arrived; t += 0.1) sw.step(0.1);
    expect(r.help).toBeNull();
    expect(r.arrived).toBe(true);
  }, 60_000);

  it("a trapped robot touching a neighbour can still be driven out", () => {
    const sw = new Swarm(7);
    const { x, y } = sw.world.start;
    const r = sw.addRobot("UGV-01", x, y);
    const nb = sw.addRobot("UGV-02", x, y + 2 * 0.35);   // exactly at the contact distance, to the left
    r.th = 0;
    r.trapped = true;
    r.help = "hidden";
    sw.command(r.id, { kind: "manual" });
    for (let t = 0; t < 2; t += 0.1) sw.step(0.1);
    expect(r.mode).toBe("manual");
    const x0 = r.x;
    for (let t = 0; t < 8; t += 0.1) {
      sw.command(r.id, { kind: "teleop", v: 0.5, w: 0 });
      sw.step(0.1);
    }
    expect(r.x - x0).toBeGreaterThan(0.5);
    expect(r.trapped).toBe(false);
    expect(Math.hypot(r.x - nb.x, r.y - nb.y)).toBeGreaterThanOrEqual(0.69);
  });

  it("teleop does not reach a robot without a mesh route", () => {
    const sw = new Swarm(7);
    const r = sw.addRobot("far", sw.world.destination.x, sw.world.destination.y);
    sw.command(r.id, { kind: "manual" });
    for (let t = 0; t < 5; t += 0.1) sw.step(0.1);
    expect(r.mode).toBe("idle");             // 100 m away, no relays: the command never arrives
    expect(sw.operatorView()[0].attention).toBe("нет связи с роботом");
  });
});
