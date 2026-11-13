/* Real workspace handlers, simulated existing controller. No hardware access. */
const test = require("node:test"),
  assert = require("node:assert/strict");
const fs = require("node:fs"),
  path = require("node:path"),
  vm = require("node:vm");
class Element {
  constructor() {
    this.disabled = false;
    this.hidden = false;
    this.checked = false;
    this.value = "";
    this.textContent = "";
    this.children = [];
    this.attributes = {};
  }
  append(...items) {
    this.children.push(...items);
  }
  replaceChildren(...items) {
    this.children = items;
  }
  setAttribute(k, v) {
    this.attributes[k] = v;
  }
  toggleAttribute(k, enabled) {
    if (enabled) this.attributes[k] = "";
    else delete this.attributes[k];
  }
}
const flush = () => new Promise((r) => setImmediate(r));
async function page({
  known = true,
  failAction = false,
  robotError = false,
  completed = 0,
} = {}) {
  const folder = path.join(__dirname, "..", "control_ui");
  const html = fs.readFileSync(
    path.join(folder, "manual_calibration.html"),
    "utf8",
  );
  const elements = new Map(
    [...html.matchAll(/\bid="([^"]+)"/g)].map((m) => [m[1], new Element()]),
  );
  let mode = "locked",
    failRobot = robotError,
    baseline = null,
    roll = 0;
  const points = ["P1", "P2", "P3", "P4", "V1"].map((id, i) => ({
    id,
    grid: [1 + i, 1],
    captured: i < completed,
    pose_warnings: [],
  }));
  const calls = [];
  const health = () => ({
    connected: true,
    teach_enabled: true,
    pending: null,
    fault: null,
    teach_active: mode === "manual",
    torque_status: known ? Array(6).fill(mode === "manual" ? 0 : 1) : null,
  });
  const status = () => ({
    setup_id: "bench-layout-v2",
    session_exists: true,
    health: health(),
    points,
    reference_tool_pose_rad: baseline,
    next_point: points.find((p) => !p.captured)?.id || null,
    current_image_available: false,
  });
  const context = {
    document: {
      getElementById: (id) => elements.get(id),
      createElement: () => new Element(),
      createElementNS: () => new Element(),
      addEventListener: () => {},
      hidden: false,
    },
    setInterval: () => 1,
    window: { addEventListener: () => {} },
    clearTimeout,
    performance,
    console,
    fetch: async (url, options = {}) => {
      const method = options.method || "GET",
        payload = options.body ? JSON.parse(options.body) : null;
      calls.push({ url, method, payload });
      let result;
      if (url === "/api/bench/status") result = status();
      else if (url === "/api/bench/sources") result = { sources: [] };
      else if (url === "/api/ports") result = [];
      else if (url === "/api/bench/robot") {
        if (failRobot)
          return {
            ok: false,
            status: 409,
            text: async () => JSON.stringify({ detail: "device gone" }),
          };
        result = {
          xyz_mm: [300, 20, -80],
          pitch_rad: 0.1,
          roll_rad: roll,
          gripper_rad: 3,
          health: health(),
        };
      } else if (url === "/api/teach/torque") {
        if (failAction)
          return {
            ok: false,
            status: 500,
            text: async () => "Internal Server Error",
          };
        mode = payload.enabled ? "locked" : "manual";
        result = { verified: true };
      } else if (url === "/api/bench/capture") {
        assert.equal(payload.aligned_and_supported, true);
        points.find((p) => p.id === payload.point_id).captured = true;
        baseline = { pitch: 0.1, roll: 0, gripper: 3 };
        result = status();
      } else throw Error("unexpected request " + url);
      return {
        ok: true,
        status: 200,
        text: async () => JSON.stringify(result),
      };
    },
  };
  vm.runInNewContext(
    fs.readFileSync(path.join(folder, "workspace.js"), "utf8"),
    context,
  );
  for (let i = 0; i < 8; i++) await flush();
  return {
    el: (id) => elements.get(id),
    calls,
    setRobotError: (v) => {
      failRobot = v;
    },
    setRoll: (v) => {
      roll = v;
    },
    points,
  };
}
async function support(p) {
  p.el("supported").checked = true;
  p.el("supported").onchange();
}
async function release(p) {
  await support(p);
  await p.el("release").onclick();
}

test("loading the page only reads, and unknown torque disables switching", async () => {
  const p = await page({ known: false });
  await support(p);
  assert.equal(p.el("release").disabled, true);
  assert.equal(p.el("lock").disabled, true);
  assert.equal(p.calls.filter((c) => c.method === "POST").length, 0);
});
test("release uses existing all-joint endpoint and never retries on double click", async () => {
  const p = await page();
  assert.equal(p.el("release").disabled, true);
  await support(p);
  await Promise.all([p.el("release").onclick(), p.el("release").onclick()]);
  const calls = p.calls.filter((c) => c.url === "/api/teach/torque");
  assert.equal(calls.length, 1);
  assert.equal(calls[0].payload.declaration, "已托住机械臂并确认释放全部关节");
  assert.match(p.el("torqueBadge").textContent, /六关节已释放/);
});
test("five manual points capture while torque stays released, without intermediate relock", async () => {
  const p = await page();
  await release(p);
  for (const id of ["P1", "P2", "P3", "P4", "V1"]) {
    p.el("aligned").checked = true;
    p.el("aligned").onchange();
    assert.equal(p.el("capture").disabled, false);
    assert.match(p.el("capture").textContent, new RegExp(id));
    await p.el("capture").onclick();
  }
  assert.equal(p.el("progress").textContent, "5 / 5");
  assert.deepEqual(
    p.calls
      .filter((c) => c.url === "/api/bench/capture")
      .map((c) => c.payload.point_id),
    ["P1", "P2", "P3", "P4", "V1"],
  );
  assert.equal(p.calls.filter((c) => c.url === "/api/teach/torque").length, 1);
  await support(p);
  await p.el("lock").onclick();
  assert.equal(p.calls.filter((c) => c.url === "/api/teach/torque").length, 2);
  assert.ok(p.points.every((p) => p.captured));
});
test("resume selects P3; changing points clears support/alignment declaration", async () => {
  const p = await page({ completed: 2 });
  assert.match(p.el("capture").textContent, /P3/);
  p.el("aligned").checked = true;
  p.el("points").children[3].onclick();
  assert.equal(p.el("aligned").checked, false);
  assert.match(p.el("capture").textContent, /P4/);
  assert.equal(p.calls.filter((c) => c.method === "POST").length, 0);
});
test("rereading state blocks a changed wrist rotation before capture", async () => {
  const p = await page();
  await release(p);
  p.el("aligned").checked = true;
  await p.el("capture").onclick();
  p.setRoll(0.5);
  await p.el("refresh").onclick();
  p.el("aligned").checked = true;
  p.el("aligned").onchange();
  assert.equal(p.el("capture").disabled, true);
  assert.match(p.el("toolConsistency").textContent, /调整回/);
});
test("plain HTTP500 explains endpoint/status and does not retry torque", async () => {
  const p = await page({ failAction: true });
  await release(p);
  assert.match(p.el("error").textContent, /HTTP 500/);
  assert.doesNotMatch(p.el("error").textContent, /Unexpected token/);
  assert.equal(p.calls.filter((c) => c.url === "/api/teach/torque").length, 1);
});
test("feedback failure invalidates alignment and blocks subsequent capture", async () => {
  const p = await page();
  p.el("aligned").checked = true;
  p.setRobotError(true);
  await p.el("refresh").onclick();
  assert.equal(p.el("aligned").checked, false);
  assert.equal(p.el("capture").disabled, true);
  assert.match(p.el("error").textContent, /device gone/);
});
