/* Manual calibration only. No joint jogs, pose teaching or automatic actions. */
const $ = (id) => document.getElementById(id);
let state = null,
  selected = "P1",
  busy = false,
  polling = false,
  imageMode = "reference",
  imageVersion = 0,
  imageReady = false;
const pointButtons = new Map();
let toolMismatch = false;
function torqueState(health) {
  const flags = health.torque_status;
  const valid =
    Array.isArray(flags) &&
    flags.length === 6 &&
    flags.every((v) => v === 0 || v === 1);
  return {
    valid,
    mode: !valid
      ? "unknown"
      : flags.every((v) => v === 1)
        ? "locked"
        : flags.every((v) => v === 0)
          ? "manual"
          : "unknown",
  };
}
async function api(url, method = "GET", payload) {
  const options = {
    method,
    cache: "no-store",
    headers: { "Content-Type": "application/json" },
  };
  if (payload !== undefined) options.body = JSON.stringify(payload);
  const response = await fetch(url, options),
    raw = await response.text();
  let value;
  try {
    value = JSON.parse(raw);
  } catch (_) {
    throw Error(
      `HTTP ${response.status} · ${url}：服务返回了非JSON内容，请查看控制服务日志。`,
    );
  }
  if (!response.ok)
    throw Error(
      `HTTP ${response.status}：${value.detail || value.error || "操作未完成"}`,
    );
  return value;
}
function showError(e) {
  $("error").textContent = e.message || String(e);
  $("error").hidden = false;
}
async function action(fn) {
  if (busy) return;
  busy = true;
  $("error").hidden = true;
  render();
  try {
    await fn();
  } catch (e) {
    showError(e);
  } finally {
    busy = false;
    render();
  }
}
function resetAlignment() {
  $("aligned").checked = false;
}
function switchImage(mode) {
  imageMode = mode;
  imageVersion++;
  $("frame").src = "/api/bench/frame?mode=" + mode + "&v=" + imageVersion;
  $("frame").hidden = false;
  $("emptyImage").hidden = true;
  render();
}
function render() {
  const health = state?.health || {},
    manual = torqueState(health),
    exists = state?.session_exists;
  $("setupBadge").textContent = state?.setup_id || "当前布置";
  $("prepare").hidden = Boolean(exists);
  $("connection").textContent = health.fault
    ? "反馈异常"
    : health.connected
      ? "机械臂已连接"
      : "机械臂未连接";
  $("connection").className =
    "status-chip " + (health.connected && !health.fault ? "connected" : "");
  const mode = manual.mode;
  $("torqueBadge").textContent =
    mode === "locked"
      ? "全部已锁定"
      : mode === "manual"
        ? "六关节已释放"
        : "状态未确认";
  $("torqueBadge").className =
    "mini-badge " + (mode === "manual" ? "manual" : "");
  $("capabilityNote").textContent = !health.connected
    ? "请先连接机械臂，读取实际状态。"
    : !manual.valid || mode === "unknown"
      ? "六关节扭矩状态尚未核实。请保持支撑并读取实际状态。"
      : !health.teach_enabled
        ? "手动标定尚未启用，请按部署说明启动控制服务。"
        : mode === "manual"
          ? "六关节都已释放。请持续支撑，逐点对准后直接记录；五点结束再恢复扭矩。"
          : "沿用现有全部释放接口。已核对的官方版本会先回位再释放；恢复扭矩也可能移动，请检查整臂范围。";
  const ready = health.connected && manual.valid && health.teach_enabled;
  $("release").disabled =
    busy ||
    !exists ||
    !ready ||
    mode !== "locked" ||
    Boolean(health.fault) ||
    Boolean(health.pending) ||
    !$("supported").checked;
  $("lock").disabled =
    busy ||
    !ready ||
    mode === "locked" ||
    Boolean(health.pending) ||
    !$("supported").checked;
  $("refresh").disabled = busy || !health.connected;
  $("snapshot").disabled = busy || !exists;
  $("currentTab").disabled = busy || !state?.current_image_available;
  $("referenceTab").disabled = busy || !exists;
  $("referenceTab").setAttribute(
    "aria-pressed",
    String(imageMode === "reference"),
  );
  $("currentTab").setAttribute("aria-pressed", String(imageMode === "current"));
  $("imageCaption").textContent =
    imageMode === "reference"
      ? "固定机位 · 非实时参考图"
      : "最近一次快照 · 非实时画面";
  $("connect").disabled = busy || health.connected;
  $("initialize").disabled = busy || !$("source").value;
  const points =
    state?.points ||
    Object.entries({
      P1: [1, 1],
      P2: [7, 1],
      P3: [7, 4],
      P4: [1, 4],
      V1: [4, 2],
    }).map(([id, grid]) => ({ id, grid, captured: false }));
  const current = points.find((p) => p.id === selected) || points[0];
  $("targetName").textContent = selected;
  $("targetGrid").textContent =
    `${selected === "V1" ? "独立验证" : "拟合点"} · 格点 (${current.grid.join(", ")})，从0计数`;
  $("progress").textContent = points.filter((p) => p.captured).length + " / 5";
  for (const point of points) {
    let button = pointButtons.get(point.id);
    if (!button) {
      button = document.createElement("button");
      button.type = "button";
      button.className = "point-button";
      button.onclick = () => {
        selected = point.id;
        resetAlignment();
        render();
      };
      $("points").append(button);
      pointButtons.set(point.id, button);
    }
    button.textContent = point.id + (point.captured ? " ✓" : "");
    button.setAttribute("aria-pressed", String(selected === point.id));
    button.disabled = busy || !exists;
  }
  $("capture").textContent =
    (current.captured ? "重新记录 " : "记录 ") + selected;
  $("capture").disabled =
    busy ||
    !exists ||
    !health.connected ||
    mode === "unknown" ||
    !manual.valid ||
    toolMismatch ||
    Boolean(health.fault) ||
    Boolean(health.pending) ||
    !$("aligned").checked;
  $("captureHint").textContent = current.pose_warnings?.length
    ? current.pose_warnings.join("；")
    : "每次读取三组静止反馈。成功后自动保存。";
  const fit = state?.fit;
  $("resultCard").hidden = !fit;
  if (fit) {
    $("resultTitle").textContent = fit.planar_fit_validated
      ? "五点标定通过"
      : "需要检查对准";
    $("resultBadge").textContent = fit.planar_fit_validated
      ? "已保存"
      : "误差未通过";
    $("rms").textContent = fit.train_rms_mm.toFixed(1) + " mm";
    $("validation").textContent = fit.validation_error_mm.toFixed(1) + " mm";
    $("resultNote").textContent = fit.planar_fit_validated
      ? "平面对应关系已保存。下一步检验抓取几何，再进行首轮实物试验。"
      : "已有采点全部保留。检查触点是否一致，再选择需要修正的点重采。";
  }
  $("sourceInfo").textContent = state?.reference_trial_id
    ? "参考录像：" + state.reference_trial_id
    : "";
}
async function update() {
  state = await api("/api/bench/status");
  if (state.next_point && !pointButtons.size) selected = state.next_point;
  render();
  if (state.session_exists && !imageReady) {
    imageReady = true;
    switchImage("reference");
  }
}
function degrees(v) {
  return typeof v === "number" ? ((v * 180) / Math.PI).toFixed(1) + "°" : "—";
}
async function robot() {
  if (polling) return;
  polling = true;
  try {
    const value = await api("/api/bench/robot");
    if (state) state.health = value.health;
    $("xyz").textContent = value.xyz_mm.every((v) => typeof v === "number")
      ? value.xyz_mm.map((v) => v.toFixed(1)).join(" / ") + " mm"
      : "—";
    $("attitude").textContent =
      `俯仰 ${degrees(value.pitch_rad)} · 旋转 ${degrees(value.roll_rad)} · 夹爪 ${degrees(value.gripper_rad)}`;
    const baseline = state?.reference_tool_pose_rad;
    toolMismatch = false;
    if (
      baseline &&
      typeof value.roll_rad === "number" &&
      typeof value.gripper_rad === "number"
    ) {
      const difference = (a, b) =>
        Math.abs(
          (Math.atan2(Math.sin(a - b), Math.cos(a - b)) * 180) / Math.PI,
        );
      const roll = difference(value.roll_rad, baseline.roll),
        grip = difference(value.gripper_rad, baseline.gripper);
      toolMismatch = roll > 5 || grip > 5;
      $("toolConsistency").textContent =
        `相对首点：旋转差 ${roll.toFixed(1)}°，开合差 ${grip.toFixed(1)}°。` +
        (toolMismatch ? "请调整回相同触点后再记录。" : "触点姿态范围一致。");
      $("toolConsistency").className = toolMismatch
        ? "attitude pose-warning"
        : "attitude";
    }
    render();
  } catch (e) {
    if (state) {
      state.health.torque_status = null;
      state.health.fault = { message: e.message };
    }
    resetAlignment();
    showError(e);
    render();
  } finally {
    polling = false;
  }
}
$("supported").onchange = render;
$("aligned").onchange = render;
$("referenceTab").onclick = () => switchImage("reference");
$("currentTab").onclick = () => switchImage("current");
$("refresh").onclick = () => action(robot);
$("connect").onclick = () =>
  action(async () => {
    const port = $("port").value;
    const result = port
      ? await api("/api/connect", "POST", { port })
      : await api("/api/auto-connect", "POST", {});
    if (result.connected === false) throw Error(result.reason || "未连接");
    await update();
    await robot();
  });
$("initialize").onclick = () =>
  action(async () => {
    await api("/api/bench/start", "POST", {
      trial_id: $("source").value,
      square_mm: Number($("square").value),
    });
    imageReady = false;
    await update();
  });
$("source").onchange = render;
async function torqueRequest(enabled) {
  try {
    return await api("/api/teach/torque", "POST", {
      enabled,
      declaration: enabled
        ? "已托住机械臂并确认重新锁定"
        : "已托住机械臂并确认释放全部关节",
    });
  } catch (error) {
    if (state) state.health.torque_status = null;
    $("actionMessage").textContent =
      "扭矩切换未核实。请保持支撑并读取实际状态，不要重复点击切换。";
    throw error;
  } finally {
    $("supported").checked = false;
  }
}
$("release").onclick = () =>
  action(async () => {
    resetAlignment();
    if (!$("supported").checked)
      throw Error("请先确认已支撑机械臂并检查可能的回位路径");
    const r = await torqueRequest(false);
    $("actionMessage").textContent = r.verified
      ? "六关节已释放。持续支撑，对准 " + selected + " 后直接记录，无需上锁。"
      : r.message;
    $("supported").checked = false;
    await update();
    if (!r.verified) state.health.torque_status = null;
  });
$("lock").onclick = () =>
  action(async () => {
    resetAlignment();
    if (!$("supported").checked)
      throw Error("请先确认已支撑机械臂并检查恢复扭矩时的运动范围");
    const r = await torqueRequest(true);
    $("actionMessage").textContent = r.verified
      ? "已确认全部扭矩恢复。请核对机械臂实际位置；已记录的点仍保留。"
      : r.message;
    $("supported").checked = false;
    await update();
    if (!r.verified) state.health.torque_status = null;
  });
$("capture").onclick = () =>
  action(async () => {
    const point = state.points.find((p) => p.id === selected),
      captured = selected;
    const result = await api("/api/bench/capture", "POST", {
      point_id: selected,
      aligned_and_supported: $("aligned").checked,
      replace: point?.captured || false,
    });
    resetAlignment();
    await update();
    if (result.next_point) selected = result.next_point;
    $("actionMessage").textContent =
      captured +
      " 已保存。" +
      (result.next_point
        ? "保持支撑，手动对准下一点 " + result.next_point + " 后直接记录。"
        : "五点已采完。检查验证结果，支撑机械臂后可恢复扭矩。");
  });
$("snapshot").onclick = () =>
  action(async () => {
    const result = await api("/api/bench/snapshot", "POST", {});
    await update();
    switchImage("current");
    $("imageCaption").textContent =
      "快照 " +
      new Date(result.captured_at).toLocaleTimeString() +
      " · 非实时画面";
  });
$("frame").onerror = () => {
  imageReady = false;
  $("frame").hidden = true;
  $("emptyImage").hidden = false;
};
(async () => {
  try {
    await update();
    const [sources, ports] = await Promise.all([
      api("/api/bench/sources"),
      api("/api/ports"),
    ]);
    $("source").replaceChildren();
    for (const row of sources.sources) {
      const option = document.createElement("option");
      option.value = row.trial_id;
      option.textContent = row.trial_id;
      $("source").append(option);
    }
    if (!sources.sources.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "尚无完整的 Gemini 录像";
      $("source").append(option);
    }
    for (const row of ports) {
      const option = document.createElement("option");
      option.value = row.device;
      option.textContent = row.device;
      $("port").append(option);
    }
    if (state.health.connected) await robot();
    render();
  } catch (e) {
    showError(e);
  }
})();
setInterval(() => {
  if (!busy && !document.hidden && state?.health.connected) robot();
}, 3000);
