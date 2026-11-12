/* One explicit UI action runs the existing server-side grasp coordinator. */
const $ = id => document.getElementById(id);
const declaration = "已核对黄球和路径，现场有人看护且双手离开运动范围";
let state = null, busy = false, polling = false, revision = 0, planReady = false;
let view = { kind: "empty", url: null, planId: null, generation: 0 };
const stream = { active: false, sequence: 0, controller: null, timer: null, url: null, received: 0 };
const phases = { idle: "尚未开始抓取。", recording_warmup: "正在启动本次10秒录像…", align_open: "高位对齐并张爪…",
  hover: "球上方悬停，核对实际到位…", descend: "正在短程下降接近黄球…", close: "正在闭爪，核对角度与接触反馈…",
  lift: "正在提升约5cm…", saving_recording: "正在保存录像和关键帧…", stopping: "已请求停止后续指令，等待当前动作反馈…" };
function sameOriginImage(path) {
  return typeof path === "string" && /^\/api\/bench\/grasp\/(plans|diagnostics|evidence)\/[a-f0-9]{32}\//.test(path);
}
async function api(path, method = "GET", payload) {
  const response = await fetch(path, { method, cache: "no-store", headers: { "Content-Type": "application/json" },
    ...(payload === undefined ? {} : { body: JSON.stringify(payload) }) });
  const raw = await response.text(); let data;
  try { data = JSON.parse(raw); } catch (_) { throw Error(`HTTP ${response.status}：服务未返回JSON，请检查控制服务日志。`); }
  if (!response.ok) {
    const detail = data.detail || data.error || "操作未完成";
    const error = Error(`HTTP ${response.status}：${typeof detail === "string" ? detail : detail.message || "操作未完成"}`);
    error.detail = typeof detail === "object" ? detail : null;
    throw error;
  }
  return data;
}
function showError(error) {
  $("error").textContent = error.message || String(error); $("error").hidden = false;
  if (error.detail) showDiagnostic(error.detail);
}
function stopStream() {
  stream.active = false; stream.controller?.abort(); clearTimeout(stream.timer);
}
function staticImage(url, kind, planId = null) {
  if (!sameOriginImage(url)) return;
  stopStream(); const generation = ++view.generation;
  view = { kind, url, planId, generation }; planReady = false;
  if (stream.url) { URL.revokeObjectURL(stream.url); stream.url = null; }
  $("graspFrame").onload = () => {
    if (view.generation !== generation) return;
    planReady = kind === "plan"; render();
  };
  $("graspFrame").onerror = () => { planReady = false; $("imageCaption").textContent = "图片加载失败，请重新观察。"; render(); };
  $("graspFrame").src = url; $("graspFrame").hidden = false; $("graspEmpty").hidden = true;
  $("streamNotice").hidden = true;
  $("imageCaption").textContent = kind === "plan" ? "本次观察生成的计划预览 · 非实时" : kind === "diagnostic" ? "尺寸核对诊断 · 蓝圈为预期尺寸，橙圈为轮廓等效尺寸" : "实测关键帧 · 抓取结果需根据画面核对";
}
function sizeMetrics(size) {
  $("sizeMetrics").hidden = !size;
  if (!size) return;
  $("observedSize").textContent = size.observed_area_diameter_px.toFixed(1) + " px";
  $("expectedSize").textContent = size.expected_diameter_px.toFixed(1) + " px";
  $("sizeNote").textContent = `偏差 ${size.relative_error_percent >= 0 ? "+" : ""}${size.relative_error_percent.toFixed(1)}%；外接圆直径 ${size.observed_enclosing_diameter_px.toFixed(1)} px。允许范围保持 ±22%。`;
}
function showDiagnostic(detail) {
  if (detail.size) sizeMetrics(detail.size);
  if (sameOriginImage(detail.image_url) && view.url !== detail.image_url) staticImage(detail.image_url, "diagnostic");
  $("maskLink").hidden = !sameOriginImage(detail.mask_url);
  if (!$("maskLink").hidden) $("maskLink").href = detail.mask_url;
}
function render() {
  const h = state?.health || {}, job = state?.job || {}, plan = state?.plan;
  const running = job.status === "running", locked = Array.isArray(h.torque_status) && h.torque_status.length === 6 && h.torque_status.every(v => v === 1);
  $("setupBadge").textContent = state?.setup_id || "当前布置";
  $("connection").textContent = h.connected ? h.fault ? "机械臂反馈异常" : "机械臂已连接" : "机械臂未连接";
  $("connection").className = "status-chip " + (h.connected && !h.fault ? "connected" : "");
  $("motionBadge").textContent = running ? "执行中" : h.motion_enabled ? "实物模式" : "只读模式";
  $("readiness").textContent = h.fault ? (h.fault.message || "存在未解决的运动故障，请先核对实际状态。") :
    !h.connected ? "先连接机械臂；打开串口可能触发初始化，请在现场看护。" :
    !locked ? "六关节扭矩状态未确认锁定，请先核对设备。" :
    !h.motion_enabled ? "控制服务仍是只读模式。请让OpenCode按更新说明开启实物执行。" :
    running ? "本次动作由控制服务执行，录像与反馈同步保存。" : "先观察并核对计划，再执行一次抓取。已有五点标定继续使用。";
  $("connect").hidden = Boolean(h.connected); $("connect").disabled = busy || running;
  $("prepare").disabled = busy || running || !h.connected || !locked || Boolean(h.fault) || Boolean(h.pending);
  $("prepare").textContent = busy ? "正在处理…" : "1 · 观察并规划";
  const ready = plan && !plan.expired && plan.status === "prepared" && planReady && view.planId === plan.plan_id;
  $("confirmed").disabled = busy || running || !ready;
  $("run").disabled = busy || running || !ready || !h.connected || !locked || !h.motion_enabled || Boolean(h.fault) || Boolean(h.pending) || !$("confirmed").checked;
  $("stop").hidden = !running; $("stop").disabled = Boolean(job.stop_requested);
  $("planExpiry").textContent = plan && plan.status === "prepared" ? plan.expired ? "计划已过期，请重新观察。" : `计划还可用 ${Math.ceil(plan.seconds_left)} 秒。` : "";
  $("viewBadge").textContent = running ? "实时执行" : view.kind === "plan" ? "计划预览" : view.kind === "diagnostic" ? "识别诊断" : view.kind === "keyframes" ? "实测关键帧" : "尚未观察";
  $("phase").textContent = running ? phases[job.phase] || "正在执行…" : job.status === "completed" ? "动作序列已完成，是否抓起请核对录像。" :
    job.status === "stopped" ? "后续动作已停止，请核对机械臂实际状态。" : job.status === "failed" ? "执行未完成：" + (job.error || "请查看本次记录") : ready ? "计划已生成。核对画面并勾选后，可执行一次抓取。" : "尚未开始抓取。";
  const result = job.result; $("runResult").hidden = !result;
  if (result) {
    $("resultTitle").textContent = result.motion_completed ? "抓取与提升动作已执行" : "本次动作未完成";
    $("resultText").textContent = (result.grasp_attempted ? "已发出闭爪尝试。" : "闭爪尚未执行。") +
      (typeof result.physical_elapsed_s === "number" ? `动作耗时 ${result.physical_elapsed_s.toFixed(2)} 秒。` : "") + "视觉抓取结果待核对；本轮未投放。";
    $("keyframesLink").hidden = !sameOriginImage(result.keyframes_url);
    if (!$("keyframesLink").hidden) $("keyframesLink").href = result.keyframes_url;
  }
}
async function refresh() {
  if (polling || busy || document.hidden) return;
  polling = true; const observedRevision = revision;
  try {
    const next = await api("/api/bench/grasp/ui-state");
    if (observedRevision !== revision) return;
    const wasRunning = state?.job?.status === "running", oldPlan = state?.plan?.plan_id;
    state = next;
    if (oldPlan !== state.plan?.plan_id || state.plan?.expired) $("confirmed").checked = false;
    if (state.job.status === "running") {
      if (!stream.active && !document.hidden) startStream();
    } else {
      stopStream();
      const resultURL = state.job.result?.keyframes_url;
      if (sameOriginImage(resultURL) && view.url !== resultURL) staticImage(resultURL, "keyframes");
      else if (state.plan?.status === "prepared" && !state.plan.expired && (view.planId !== state.plan.plan_id || view.kind === "empty")) {
        staticImage(state.plan.preview_url, "plan", state.plan.plan_id);
      }
      if (state.plan?.status === "prepared" && state.plan.size) sizeMetrics(state.plan.size);
      if (state.job.last_error) { $("error").textContent = state.job.last_error.message; $("error").hidden = false; showDiagnostic(state.job.last_error); }
      if (wasRunning) loadTrials();
    }
    render();
  } catch (error) { showError(error); }
  finally { polling = false; }
}
async function action(fn) {
  if (busy) return; busy = true; revision++; $("error").hidden = true; render();
  try { await fn(); } catch (error) { showError(error); }
  finally { busy = false; await refresh(); render(); }
}
$("connect").onclick = () => action(async () => {
  const result = await api("/api/auto-connect", "POST", {});
  if (!result.connected) throw Error(result.reason || "未能连接机械臂");
  $("confirmed").checked = false; planReady = false;
});
$("prepare").onclick = () => action(async () => {
  $("confirmed").checked = false; planReady = false; $("maskLink").hidden = true; $("sizeMetrics").hidden = true;
  const result = await api("/api/bench/grasp/prepare", "POST", {});
  staticImage(result.preview_url, "plan", result.plan_id);
});
$("confirmed").onchange = render;
$("run").onclick = () => {
  if ($("run").disabled) return;
  const plan_id = state.plan.plan_id;
  return action(async () => {
    $("confirmed").checked = false;
    await api("/api/bench/grasp/run", "POST", { plan_id, declaration });
  });
};
$("stop").onclick = async () => {
  if (!state?.job?.plan_id || $("stop").disabled) return;
  $("stop").disabled = true;
  try { await api("/api/bench/grasp/stop", "POST", { plan_id: state.job.plan_id }); await refresh(); }
  catch (error) { showError(error); }
};
function startStream() {
  stream.active = true; stream.sequence = 0; stream.received = 0; planReady = false;
  view = { kind: "live", url: null, planId: null, generation: view.generation + 1 };
  $("graspFrame").onload = null; $("graspFrame").onerror = null;
  $("imageCaption").textContent = "正在等待本次实时画面…";
  nextFrame(view.generation);
}
async function nextFrame(generation) {
  if (!stream.active || generation !== view.generation) return;
  stream.controller = new AbortController(); let nextURL;
  try {
    const response = await fetch("/api/bench/live-frame?after=" + stream.sequence, { cache: "no-store", signal: stream.controller.signal });
    if (!response.ok) throw Error("实时画面暂不可用，执行状态仍在更新。");
    const sequence = Number(response.headers.get("X-Frame-Sequence")), age = Number(response.headers.get("X-Frame-Age-Ms"));
    const width = Number(response.headers.get("X-Frame-Width")), height = Number(response.headers.get("X-Frame-Height"));
    if (!Number.isSafeInteger(sequence) || sequence <= stream.sequence || !Number.isFinite(age) || age < 0 || age > 1500 ||
      width !== state.plan?.image_size?.[0] || height !== state.plan?.image_size?.[1]) throw Error("实时帧信息不匹配。");
    nextURL = URL.createObjectURL(await response.blob()); const decoded = new Image(); decoded.src = nextURL; await decoded.decode();
    if (!stream.active || generation !== view.generation) { URL.revokeObjectURL(nextURL); return; }
    if (decoded.naturalWidth !== width || decoded.naturalHeight !== height) throw Error("实时画面尺寸不匹配。");
    const old = stream.url; $("graspFrame").src = nextURL; stream.url = nextURL; nextURL = null;
    if (old) URL.revokeObjectURL(old);
    $("graspFrame").hidden = false; $("graspEmpty").hidden = true; $("streamNotice").hidden = true;
    $("imageCaption").textContent = "实时画面 · 执行进度见右侧";
    stream.sequence = sequence; stream.received = performance.now() - age;
    stream.timer = setTimeout(() => nextFrame(generation), 100);
  } catch (error) {
    if (nextURL) URL.revokeObjectURL(nextURL);
    if (!stream.active || generation !== view.generation) return;
    stopStream(); $("streamNotice").hidden = false; $("streamNotice").textContent = error.message;
    $("imageCaption").textContent = "实时画面已暂停 · 控制服务仍在执行本次动作";
  }
}
async function loadTrials() {
  try {
    const data = await api("/api/task/trials"); $("trials").replaceChildren();
    $("count").textContent = data.trials.length + " 次"; $("emptyTrials").hidden = data.trials.length > 0;
    const label = value => value === true ? "已观察到" : value === false ? "未观察到" : "无法确认";
    for (const trial of data.trials) {
      const row = document.createElement("article"), title = document.createElement("strong"), result = document.createElement("p");
      row.className = "trial-row"; title.textContent = (trial.phase === "grasp_lift" ? "抓取试次 " : "试次 ") + trial.trial_id.slice(0, 10);
      result.textContent = `抓起：${label(trial.pickup_observed)}　${trial.phase === "grasp_lift" ? "投放：本轮未执行　抓取动作" : "入漏斗：" + label(trial.ball_entered_funnel) + "　完整动作"}10秒内：${trial.within_10s === true ? "是" : trial.within_10s === false ? "否" : "未知"}`;
      row.append(title, result);
      for (const text of trial.evidence) { const p = document.createElement("p"); p.textContent = text; row.append(p); }
      $("trials").append(row);
    }
  } catch (error) { showError(error); }
}
document.addEventListener("visibilitychange", () => { if (document.hidden) stopStream(); else refresh(); });
window.addEventListener("pagehide", () => { stopStream(); if (stream.url) URL.revokeObjectURL(stream.url); });
setInterval(() => {
  if (stream.active && stream.received && performance.now() - stream.received > 1500) {
    $("streamNotice").hidden = false; $("streamNotice").textContent = "画面已过期，正在等待新帧…";
  }
}, 250);
setInterval(refresh, 1000); setInterval(() => { if (!document.hidden) loadTrials(); }, 7000);
refresh(); loadTrials();
