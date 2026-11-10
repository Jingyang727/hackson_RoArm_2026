/* Read-only connection chip for pages without a live arm status view. */
(() => {
  const chip=document.getElementById("navConnectionStatus");
  if (!chip) return;
  async function refresh() {
    try {
      const response=await fetch("/api/health",{cache:"no-store"});
      if (!response.ok) throw new Error("unavailable");
      const health=await response.json();
      chip.textContent=health.connected ? "已连接 · "+(health.port||"RoArm") : "机械臂未连接";
      chip.classList.toggle("offline",!health.connected);
    } catch (_) {
      chip.textContent="连接状态未知";
      chip.classList.add("offline");
    }
  }
  refresh();setInterval(refresh,4000);
})();
