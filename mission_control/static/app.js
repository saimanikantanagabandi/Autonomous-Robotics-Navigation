/**
 * Autonomous UGV Tactical Mission Control Client
 * ===============================================
 * Real-time 60 FPS Canvas rendering, WebSocket telemetry client,
 * interactive waypoint dispatch, teleoperation, and live sparkline charts.
 */

// --- Global State ---
let ws = null;
let telemetryData = null;
let staticMapData = null;

// Canvas & Viewport
const canvas = document.getElementById('radarCanvas');
const ctx = canvas.getContext('2d');
const errorCanvas = document.getElementById('errorChartCanvas');
const errCtx = errorCanvas.getContext('2d');

let viewScale = 21.0; // pixels per meter
let viewOffsetX = 0.0;
let viewOffsetY = 0.0;
let showLidar = true;
let showTrails = true;

// Key state for teleoperation
const activeKeys = {};
let teleopTimer = null;

// --- Initialize App ---
window.addEventListener('DOMContentLoaded', async () => {
  setupCanvasSize();
  window.addEventListener('resize', setupCanvasSize);
  
  await fetchStaticMap();
  connectWebSocket();
  setupUIEventListeners();
  setupKeyboardControls();

  // Start 60 FPS Render Loop
  requestAnimationFrame(renderLoop);
});

function setupCanvasSize() {
  const container = document.getElementById('canvasViewport');
  const size = Math.min(container.clientWidth, container.clientHeight);
  canvas.width = size;
  canvas.height = size;
  viewScale = (size * 0.92) / 40.0;
}

// --- Fetch Static Map ---
async function fetchStaticMap() {
  try {
    const res = await fetch('/api/map');
    staticMapData = await res.json();
  } catch (err) {
    console.error('Failed to load static map:', err);
  }
}

// --- WebSocket Connection ---
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;
  const badgeWs = document.getElementById('badge-ws');

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    badgeWs.className = 'badge badge-success';
    badgeWs.innerHTML = '<span class="badge-dot"></span> LINK: ONLINE';
  };

  ws.onmessage = (event) => {
    try {
      telemetryData = JSON.parse(event.data);
      updateTelemetryUI(telemetryData);
    } catch (e) {
      console.error('Error parsing telemetry JSON:', e);
    }
  };

  ws.onclose = () => {
    badgeWs.className = 'badge badge-danger';
    badgeWs.innerHTML = '<span class="badge-dot"></span> LINK: DISCONNECTED';
    setTimeout(connectWebSocket, 1500);
  };

  ws.onerror = () => {
    ws.close();
  };
}

function sendWsCommand(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(msg));
  }
}

// --- Coordinate Transformations ---
function worldToScreen(wx, wy) {
  const cx = canvas.width / 2 + viewOffsetX;
  const cy = canvas.height / 2 + viewOffsetY;
  return {
    x: cx + wx * viewScale,
    y: cy - wy * viewScale // Invert Y for Cartesian
  };
}

function screenToWorld(sx, sy) {
  const cx = canvas.width / 2 + viewOffsetX;
  const cy = canvas.height / 2 + viewOffsetY;
  return {
    x: (sx - cx) / viewScale,
    y: (cy - sy) / -viewScale
  };
}

// --- UI Event Listeners ---
function setupUIEventListeners() {
  // Click-to-Navigate on Canvas
  canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const worldPos = screenToWorld(sx, sy);
    
    sendWsCommand({
      type: 'goal',
      x: worldPos.x,
      y: worldPos.y
    });
  });

  // Center button
  document.getElementById('btnRecenter').addEventListener('click', () => {
    viewOffsetX = 0.0;
    viewOffsetY = 0.0;
  });

  // Toggle LiDAR
  document.getElementById('btnToggleRays').addEventListener('click', () => {
    showLidar = !showLidar;
  });

  // Toggle Trails
  document.getElementById('btnToggleTrails').addEventListener('click', () => {
    showTrails = !showTrails;
  });

  // Mission buttons
  document.getElementById('btnPatrol').addEventListener('click', () => {
    sendWsCommand({ type: 'action', action: 'patrol' });
  });

  document.getElementById('btnPause').addEventListener('click', () => {
    sendWsCommand({ type: 'action', action: 'pause' });
  });

  document.getElementById('btnReset').addEventListener('click', () => {
    sendWsCommand({ type: 'action', action: 'reset' });
  });

  document.getElementById('btnExportCsv').addEventListener('click', () => {
    window.location.href = '/api/export_csv';
  });

  // Fault Toggles
  const toggleSlip = document.getElementById('toggleSlip');
  const toggleBlackout = document.getElementById('toggleBlackout');
  const toggleImuBias = document.getElementById('toggleImuBias');

  toggleSlip.addEventListener('change', () => {
    sendWsCommand({ type: 'fault', wheel_slip: toggleSlip.checked });
  });

  toggleBlackout.addEventListener('change', () => {
    sendWsCommand({ type: 'fault', vslam_blackout: toggleBlackout.checked });
  });

  toggleImuBias.addEventListener('change', () => {
    sendWsCommand({ type: 'fault', imu_bias: toggleImuBias.checked });
  });

  // D-Pad buttons
  setupDpadButton('dpad-up', 0.8, 0.0);
  setupDpadButton('dpad-down', -0.6, 0.0);
  setupDpadButton('dpad-left', 0.2, 1.2);
  setupDpadButton('dpad-right', 0.2, -1.2);
  document.getElementById('dpad-stop').addEventListener('click', () => {
    sendWsCommand({ type: 'teleop', v: 0.0, omega: 0.0 });
  });
}

function setupDpadButton(id, v, omega) {
  const btn = document.getElementById(id);
  const start = (e) => {
    e.preventDefault();
    sendWsCommand({ type: 'teleop', v: v, omega: omega });
  };
  const stop = (e) => {
    e.preventDefault();
    sendWsCommand({ type: 'teleop', v: 0.0, omega: 0.0 });
  };
  btn.addEventListener('mousedown', start);
  btn.addEventListener('mouseup', stop);
  btn.addEventListener('touchstart', start);
  btn.addEventListener('touchend', stop);
}

// --- Keyboard Teleoperation ---
function setupKeyboardControls() {
  window.addEventListener('keydown', (e) => {
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(e.code) ||
        ['KeyW', 'KeyS', 'KeyA', 'KeyD'].includes(e.code)) {
      e.preventDefault();
      activeKeys[e.code] = true;
      processKeyboardDrive();
    }
  });

  window.addEventListener('keyup', (e) => {
    if (activeKeys[e.code]) {
      delete activeKeys[e.code];
      processKeyboardDrive();
    }
  });
}

function processKeyboardDrive() {
  let v = 0.0;
  let omega = 0.0;

  if (activeKeys['Space']) {
    sendWsCommand({ type: 'teleop', v: 0.0, omega: 0.0 });
    return;
  }

  if (activeKeys['KeyW'] || activeKeys['ArrowUp']) v += 0.85;
  if (activeKeys['KeyS'] || activeKeys['ArrowDown']) v -= 0.65;
  if (activeKeys['KeyA'] || activeKeys['ArrowLeft']) omega += 1.25;
  if (activeKeys['KeyD'] || activeKeys['ArrowRight']) omega -= 1.25;

  if (v !== 0.0 || omega !== 0.0) {
    sendWsCommand({ type: 'teleop', v: v, omega: omega });
  } else {
    sendWsCommand({ type: 'teleop', v: 0.0, omega: 0.0 });
  }
}

// --- Update Telemetry UI ---
function updateTelemetryUI(data) {
  if (!data) return;

  // Header badges
  const badgeGps = document.getElementById('badge-gps');
  const badgeVslam = document.getElementById('badge-vslam');
  const badgeEkf = document.getElementById('badge-ekf');

  if (data.watchdog.vslam_degraded || data.in_dropout_zone) {
    badgeVslam.className = 'badge badge-danger blink';
    badgeVslam.innerHTML = '<span class="badge-dot"></span> VSLAM: DROPOUT / BLACKOUT';
  } else {
    badgeVslam.className = 'badge badge-success';
    badgeVslam.innerHTML = '<span class="badge-dot"></span> VSLAM: 15 Hz TRACKING';
  }

  if (data.watchdog.wheel_slipping) {
    badgeEkf.className = 'badge badge-danger blink';
    badgeEkf.innerHTML = '<span class="badge-dot"></span> EKF: SLIP DETECTED';
  } else {
    badgeEkf.className = 'badge badge-info';
    badgeEkf.innerHTML = '<span class="badge-dot"></span> EKF: 15-STATE NOMINAL';
  }

  // Timer
  const secs = Math.floor(data.timestamp);
  const m = String(Math.floor(secs / 60)).padStart(2, '0');
  const s = String(secs % 60).padStart(2, '0');
  document.getElementById('mission-clock').innerText = `T+ ${m}:${s}`;

  // Mode and target display
  document.getElementById('radar-mode-display').innerText = `MODE: ${data.mode}`;
  const targetName = data.current_goal ? data.current_goal.name : 'None';
  document.getElementById('radar-target-display').innerText = `TARGET: ${targetName}`;

  // Pause button state
  const pauseText = document.getElementById('pause-text');
  const pauseIcon = document.getElementById('pause-icon');
  if (data.is_paused) {
    pauseText.innerText = 'RESUME';
    pauseIcon.innerText = '▶';
  } else {
    pauseText.innerText = 'PAUSE';
    pauseIcon.innerText = '⏸';
  }

  // Telemetry numbers
  document.getElementById('gt-x').innerText = `${data.gt.x.toFixed(2)} m`;
  document.getElementById('gt-y').innerText = `${data.gt.y.toFixed(2)} m`;
  document.getElementById('gt-theta').innerText = `${(data.gt.theta * 180 / Math.PI).toFixed(1)}°`;
  document.getElementById('gt-v').innerText = `${data.gt.v.toFixed(2)} m/s`;
  document.getElementById('gt-omega').innerText = `${data.gt.omega.toFixed(2)} rad/s`;

  document.getElementById('ekf-x').innerText = `${data.ekf.x.toFixed(2)} m`;
  document.getElementById('ekf-y').innerText = `${data.ekf.y.toFixed(2)} m`;
  document.getElementById('ekf-theta').innerText = `${(data.ekf.theta * 180 / Math.PI).toFixed(1)}°`;
  document.getElementById('ekf-v').innerText = `${data.ekf.v.toFixed(2)} m/s`;
  document.getElementById('ekf-omega').innerText = `${data.ekf.omega.toFixed(2)} rad/s`;

  document.getElementById('odom-x').innerText = `${data.odom.x.toFixed(2)} m`;
  document.getElementById('odom-y').innerText = `${data.odom.y.toFixed(2)} m`;
  document.getElementById('odom-theta').innerText = `${(data.odom.theta * 180 / Math.PI).toFixed(1)}°`;
  document.getElementById('odom-v').innerText = `${data.odom.v.toFixed(2)} m/s`;
  document.getElementById('odom-omega').innerText = `${data.odom.omega.toFixed(2)} rad/s`;

  // Covariance 2σ
  const ellipse = data.ekf.ellipse;
  document.getElementById('cov-radius').innerText = `±${Math.max(ellipse.rx, ellipse.ry).toFixed(2)} m`;
  document.getElementById('cov-trace').innerText = ellipse.trace.toFixed(4);

  // Benchmarks
  const mtr = data.metrics;
  document.getElementById('drift-reduction-pct').innerText = `+${mtr.drift_reduction_pct.toFixed(1)}%`;
  document.getElementById('ekf-rmse').innerText = `${mtr.rmse_ekf.toFixed(2)} m`;
  document.getElementById('ekf-max-err').innerText = `Max: ${mtr.max_err_ekf.toFixed(2)} m`;
  document.getElementById('odom-rmse').innerText = `${mtr.rmse_odom.toFixed(2)} m`;
  document.getElementById('odom-max-err').innerText = `Max: ${mtr.max_err_odom.toFixed(2)} m`;

  // Render Sparkline Error Chart
  renderErrorChart(mtr.recent_ate_odom, mtr.recent_ate_ekf);

  // Watchdog Alerts Terminal
  updateWatchdogTerminal(data.alerts);

  // Checkpoints list
  updateCheckpointsList(data.checkpoints, data.active_wp_idx, data.mode);
}

// --- Render Sparkline Error Chart ---
function renderErrorChart(odomErrors, ekfErrors) {
  if (!odomErrors || odomErrors.length < 2) return;

  const w = errorCanvas.width;
  const h = errorCanvas.height;
  errCtx.clearRect(0, 0, w, h);

  // Find max scale
  const maxVal = Math.max(5.0, ...odomErrors, ...ekfErrors);

  // Background grid
  errCtx.strokeStyle = '#1e293b';
  errCtx.lineWidth = 1;
  errCtx.beginPath();
  errCtx.moveTo(0, h * 0.5);
  errCtx.lineTo(w, h * 0.5);
  errCtx.stroke();

  // Draw Raw Odometry curve (Red)
  drawLineSeries(odomErrors, maxVal, '#ef4444', 2, [3, 2]);

  // Draw EKF curve (Cyan)
  drawLineSeries(ekfErrors, maxVal, '#06b6d4', 2.5, []);
}

function drawLineSeries(dataSeries, maxVal, color, lineWidth, dash) {
  const w = errorCanvas.width;
  const h = errorCanvas.height;
  const step = w / (dataSeries.length - 1);

  errCtx.save();
  errCtx.strokeStyle = color;
  errCtx.lineWidth = lineWidth;
  errCtx.setLineDash(dash);
  errCtx.beginPath();

  dataSeries.forEach((val, i) => {
    const x = i * step;
    const y = h - (val / maxVal) * (h - 8) - 4;
    if (i === 0) errCtx.moveTo(x, y);
    else errCtx.lineTo(x, y);
  });
  errCtx.stroke();
  errCtx.restore();
}

// --- Watchdog Terminal Logs ---
function updateWatchdogTerminal(alerts) {
  if (!alerts) return;
  const term = document.getElementById('watchdogTerminal');
  term.innerHTML = '';
  alerts.forEach(a => {
    const div = document.createElement('div');
    div.className = `log-line log-${a.level.toLowerCase()}`;
    div.innerHTML = `<span class="log-t">[${a.timestamp}] [${a.category}]</span> ${a.text}`;
    term.appendChild(div);
  });
  term.scrollTop = term.scrollHeight;
}

// --- Checkpoints Checklist ---
function updateCheckpointsList(checkpoints, activeIdx, mode) {
  if (!checkpoints) return;
  const list = document.getElementById('waypointsList');
  list.innerHTML = '';

  checkpoints.forEach((wp, idx) => {
    const item = document.createElement('div');
    const isActive = (mode === 'PATROL' && idx === activeIdx);
    const isCleared = (mode === 'PATROL' && idx < activeIdx);

    item.className = `wp-item ${isActive ? 'active' : ''} ${isCleared ? 'cleared' : ''}`;
    
    let dotClass = 'pending';
    if (isActive) dotClass = 'active';
    else if (isCleared) dotClass = 'cleared';

    item.innerHTML = `
      <span><span class="wp-status-dot ${dotClass}"></span> <b>${wp.id}</b>: ${wp.name.split('(')[0]}</span>
      <span style="color: #64748b;">(${wp.x}, ${wp.y})</span>
    `;
    list.appendChild(item);
  });
}

// ==========================================================================
// 60 FPS Tactical Radar Render Loop
// ==========================================================================
function renderLoop() {
  drawRadar();
  requestAnimationFrame(renderLoop);
}

function drawRadar() {
  const w = canvas.width;
  const h = canvas.height;

  // Clear canvas
  ctx.clearRect(0, 0, w, h);

  // 1. Draw Tactical Grid
  drawGrid();

  // 2. Draw Optical Blackout Zone
  if (staticMapData && staticMapData.dropout_zone) {
    drawDropoutZone(staticMapData.dropout_zone);
  }

  // 3. Draw Bunker Obstacles (Walls, Pillars, Crates)
  if (staticMapData) {
    drawBunkerObstacles();
  }

  // 4. Draw Checkpoint Waypoints
  if (staticMapData && staticMapData.checkpoints) {
    drawCheckpoints(staticMapData.checkpoints);
  }

  // 5. Draw A* Nav2 Planned Path
  if (telemetryData && telemetryData.planned_path && telemetryData.planned_path.length > 0) {
    drawPlannedPath(telemetryData.planned_path);
  }

  // 6. Draw Trajectory Trails
  if (showTrails && telemetryData && telemetryData.trails) {
    drawTrails(telemetryData.trails);
  }

  // 7. Draw 2D LiDAR Rays
  if (showLidar && telemetryData && telemetryData.lidar && telemetryData.gt) {
    drawLidarRays(telemetryData.lidar, telemetryData.gt);
  }

  // 8. Draw Ghost Raw Odometry Robot
  if (telemetryData && telemetryData.odom) {
    drawGhostOdomRobot(telemetryData.odom);
  }

  // 9. Draw EKF Covariance Uncertainty Ellipse
  if (telemetryData && telemetryData.ekf) {
    drawCovarianceEllipse(telemetryData.ekf);
  }

  // 10. Draw Live UGV Chassis (Ground Truth)
  if (telemetryData && telemetryData.gt) {
    drawUgvChassis(telemetryData.gt);
  }
}

// 1. Grid
function drawGrid() {
  const w = canvas.width;
  const h = canvas.height;
  ctx.save();
  ctx.strokeStyle = 'rgba(30, 41, 59, 0.45)';
  ctx.lineWidth = 1;

  for (let m = -20; m <= 20; m += 5) {
    const p1 = worldToScreen(m, -20);
    const p2 = worldToScreen(m, 20);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();

    const p3 = worldToScreen(-20, m);
    const p4 = worldToScreen(20, m);
    ctx.beginPath();
    ctx.moveTo(p3.x, p3.y);
    ctx.lineTo(p4.x, p4.y);
    ctx.stroke();

    // Coordinate text
    if (m % 10 === 0) {
      ctx.fillStyle = '#475569';
      ctx.font = '9px JetBrains Mono';
      ctx.fillText(`${m}m`, p1.x + 3, h - 8);
      ctx.fillText(`${m}m`, 8, p3.y - 3);
    }
  }

  // Crosshair center
  const center = worldToScreen(0, 0);
  ctx.strokeStyle = 'rgba(6, 182, 212, 0.3)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(center.x, center.y, 14, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(center.x - 20, center.y);
  ctx.lineTo(center.x + 20, center.y);
  ctx.moveTo(center.x, center.y - 20);
  ctx.lineTo(center.x, center.y + 20);
  ctx.stroke();

  ctx.restore();
}

// 2. Optical Blackout Zone
function drawDropoutZone(zone) {
  const p1 = worldToScreen(zone.x_min, zone.y_max);
  const p2 = worldToScreen(zone.x_max, zone.y_min);
  const zw = p2.x - p1.x;
  const zh = p2.y - p1.y;

  ctx.save();
  ctx.fillStyle = 'rgba(239, 68, 68, 0.08)';
  ctx.strokeStyle = 'rgba(239, 68, 68, 0.4)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  ctx.fillRect(p1.x, p1.y, zw, zh);
  ctx.strokeRect(p1.x, p1.y, zw, zh);

  // Label
  ctx.fillStyle = '#f87171';
  ctx.font = '10px JetBrains Mono';
  ctx.fillText('⚠ OPTICAL BLACKOUT // IR JAMMED', p1.x + 8, p1.y + 16);
  ctx.restore();
}

// 3. Bunker Obstacles
function drawBunkerObstacles() {
  ctx.save();

  // Walls
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 3;
  ctx.shadowColor = 'rgba(56, 189, 248, 0.3)';
  ctx.shadowBlur = 8;

  staticMapData.walls.forEach(w => {
    const p1 = worldToScreen(w[0], w[1]);
    const p2 = worldToScreen(w[2], w[3]);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  });
  ctx.shadowBlur = 0;

  // Pillars
  staticMapData.pillars.forEach((p, idx) => {
    const sc = worldToScreen(p[0], p[1]);
    const r = p[2] * viewScale;

    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#06b6d4';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Pulse ring
    ctx.strokeStyle = 'rgba(6, 182, 212, 0.4)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, r + 4, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = '#94a3b8';
    ctx.font = '9px JetBrains Mono';
    ctx.fillText(`P${idx + 1}`, sc.x - 6, sc.y + 3);
  });

  ctx.restore();
}

// 4. Checkpoints
function drawCheckpoints(checkpoints) {
  ctx.save();
  checkpoints.forEach((wp) => {
    const sc = worldToScreen(wp.x, wp.y);

    ctx.fillStyle = '#a855f7';
    ctx.strokeStyle = '#c084fc';
    ctx.lineWidth = 1.5;

    ctx.beginPath();
    ctx.arc(sc.x, sc.y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#e2e8f0';
    ctx.font = '10px JetBrains Mono';
    ctx.fillText(wp.id, sc.x + 8, sc.y + 3);
  });
  ctx.restore();
}

// 5. A* Planned Path
function drawPlannedPath(path) {
  if (path.length < 2) return;
  ctx.save();
  ctx.strokeStyle = '#c084fc';
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 4]);

  ctx.beginPath();
  path.forEach((pt, i) => {
    const sc = worldToScreen(pt[0], pt[1]);
    if (i === 0) ctx.moveTo(sc.x, sc.y);
    else ctx.lineTo(sc.x, sc.y);
  });
  ctx.stroke();
  ctx.restore();
}

// 6. Trajectory Trails
function drawTrails(trails) {
  // Ground truth (Green)
  drawSingleTrail(trails.gt, '#10b981', 2, []);

  // Raw Odometry (Red dashed - showing drift!)
  drawSingleTrail(trails.odom, '#ef4444', 1.8, [5, 3]);

  // EKF Fused (Cyan solid)
  drawSingleTrail(trails.ekf, '#06b6d4', 2.2, []);
}

function drawSingleTrail(pts, color, width, dash) {
  if (!pts || pts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);

  ctx.beginPath();
  pts.forEach((pt, i) => {
    const sc = worldToScreen(pt[0], pt[1]);
    if (i === 0) ctx.moveTo(sc.x, sc.y);
    else ctx.lineTo(sc.x, sc.y);
  });
  ctx.stroke();
  ctx.restore();
}

// 7. LiDAR Rays
function drawLidarRays(lidar, robot) {
  const origin = worldToScreen(robot.x, robot.y);
  ctx.save();

  lidar.forEach(beam => {
    const hit = worldToScreen(beam.x, beam.y);

    // Laser beam line (translucent yellow)
    ctx.strokeStyle = 'rgba(234, 179, 8, 0.18)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(hit.x, hit.y);
    ctx.stroke();

    // Laser hit point
    ctx.fillStyle = '#eab308';
    ctx.beginPath();
    ctx.arc(hit.x, hit.y, 2, 0, Math.PI * 2);
    ctx.fill();
  });
  ctx.restore();
}

// 8. Ghost Odometry Robot
function drawGhostOdomRobot(odom) {
  const sc = worldToScreen(odom.x, odom.y);
  ctx.save();
  ctx.translate(sc.x, sc.y);
  ctx.rotate(-odom.theta);

  ctx.strokeStyle = 'rgba(239, 68, 68, 0.6)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([3, 2]);

  const rw = 0.55 * viewScale;
  const rh = 0.40 * viewScale;
  ctx.strokeRect(-rw / 2, -rh / 2, rw, rh);

  // Direction pointer
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(rw * 0.7, 0);
  ctx.stroke();

  ctx.restore();
}

// 9. EKF Covariance Uncertainty Ellipse
function drawCovarianceEllipse(ekf) {
  const sc = worldToScreen(ekf.x, ekf.y);
  const ellipse = ekf.ellipse;
  const rx = ellipse.rx * viewScale;
  const ry = ellipse.ry * viewScale;

  ctx.save();
  ctx.translate(sc.x, sc.y);
  ctx.rotate(-ellipse.angle);

  // 2-sigma boundary
  ctx.strokeStyle = 'rgba(6, 182, 212, 0.75)';
  ctx.lineWidth = 1.5;
  ctx.fillStyle = 'rgba(6, 182, 212, 0.12)';

  ctx.beginPath();
  ctx.ellipse(0, 0, Math.max(rx, 4), Math.max(ry, 4), 0, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.restore();
}

// 10. Live UGV Chassis (Ground Truth)
function drawUgvChassis(robot) {
  const sc = worldToScreen(robot.x, robot.y);
  ctx.save();
  ctx.translate(sc.x, sc.y);
  ctx.rotate(-robot.theta);

  const rw = 0.60 * viewScale;
  const rh = 0.45 * viewScale;
  const wheelW = 0.22 * viewScale;
  const wheelH = 0.10 * viewScale;

  // Left & Right Wheels (Treads)
  ctx.fillStyle = '#0f172a';
  ctx.strokeStyle = '#475569';
  ctx.lineWidth = 1.5;
  // Left wheel
  ctx.fillRect(-rw * 0.3, -rh * 0.65, wheelW, wheelH);
  ctx.strokeRect(-rw * 0.3, -rh * 0.65, wheelW, wheelH);
  // Right wheel
  ctx.fillRect(-rw * 0.3, rh * 0.65 - wheelH, wheelW, wheelH);
  ctx.strokeRect(-rw * 0.3, rh * 0.65 - wheelH, wheelW, wheelH);

  // Main Chassis body
  ctx.fillStyle = '#10b981';
  ctx.strokeStyle = '#34d399';
  ctx.lineWidth = 2;
  ctx.shadowColor = 'rgba(16, 185, 129, 0.5)';
  ctx.shadowBlur = 10;
  ctx.fillRect(-rw / 2, -rh / 2, rw, rh);
  ctx.strokeRect(-rw / 2, -rh / 2, rw, rh);
  ctx.shadowBlur = 0;

  // Laser Scanner Turret
  ctx.fillStyle = '#090d16';
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(0, 0, rw * 0.22, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  // Forward Heading Arrow
  ctx.strokeStyle = '#ffffff';
  ctx.fillStyle = '#ffffff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(rw * 0.65, 0);
  ctx.stroke();

  // Arrowhead
  ctx.beginPath();
  ctx.moveTo(rw * 0.65, 0);
  ctx.lineTo(rw * 0.5, -4);
  ctx.lineTo(rw * 0.5, 4);
  ctx.closePath();
  ctx.fill();

  ctx.restore();
}
