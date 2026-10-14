/**
 * AEGIS-SAR Tactical Mission Control Client
 * =========================================
 * Real-time 60 FPS Canvas rendering, WebSocket telemetry client,
 * OpenCV AI vision stream, survivor triage manifest, and emergency alerting.
 */

// --- Global State ---
let ws = null;
let telemetryData = null;
let staticMapData = null;
let audioEnabled = true;
let audioCtx = null;
let lastAlertId = null;
let activeRescueRoute = null;

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
  if (!container) return;
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

// --- Web Audio API Emergency Siren / Alarm Chime ---
function playRescueAlertSound() {
  if (!audioEnabled) return;
  try {
    const AudioContext = window.AudioContext || window.webkitAudioContext;
    if (!audioCtx) audioCtx = new AudioContext();
    if (audioCtx.state === 'suspended') audioCtx.resume();

    const now = audioCtx.currentTime;
    const osc1 = audioCtx.createOscillator();
    const osc2 = audioCtx.createOscillator();
    const gain = audioCtx.createGain();

    osc1.type = 'sawtooth';
    osc2.type = 'sine';

    // Emergency warble tone (880Hz -> 1320Hz)
    osc1.frequency.setValueAtTime(880, now);
    osc1.frequency.linearRampToValueAtTime(1320, now + 0.18);
    osc1.frequency.linearRampToValueAtTime(880, now + 0.36);

    osc2.frequency.setValueAtTime(440, now);
    osc2.frequency.linearRampToValueAtTime(660, now + 0.18);
    osc2.frequency.linearRampToValueAtTime(440, now + 0.36);

    gain.gain.setValueAtTime(0.18, now);
    gain.gain.exponentialRampToValueAtTime(0.001, now + 0.45);

    osc1.connect(gain);
    osc2.connect(gain);
    gain.connect(audioCtx.destination);

    osc1.start(now);
    osc2.start(now);
    osc1.stop(now + 0.46);
    osc2.stop(now + 0.46);
  } catch (e) {
    console.log('Audio alert error or blocked by browser policy:', e);
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
    y: cy - wy * viewScale // Invert Y for Cartesian coordinates
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

  // Audio Toggle Button
  const btnToggleAudio = document.getElementById('btnToggleAudio');
  btnToggleAudio.addEventListener('click', () => {
    audioEnabled = !audioEnabled;
    const icon = document.getElementById('audio-icon');
    if (audioEnabled) {
      btnToggleAudio.innerHTML = '<span id="audio-icon">🔊</span> AUDIO: ON';
      btnToggleAudio.style.color = 'var(--text-secondary)';
    } else {
      btnToggleAudio.innerHTML = '<span id="audio-icon">🔇</span> AUDIO: MUTED';
      btnToggleAudio.style.color = '#ef4444';
    }
  });

  // Vision Mode Switcher
  document.querySelectorAll('.btn-vision-mode').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.btn-vision-mode').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const mode = btn.getAttribute('data-mode');
      document.getElementById('vision-mode-tag').innerText = btn.innerText;
      sendWsCommand({ type: 'sar_mode', mode: mode });
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

  // Mission Buttons
  document.getElementById('btnSarSweep').addEventListener('click', () => {
    sendWsCommand({ type: 'sar_action', action: 'sweep' });
  });

  document.getElementById('btnPause').addEventListener('click', () => {
    sendWsCommand({ type: 'action', action: 'pause' });
  });

  document.getElementById('btnReset').addEventListener('click', () => {
    sendWsCommand({ type: 'action', action: 'reset' });
    activeRescueRoute = null;
    document.getElementById('emergencyBanner').style.display = 'none';
  });

  document.getElementById('btnReturnToBase').addEventListener('click', () => {
    sendWsCommand({ type: 'sar_action', action: 'rtb' });
  });

  // Export Buttons
  document.getElementById('btnExportCsv').addEventListener('click', () => {
    window.location.href = '/api/export_csv';
  });

  document.getElementById('btnExportManifest').addEventListener('click', async () => {
    try {
      const res = await fetch('/api/sar/manifest');
      const data = await res.json();
      const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `sar_casualty_manifest_${Date.now()}.json`;
      a.click();
      URL.revokeObjectURL(url);
    } catch (e) {
      console.error('Failed to export manifest:', e);
    }
  });

  // Disaster Event Buttons
  document.getElementById('btnEventAftershock').addEventListener('click', () => {
    sendWsCommand({ type: 'sar_action', action: 'disaster_event', event_type: 'aftershock' });
  });

  document.getElementById('btnEventFireFlare').addEventListener('click', () => {
    sendWsCommand({ type: 'sar_action', action: 'disaster_event', event_type: 'fire_flare' });
  });

  document.getElementById('btnEventSmoke').addEventListener('click', () => {
    sendWsCommand({ type: 'sar_action', action: 'disaster_event', event_type: 'smoke_blackout' });
  });

  // Fault Toggles
  document.getElementById('toggleSlip').addEventListener('change', (e) => {
    sendWsCommand({ type: 'fault', wheel_slip: e.target.checked });
  });
  document.getElementById('toggleBlackout').addEventListener('change', (e) => {
    sendWsCommand({ type: 'fault', vslam_blackout: e.target.checked });
  });
  document.getElementById('toggleImuBias').addEventListener('change', (e) => {
    sendWsCommand({ type: 'fault', imu_bias: e.target.checked });
  });

  // Emergency Banner Actions
  document.getElementById('btnBannerDismiss').addEventListener('click', () => {
    document.getElementById('emergencyBanner').style.display = 'none';
  });

  document.getElementById('btnBannerDispatch').addEventListener('click', () => {
    if (lastAlertId) {
      dispatchRescue(lastAlertId);
    }
    document.getElementById('emergencyBanner').style.display = 'none';
  });

  // D-Pad Touch / Click Buttons
  setupDpadButton('dpad-up', 0.85, 0.0);
  setupDpadButton('dpad-down', -0.65, 0.0);
  setupDpadButton('dpad-left', 0.0, 1.25);
  setupDpadButton('dpad-right', 0.0, -1.25);
  document.getElementById('dpad-stop').addEventListener('click', () => {
    sendWsCommand({ type: 'teleop', v: 0.0, omega: 0.0 });
  });
}

function setupDpadButton(id, v, omega) {
  const btn = document.getElementById(id);
  if (!btn) return;
  const start = (e) => {
    e.preventDefault();
    sendWsCommand({ type: 'teleop', v, omega });
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

// --- Dispatch Rescue API Call ---
async function dispatchRescue(survivorId) {
  try {
    const res = await fetch('/api/sar/action', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: 'rescue', survivor_id: survivorId })
    });
    const result = await res.json();
    if (result.status === 'ok' && result.ingress_route) {
      activeRescueRoute = result.ingress_route;
    }
  } catch (err) {
    console.error('Failed to dispatch rescue:', err);
  }
}

async function markRescued(survivorId) {
  try {
    await fetch('/api/sar/action', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: 'mark_rescued', survivor_id: survivorId })
    });
  } catch (err) {
    console.error('Failed to mark rescued:', err);
  }
}

// --- Update Telemetry UI ---
function updateTelemetryUI(data) {
  if (!data) return;

  // Header badges
  const badgeVslam = document.getElementById('badge-vslam');
  const badgeEkf = document.getElementById('badge-ekf');

  if (data.watchdog.vslam_degraded || data.in_dropout_zone) {
    badgeVslam.className = 'badge badge-danger blink';
    badgeVslam.innerHTML = '<span class="badge-dot"></span> VSLAM: DROPOUT / SMOKE';
  } else {
    badgeVslam.className = 'badge badge-success';
    badgeVslam.innerHTML = '<span class="badge-dot"></span> VSLAM: 15 Hz TRACKING';
  }

  if (data.watchdog.wheel_slipping) {
    badgeEkf.className = 'badge badge-danger blink';
    badgeEkf.innerHTML = '<span class="badge-dot"></span> EKF: RUBBLE SLIP';
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
  const targetName = data.current_goal ? data.current_goal.name : 'Sector Sweep';
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

  // Sparkline Error Chart
  renderErrorChart(mtr.recent_ate_odom, mtr.recent_ate_ekf);

  // Watchdog Alerts Terminal
  updateWatchdogTerminal(data.alerts);

  // Checkpoints list
  updateCheckpointsList(data.checkpoints, data.active_wp_idx, data.mode);

  // Update SAR Specific Features: Camera stream, Detections, Triage
  if (data.sar) {
    updateSARFeatures(data.sar);
  }
}

// --- Update Search & Rescue Video Stream & Triage ---
function updateSARFeatures(sar) {
  // 1. Camera Stream Frame
  const cameraImg = document.getElementById('cameraStreamImg');
  if (sar.camera_frame && sar.camera_frame.length > 50) {
    cameraImg.src = sar.camera_frame;
  }

  // 2. Detection Counter & Active Detection Pill
  const numDets = sar.active_detections ? sar.active_detections.length : 0;
  document.getElementById('hud-det-count').innerText = `${numDets} HUMAN DETECTIONS`;

  const detBox = document.getElementById('activeDetectionBox');
  if (numDets > 0) {
    const primary = sar.active_detections[0];
    document.getElementById('det-target-name').innerText = `${primary.name}`;
    document.getElementById('det-conf').innerText = `${Math.round(primary.confidence * 100)}%`;
    document.getElementById('det-range').innerText = `${primary.dist_m} m`;
    document.getElementById('det-temp').innerText = `${primary.temp_c} °C`;
    document.getElementById('det-triage').innerText = `${primary.triage}`;
  } else {
    document.getElementById('det-target-name').innerText = 'SECTOR CLEAR // SEARCHING...';
    document.getElementById('det-conf').innerText = '--%';
    document.getElementById('det-range').innerText = '-- m';
    document.getElementById('det-temp').innerText = '-- °C';
    document.getElementById('det-triage').innerText = '--';
  }

  // 3. Triage Counts
  const tc = sar.triage_counts || { RED: 0, YELLOW: 0, GREEN: 0, RESCUED: 0, TOTAL: 4 };
  document.getElementById('count-red').innerText = tc.RED;
  document.getElementById('count-yellow').innerText = tc.YELLOW;
  document.getElementById('count-green').innerText = tc.GREEN;
  document.getElementById('count-rescued').innerText = tc.RESCUED;
  document.getElementById('triage-status-txt').innerText = `${tc.TOTAL - tc.RESCUED} ACTIVE (${tc.RED} RED)`;

  // 4. Render Survivor Triage Manifest Cards
  renderSurvivorCards(sar.survivors || []);

  // 5. Check for New Urgent Alerts
  if (sar.sar_alerts && sar.sar_alerts.length > 0) {
    const latestAlert = sar.sar_alerts[0];
    if (latestAlert.id !== lastAlertId) {
      lastAlertId = latestAlert.id;
      showEmergencyBanner(latestAlert);
      playRescueAlertSound();
    }
  }
}

function showEmergencyBanner(alert) {
  const banner = document.getElementById('emergencyBanner');
  document.getElementById('bannerHeadline').innerText = `🚨 CRITICAL SURVIVOR LOCATED: ${alert.name}`;
  document.getElementById('bannerSub').innerText = 
    `Grid: (X: ${alert.world_x}m, Y: ${alert.world_y}m) | TRIAGE ${alert.triage} | Temp: ${alert.temp_c}°C | Vitals: ${alert.vitals_summary}`;
  banner.style.display = 'block';
}

function renderSurvivorCards(survivors) {
  const container = document.getElementById('survivorCardsContainer');
  if (!container) return;

  container.innerHTML = '';
  survivors.forEach(s => {
    const card = document.createElement('div');
    const isDiscovered = s.discovered;
    const isRescued = s.rescued;

    let triageClass = s.triage === 'RED' ? 'critical' : (s.triage === 'YELLOW' ? 'delayed' : 'minor');
    if (isRescued) triageClass = 'rescued';

    card.className = `survivor-card ${isDiscovered ? 'discovered' : ''} ${triageClass}`;

    let badgeColor = s.triage === 'RED' ? 'badge-red' : (s.triage === 'YELLOW' ? 'badge-yellow' : 'badge-green');
    if (isRescued) badgeColor = 'badge-blue';

    const statusLabel = isRescued ? 'SAFELY EVACUATED' : (isDiscovered ? 'LOCATED // PENDING USAR' : 'SEARCH PENDING');

    card.innerHTML = `
      <div class="surv-header">
        <span class="surv-name">${s.name}</span>
        <span class="surv-badge ${badgeColor}">${isRescued ? 'RESCUED' : s.triage}</span>
      </div>
      <div class="surv-sub">
        <span>📍 Grid: (${s.x.toFixed(1)}m, ${s.y.toFixed(1)}m) &bull; ${statusLabel}</span>
      </div>
      <div class="surv-vitals-row">
        <span>🌡️ ${s.temp_c}°C</span>
        <span>❤️ ${s.heart_rate_bpm} BPM</span>
        <span>🔊 ${s.sound_detected ? `${s.sound_db} dB` : 'Silent'}</span>
      </div>
      <div class="surv-actions">
        ${!isRescued ? `
          <button class="btn-card-action btn-dispatch-team" onclick="dispatchRescue('${s.id}')">DISPATCH USAR</button>
          <button class="btn-card-action btn-card-rescued" onclick="markRescued('${s.id}')">MARK RESCUED</button>
        ` : `
          <div style="font-size: 9px; color: #38bdf8; padding: 2px;">✓ Evacuated to Base Staging Area</div>
        `}
      </div>
    `;
    container.appendChild(card);
  });
}

// --- Render Sparkline Error Chart ---
function renderErrorChart(odomErrors, ekfErrors) {
  if (!odomErrors || odomErrors.length < 2) return;

  const w = errorCanvas.width;
  const h = errorCanvas.height;
  errCtx.clearRect(0, 0, w, h);

  const maxVal = Math.max(5.0, ...odomErrors, ...ekfErrors);

  // Background grid
  errCtx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
  errCtx.lineWidth = 1;
  errCtx.beginPath();
  errCtx.moveTo(0, h / 2);
  errCtx.lineTo(w, h / 2);
  errCtx.stroke();

  // Draw Raw Odometry curve (Red)
  drawLine(odomErrors, '#ef4444', 1.8, maxVal, w, h);
  // Draw EKF Fused curve (Cyan)
  drawLine(ekfErrors, '#06b6d4', 2.2, maxVal, w, h);
}

function drawLine(points, color, width, maxVal, w, h) {
  errCtx.strokeStyle = color;
  errCtx.lineWidth = width;
  errCtx.beginPath();
  const step = w / (points.length - 1);
  points.forEach((val, i) => {
    const x = i * step;
    const y = h - (val / maxVal) * (h - 8) - 4;
    if (i === 0) errCtx.moveTo(x, y);
    else errCtx.lineTo(x, y);
  });
  errCtx.stroke();
}

// --- Watchdog Terminal ---
function updateWatchdogTerminal(alerts) {
  const terminal = document.getElementById('watchdogTerminal');
  if (!alerts || !terminal) return;

  terminal.innerHTML = '';
  alerts.forEach(a => {
    const div = document.createElement('div');
    const lvlClass = a.level === 'WARN' ? 'log-warn' : (a.level === 'SUCCESS' ? 'log-success' : (a.level === 'WARNING' ? 'log-warn' : 'log-info'));
    div.className = `log-line ${lvlClass}`;
    div.innerHTML = `<span class="log-t">[${a.timestamp || ''} ${a.category}]</span> ${a.text}`;
    terminal.appendChild(div);
  });
  terminal.scrollTop = terminal.scrollHeight;
}

// --- Checkpoints List ---
function updateCheckpointsList(checkpoints, activeIdx, mode) {
  const container = document.getElementById('waypointsList');
  if (!checkpoints || !container) return;

  container.innerHTML = '';
  checkpoints.forEach((wp, idx) => {
    const item = document.createElement('div');
    const isActive = (idx === activeIdx) && (mode === 'PATROL' || mode === 'SAR_SWEEP');
    item.className = `wp-item ${isActive ? 'active' : ''}`;
    item.innerHTML = `
      <span class="wp-bullet">${isActive ? '▶' : '○'}</span>
      <span class="wp-name">${wp.name}</span>
      <span class="wp-coords">(${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})</span>
    `;
    item.addEventListener('click', () => {
      sendWsCommand({ type: 'goal', x: wp.x, y: wp.y });
    });
    container.appendChild(item);
  });
}

// ==========================================================================
// 60 FPS Canvas Radar Render Loop
// ==========================================================================
function renderLoop() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  if (staticMapData) {
    drawGrid();
    drawDropoutZone(staticMapData.dropout_zone);
    drawBunkerObstacles();
    drawRubblePiles(staticMapData.rubble_piles || []);
    drawHazards(staticMapData.hazards || []);
    drawCheckpoints(staticMapData.checkpoints || []);
  }

  if (telemetryData) {
    if (telemetryData.planned_path) {
      drawPlannedPath(telemetryData.planned_path);
    }

    if (activeRescueRoute) {
      drawRescueIngressRoute(activeRescueRoute);
    }

    if (showTrails && telemetryData.trails) {
      drawTrails(telemetryData.trails);
    }

    if (showLidar && telemetryData.lidar) {
      drawLidarRays(telemetryData.lidar, telemetryData.gt);
    }

    // Draw Survivors on Map
    if (telemetryData.sar && telemetryData.sar.survivors) {
      drawSurvivorsOnRadar(telemetryData.sar.survivors);
    }

    // Ghost Odometry (drifted)
    drawGhostOdomRobot(telemetryData.odom);

    // EKF Covariance Uncertainty Ellipse
    drawCovarianceEllipse(telemetryData.ekf);

    // Camera FOV Viewing Cone
    drawCameraCone(telemetryData.gt);

    // Live Chassis (Ground Truth)
    drawUgvChassis(telemetryData.gt);
  }

  requestAnimationFrame(renderLoop);
}

// 1. Grid & Coordinates
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
  }

  // Crosshair center (0, 0) Base Station
  const center = worldToScreen(0, 0);
  ctx.strokeStyle = 'rgba(6, 182, 212, 0.4)';
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

  ctx.fillStyle = '#38bdf8';
  ctx.font = '10px JetBrains Mono';
  ctx.fillText('BASE STAGING', center.x + 16, center.y - 6);

  ctx.restore();
}

// 2. Optical Dropout / Smoke Zone
function drawDropoutZone(zone) {
  if (!zone) return;
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

  ctx.fillStyle = '#f87171';
  ctx.font = '10px JetBrains Mono';
  ctx.fillText('⚠ DENSE SMOKE & COLLAPSE ZONE', p1.x + 8, p1.y + 16);
  ctx.restore();
}

// 3. Bunker Walls & Pillars
function drawBunkerObstacles() {
  ctx.save();
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 3;
  ctx.shadowColor = 'rgba(56, 189, 248, 0.3)';
  ctx.shadowBlur = 8;

  if (staticMapData.walls) {
    staticMapData.walls.forEach(w => {
      const p1 = worldToScreen(w[0], w[1]);
      const p2 = worldToScreen(w[2], w[3]);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    });
  }
  ctx.shadowBlur = 0;

  if (staticMapData.pillars) {
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

      ctx.fillStyle = '#94a3b8';
      ctx.font = '9px JetBrains Mono';
      ctx.fillText(`P${idx + 1}`, sc.x - 6, sc.y + 3);
    });
  }
  ctx.restore();
}

// 4. Disaster Rubble Piles
function drawRubblePiles(rubble) {
  ctx.save();
  rubble.forEach(([rx, ry, radius], idx) => {
    const sc = worldToScreen(rx, ry);
    const r = radius * viewScale;

    // Draw rubble jagged cluster
    ctx.fillStyle = 'rgba(71, 85, 105, 0.85)';
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.5;

    ctx.beginPath();
    ctx.arc(sc.x, sc.y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#cbd5e1';
    ctx.font = '8px JetBrains Mono';
    ctx.fillText(`RUBBLE`, sc.x - 14, sc.y + 3);
  });
  ctx.restore();
}

// 5. Fire & Gas Hazards
function drawHazards(hazards) {
  const t = Date.now() * 0.003;
  ctx.save();

  hazards.forEach(haz => {
    const sc = worldToScreen(haz.x, haz.y);
    const pulse = Math.sin(t + haz.x) * 4;
    const r = haz.radius * viewScale + pulse;

    if (haz.temp_c > 100) {
      // Fire glow
      const grad = ctx.createRadialGradient(sc.x, sc.y, 2, sc.x, sc.y, r);
      grad.addColorStop(0, 'rgba(255, 200, 0, 0.85)');
      grad.addColorStop(0.5, 'rgba(239, 68, 68, 0.55)');
      grad.addColorStop(1, 'rgba(185, 28, 28, 0.05)');

      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(sc.x, sc.y, r, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#ffedd5';
      ctx.font = '9px JetBrains Mono';
      ctx.fillText(`🔥 ${haz.temp_c}°C`, sc.x - 18, sc.y - r - 4);
    } else {
      // Collapse danger zone
      ctx.strokeStyle = 'rgba(245, 158, 11, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 3]);
      ctx.beginPath();
      ctx.arc(sc.x, sc.y, r, 0, Math.PI * 2);
      ctx.stroke();

      ctx.fillStyle = '#fbbf24';
      ctx.font = '9px JetBrains Mono';
      ctx.fillText(`⚠️ COLLAPSE`, sc.x - 24, sc.y - r - 4);
    }
  });
  ctx.restore();
}

// 6. Checkpoints
function drawCheckpoints(checkpoints) {
  ctx.save();
  checkpoints.forEach((wp) => {
    const sc = worldToScreen(wp.x, wp.y);

    ctx.fillStyle = '#a855f7';
    ctx.strokeStyle = '#c084fc';
    ctx.lineWidth = 1.5;

    ctx.beginPath();
    ctx.arc(sc.x, sc.y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#e2e8f0';
    ctx.font = '9px JetBrains Mono';
    ctx.fillText(wp.id, sc.x + 7, sc.y + 3);
  });
  ctx.restore();
}

// 7. Survivors on Radar
function drawSurvivorsOnRadar(survivors) {
  const t = Date.now() * 0.005;
  ctx.save();

  survivors.forEach(s => {
    const sc = worldToScreen(s.x, s.y);
    const pulseR = 8 + Math.sin(t + s.x) * 3;

    let color = s.triage === 'RED' ? '#ef4444' : (s.triage === 'YELLOW' ? '#f59e0b' : '#10b981');
    if (s.rescued) color = '#06b6d4';

    // Pulsing outer ring
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, pulseR + 4, 0, Math.PI * 2);
    ctx.stroke();

    // Center dot / cross
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(sc.x, sc.y, 5, 0, Math.PI * 2);
    ctx.fill();

    // Label
    ctx.fillStyle = '#ffffff';
    ctx.font = '9px JetBrains Mono';
    const tag = s.rescued ? `✓ ${s.id}` : `${s.id} (${s.triage})`;
    ctx.fillText(tag, sc.x + 12, sc.y + 3);
  });

  ctx.restore();
}

// 8. Rescuer Ingress Route
function drawRescueIngressRoute(route) {
  if (!route || route.length < 2) return;
  ctx.save();
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 3;
  ctx.setLineDash([8, 4]);

  ctx.beginPath();
  route.forEach((pt, i) => {
    const sc = worldToScreen(pt[0], pt[1]);
    if (i === 0) ctx.moveTo(sc.x, sc.y);
    else ctx.lineTo(sc.x, sc.y);
  });
  ctx.stroke();

  // Ingress label
  const mid = worldToScreen(route[Math.floor(route.length / 2)][0], route[Math.floor(route.length / 2)][1]);
  ctx.fillStyle = '#38bdf8';
  ctx.font = '10px JetBrains Mono';
  ctx.fillText('🚑 USAR EXTRACTION CORRIDOR', mid.x + 8, mid.y - 6);
  ctx.restore();
}

// 9. Camera Viewing Cone
function drawCameraCone(robot) {
  const origin = worldToScreen(robot.x, robot.y);
  const fov = (72.0 * Math.PI) / 180.0;
  const range = 6.5 * viewScale;

  const leftAngle = -robot.theta - fov / 2.0;
  const rightAngle = -robot.theta + fov / 2.0;

  ctx.save();
  ctx.fillStyle = 'rgba(6, 182, 212, 0.08)';
  ctx.strokeStyle = 'rgba(6, 182, 212, 0.35)';
  ctx.lineWidth = 1;

  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.arc(origin.x, origin.y, range, leftAngle, rightAngle);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  ctx.restore();
}

// 10. A* Planned Path
function drawPlannedPath(path) {
  if (!path || path.length < 2) return;
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

// 11. Trajectory Trails
function drawTrails(trails) {
  drawSingleTrail(trails.gt, '#10b981', 2, []);
  drawSingleTrail(trails.odom, '#ef4444', 1.8, [5, 3]);
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

// 12. LiDAR Rays
function drawLidarRays(lidar, robot) {
  const origin = worldToScreen(robot.x, robot.y);
  ctx.save();

  lidar.forEach(beam => {
    const hit = worldToScreen(beam.x, beam.y);
    ctx.strokeStyle = 'rgba(234, 179, 8, 0.16)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(hit.x, hit.y);
    ctx.stroke();

    ctx.fillStyle = '#eab308';
    ctx.beginPath();
    ctx.arc(hit.x, hit.y, 2, 0, Math.PI * 2);
    ctx.fill();
  });
  ctx.restore();
}

// 13. Ghost Odometry Robot
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

  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(rw * 0.7, 0);
  ctx.stroke();

  ctx.restore();
}

// 14. EKF Covariance Uncertainty Ellipse
function drawCovarianceEllipse(ekf) {
  const sc = worldToScreen(ekf.x, ekf.y);
  const ellipse = ekf.ellipse;
  const rx = ellipse.rx * viewScale;
  const ry = ellipse.ry * viewScale;

  ctx.save();
  ctx.translate(sc.x, sc.y);
  ctx.rotate(-ellipse.angle);

  ctx.strokeStyle = 'rgba(6, 182, 212, 0.75)';
  ctx.lineWidth = 1.5;
  ctx.fillStyle = 'rgba(6, 182, 212, 0.12)';

  ctx.beginPath();
  ctx.ellipse(0, 0, Math.max(rx, 4), Math.max(ry, 4), 0, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.restore();
}

// 15. Live UGV Chassis
function drawUgvChassis(robot) {
  const sc = worldToScreen(robot.x, robot.y);
  ctx.save();
  ctx.translate(sc.x, sc.y);
  ctx.rotate(-robot.theta);

  const rw = 0.60 * viewScale;
  const rh = 0.45 * viewScale;
  const wheelW = 0.22 * viewScale;
  const wheelH = 0.10 * viewScale;

  // Left & Right Wheels
  ctx.fillStyle = '#0f172a';
  ctx.strokeStyle = '#475569';
  ctx.lineWidth = 1.5;
  ctx.fillRect(-rw * 0.3, -rh * 0.65, wheelW, wheelH);
  ctx.strokeRect(-rw * 0.3, -rh * 0.65, wheelW, wheelH);
  ctx.fillRect(-rw * 0.3, rh * 0.65 - wheelH, wheelW, wheelH);
  ctx.strokeRect(-rw * 0.3, rh * 0.65 - wheelH, wheelW, wheelH);

  // Chassis body
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

  ctx.beginPath();
  ctx.moveTo(rw * 0.65, 0);
  ctx.lineTo(rw * 0.5, -4);
  ctx.lineTo(rw * 0.5, 4);
  ctx.closePath();
  ctx.fill();

  ctx.restore();
}
