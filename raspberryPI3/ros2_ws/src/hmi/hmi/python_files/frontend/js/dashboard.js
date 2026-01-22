const CAR_IP = "10.105.1.169";
const ws = new WebSocket(`ws://${CAR_IP}:8000/ws/state`);

// ===== Utils =====
function clamp(x, a, b){ return Math.max(a, Math.min(b, x)); }

// Convertit un angle (deg) sur un cercle en coord cartésiennes
function polarToCart(cx, cy, r, angleDeg){
  const a = (Math.PI / 180) * angleDeg;
  return { x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) };
}

// Génère le path d’un arc (semi-cercle)
// SVG arc helper.
// sweepFlag: 0 = anti-horaire, 1 = horaire
function describeArc(cx, cy, r, startAngle, endAngle, sweepFlag = 0){
  const start = polarToCart(cx, cy, r, startAngle);
  const end = polarToCart(cx, cy, r, endAngle);

  const delta = Math.abs(endAngle - startAngle);
  const largeArcFlag = delta <= 180 ? "0" : "1";

  return `M ${start.x} ${start.y} A ${r} ${r} 0 ${largeArcFlag} ${sweepFlag} ${end.x} ${end.y}`;
}


/**
 * Gauge semi-circulaire : start=180° (gauche) -> end=0° (droite)
 * percent: 0..1
 */
function renderSemiGauge(containerId, percent, labelLeft="0", labelRight="100"){
  const w = 360, h = 220;

  // Centre du cercle proche du bas du SVG
  const cx = w / 2;
  const cy = h - 20;

  // Rayon adapté pour rester dans le viewBox
  const r = 120;


  // Demi-cercle du haut : gauche (180°) -> droite (360°)
  const start = 180;
  const end = 360;

  const p = clamp(percent, 0, 1);
  const prog = start + (end - start) * p;

  // IMPORTANT : sweepFlag = 1 pour prendre le bon chemin (arc du haut)
  const bgPath = describeArc(cx, cy, r, start, end, 1);
  const fgPath = describeArc(cx, cy, r, start, prog, 1);

  const svg = `
    <svg width="${w}" height="${h}" viewBox="0 0 ${w} ${h}" xmlns="http://www.w3.org/2000/svg">
      <defs>
        <linearGradient id="${containerId}_grad" x1="0%" y1="0%" x2="100%" y2="0%">
          <stop offset="0%" stop-color="#4EE6FF"/>
          <stop offset="100%" stop-color="#6EF2B0"/>
        </linearGradient>
      </defs>

      <path d="${bgPath}" fill="none" stroke="rgba(255,255,255,0.08)" stroke-width="10" stroke-linecap="round"/>
      <path d="${fgPath}" fill="none" stroke="url(#${containerId}_grad)" stroke-width="10" stroke-linecap="round"/>

      <text x="20" y="${h-10}" fill="rgba(184,215,255,0.75)" font-size="12">${labelLeft}</text>
      <text x="${w-40}" y="${h-10}" fill="rgba(184,215,255,0.75)" font-size="12">${labelRight}</text>
    </svg>
  `;

  const el = document.getElementById(containerId);
  if (el) el.innerHTML = svg;
}


// ===== UI update =====
function setIndicator(id, active){
  const el = document.getElementById(id);
  if (!el) return;
  if (active) el.classList.add("active");
  else el.classList.remove("active");
}

function setCollisionBanner(state){
  const banner = document.getElementById("collisionBanner");
  if (!banner) return;

  // mapping basé sur ton Dashboard.py :contentReference[oaicite:1]{index=1}
  let text = "";
  if (state === "state_slow") text = "⚠️ Obstacle near in front";
  else if (state === "state_stop") text = "⚠️ Obstacle close : Stopping";
  else if (state === "state_rear") text = "⚠️ Obstacle behind : Stopping";

  if (text){
    banner.textContent = text;
    banner.style.display = "block";
  } else {
    banner.style.display = "none";
  }
}

function isIndicatorActive(id){
  const el = document.getElementById(id);
  return !!(el && el.classList.contains("active"));
}

function setAirbagOverlay(airbagState){
  const overlay = document.getElementById("airbagOverlay");
  if (!overlay) return;

  const airbagLampOn = isIndicatorActive("indABG"); // voyant airbag (ADAS)
  const deployed = (airbagState === "state_deployed");

  overlay.style.display = (deployed && airbagLampOn) ? "flex" : "none";
}

// WS events for connection status
const pill = document.getElementById("connPill");
const connText = document.getElementById("connText");

if (pill && connText){
  connText.textContent = "Connecting...";
  pill.classList.add("connecting");
}

ws.onopen = () => {
  if (connText) connText.textContent = "Connected";
  if (pill){
    pill.classList.remove("connecting", "disconnected");
    pill.classList.add("connected");
  }
};

ws.onclose = () => {
  if (connText) connText.textContent = "Disconnected";
  if (pill){
    pill.classList.remove("connected", "connecting");
    pill.classList.add("disconnected");
  }
};

ws.onerror = () => {
  if (connText) connText.textContent = "Disconnected";
  if (pill){
    pill.classList.remove("connected", "connecting");
    pill.classList.add("disconnected");
  }
};




ws.onmessage = (event) => {
  const data = JSON.parse(event.data);

  // Valeurs brutes
  const speed = Number(data.speed ?? 0);
  const rpm = Number(data.RPM ?? 0);
  const batt = Number(data.battery ?? 0);
  const temp = Number(data.temperature ?? 0);
  const press = Number(data.pressure ?? 0);

  // Update texte
  const elSpeed = document.getElementById("speed");
  const elRpm = document.getElementById("rpm");
  const elBatt = document.getElementById("battery");
  const elTemp = document.getElementById("temperature");
  const elPress = document.getElementById("pressure");

  if (elSpeed) elSpeed.innerText = speed.toFixed(0);
  if (elRpm) elRpm.innerText = rpm.toFixed(0);
  if (elBatt) elBatt.innerText = clamp(batt, 0, 100).toFixed(0);
  if (elTemp) elTemp.innerText = temp.toFixed(0);
  if (elPress) elPress.innerText = press.toFixed(0);

  // Jauges (0..1)
  renderSemiGauge("batteryGauge", clamp(batt,0,100) / 100, "0", "100");
  // rpm : adapte le max à ton système (ici 0..3000)
  const RPM_max = 100 ;
  renderSemiGauge("rpmGauge", clamp(rpm,0,RPM_max) / RPM_max, "0", "100");

  // Alertes
  window.__lastAirbagState = data.airbag_state;   // <--- ajoute ça
  setCollisionBanner(data.collision_state);
  setAirbagOverlay(data.airbag_state);
};

// Rendu initial (avant 1er message)
renderSemiGauge("batteryGauge", 0, "0", "100");
renderSemiGauge("rpmGauge", 0, "0", "100");

// ===== Voyants cliquables (ADAS options dans l'index) =====
function setIndicator(id, active){
  const el = document.getElementById(id);
  if (!el) return;
  if (active) el.classList.add("active");
  else el.classList.remove("active");
}


// ----- ADAS config management ----
// Mapping voyants UI -> clés backend /adas (comme adas.js)
const INDICATORS = {
  indESP: "esp",
  indFCTA: "collision",
  indABG: "airbag", // si ABG correspond à Airbag chez toi
  indLCA: "lca"
};

// Charge l'état initial depuis le backend (GET /adas)
async function loadAdasConfig(){
  try{
    const r = await fetch(`http://${CAR_IP}:8000/adas`);
    if (!r.ok) return;
    const cfg = await r.json();

    // applique au visuel
    for (const [id, key] of Object.entries(INDICATORS)){
      setIndicator(id, !!cfg[key]);
    }
  } catch(e){
    // silencieux si pas dispo
  }
}

// Sauvegarde l'état courant (POST /adas)
async function saveAdasConfig(){
  try{
    // construit l'objet à partir du visuel
    const body = {};
    for (const [id, key] of Object.entries(INDICATORS)){
      const el = document.getElementById(id);
      body[key] = !!(el && el.classList.contains("active"));
    }

    await fetch(`http://${CAR_IP}:8000/adas`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(body)
    });
  } catch(e){
    // silencieux
  }
}

// Active le clic sur chaque voyant
function setupIndicatorClicks(){
  for (const id of Object.keys(INDICATORS)){
    const el = document.getElementById(id);
    if (!el) continue;

    el.addEventListener("click", async () => {
      el.classList.toggle("active");
      // met à jour l’overlay immédiatement selon l'état courant reçu
      setAirbagOverlay(window.__lastAirbagState);
      await saveAdasConfig();
    });
  }
}

// init
setupIndicatorClicks();
loadAdasConfig();
