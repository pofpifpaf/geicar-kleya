import streamlit as st
from math import radians, cos, sin
import time
import requests

from streamlit_autorefresh import st_autorefresh
   
@st.cache_data(ttl=0.5)
def get_state():
    try:
        r = requests.get("http://localhost:8000/state", timeout=0.3)
        return r.json()
    except:
        return None

@st.cache_data(ttl=0.5)
def get_adas():
    try:
        r = requests.get("http://localhost:8000/adas", timeout=0.3)
        return r.json()
    except:
        return None


def svg_mini_gauge(width=120, height=100, percent=0.5, ticks=5, primary_color="#FFD36E"):
    # On peut juste réutiliser svg_semi_gauge avec des dimensions réduites
    return svg_semi_gauge(width=width, height=height, percent=percent, ticks=ticks, primary_color=primary_color)

def polar_to_cart(cx, cy, r, angle):
    a = radians(angle)
    return cx + r * cos(a), cy - r * sin(a)

def svg_semi_gauge(width=450, height=350, percent=0.5, ticks=10, primary_color="#8BE38B"):
    cx, cy = width / 2, height
    r = min(width * 0.42, height * 0.85)
    start_angle, end_angle = 180, 0

    start_x, start_y = polar_to_cart(cx, cy, r, start_angle)
    end_x, end_y = polar_to_cart(cx, cy, r, end_angle)
    prog_angle = start_angle - (start_angle - end_angle) * percent
    prog_x, prog_y = polar_to_cart(cx, cy, r, prog_angle)

    ticks_svg, labels_svg = "", ""
    for i in range(ticks + 1):
        ang = start_angle - (start_angle - end_angle) * (i / ticks)
        x1, y1 = polar_to_cart(cx, cy, r * 0.92, ang)
        x2, y2 = polar_to_cart(cx, cy, r * 1.02, ang)
        lx, ly = polar_to_cart(cx, cy, r * 0.78, ang)
        val = int(i * 100 / ticks)
        ticks_svg += f'<line class="tick" x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke-width="2"/>'
        labels_svg += f'<text x="{lx:.2f}" y="{ly:.2f}" text-anchor="middle" fill="#4EE6FF" font-size="14">{val}</text>'

    grad_id = f"g{int(time.time()*1000)%100000}"
    return f"""
    <svg width="{width}" height="{height}" viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">
      <defs>
        <linearGradient id="{grad_id}" x1="0%" y1="0%" x2="100%" y2="0%">
          <stop offset="0%" stop-color="#4EE6FF"/>
          <stop offset="100%" stop-color="{primary_color}"/>
        </linearGradient>
      </defs>
      <path class="arc-bg" d="M {start_x} {start_y} A {r} {r} 0 0 1 {end_x} {end_y}"/>
      {ticks_svg}
      {labels_svg}
      <path class="arc-fill" stroke="url(#{grad_id})" d="M {start_x} {start_y} A {r} {r} 0 0 1 {prog_x} {prog_y}"/>
    </svg>
    """
def generate_dashboard_html(shared_data, adas_data, power_val=120):
    # power_norm = min(power_val / 200.0, 1.0)
    batt_norm = min(shared_data["battery"] / 100.0, 1.0)
    rpm_norm = min(shared_data["RPM"] / 100.0, 1.0)
    # fuel_norm = min(fuel_percent / 10000.0, 1.0)
    # temperature_norm = min(temperature_value/100, 1.0)
    # pressure_norm =  min(pressure_value/100, 1.0)
    if shared_data["collision_state"] == "state_slow":
      collision_html = "<div class='collisionmessages'>⚠️ Obstacle near in front </div>"
    elif shared_data["collision_state"] == "state_stop" :
      collision_html = "<div class='collisionmessages'>⚠️ Obstacle close : Stopping </div>"
    elif shared_data["collision_state"] == "state_rear" :
       collision_html = "<div class='collisionmessages'>⚠️ Obstacle behind : Stopping </div>"
    else:
      collision_html = ""
        
    left_gauge = f"""
    <div class="gauge-container" style="position:relative;">
      <div class="gauge-label-top">Battery</div>
      {svg_semi_gauge(percent=batt_norm, primary_color="#6EF2B0")}

      <div class="gauge-label-bottom">{shared_data["battery"]:.0f}% battery</div>

    </div>

    
    """

    right_gauge = f"""
    
    <div class="gauge-container" style="position:relative;">
    <div class="gauge-container right-gauge-container" style="position:relative;">
      <div class="gauge-label-top"> RPM x100 </div>
      {svg_semi_gauge(percent=rpm_norm, primary_color="#6EE6FF")}

      <div class="">{shared_data["RPM"]:.0f} RPM</div>
    </div>
    </div>
    {collision_html}
  
    """
    
    # Centre et status inchangés
    center_html = f"""
    <div class="center-panel">
      <div style="color:#4EE6FF; font-size:18px;">Hold My Wheel</div>
      <div class="speed-value">{int(shared_data["speed"]):02d}</div>
      <div class="unit">km/h</div>    
      <div class="generaltempandpressure">    
        <div class="tempandpressure">🌡️ : {shared_data["temperature"] :.0f} °C </div>
      </div>
    </div>
    """
    

    airbag_overlay = '<div class="airbag-overlay">⚠ AIRBAG DEPLOYED ⚠</div>' if shared_data["airbag_state"] == "state_deployed" else ""
    # ldw_overlay = '<div class="ldw-overlay">⚠ Lane Departure Left</div>' if shared_data["ldw_state"] else ""
    ldw_overlay = ""

    def indicator(label, active):
        cls = "indicator active" if active else "indicator"
        return f'<div class="{cls}">{label}</div>'
    
    status_html = f"""
    <div class="status-row">
      {indicator("ESP", adas_data["ESP"])}
      {indicator("FCTA", adas_data["Collision"])}
      {indicator("ABG", adas_data["Airbag"])}
    </div>
    """
    return f"""
    <div class="main-container">
      <div class="dashboard">
        {airbag_overlay}
        {ldw_overlay}
        {left_gauge}
        {center_html}
        {right_gauge}
        {status_html}
      </div>
    </div>
    """

def generate_dashboard():

  if "shared_data" not in st.session_state:
    st.session_state.shared_data = {
        "speed": 0,
        "RPM": 0,
        "battery": 0,
        "pressure": 0,
        "temperature": 0,
        "airbag_state": "None",
        "collision_state": "None",
        "esp_state": "None",
    }

  if "adas_data" not in st.session_state:
      st.session_state.adas_data = {
          "Collision": False,
          "ESP": False,
          "Airbag": False,
      }

  state = get_state()
  if state:
    st.session_state.shared_data.update(state)

  adas = get_adas()
  if adas:
    st.session_state.adas_data.update(adas)
  
  st.markdown("""
    <style>
    body, .stApp {
    margin: 0;
    padding: 0;
    height: 100vh;
    background: linear-gradient(135deg, #0d0a36, #0b2555, #3b003a);
    overflow: hidden;
    }

    .block-container {
    padding: 0 !important;
    margin: 0 !important;
    }

    .main-container {
    width:100vw; height:100vh;
    display:flex; justify-content:center; align-items:center;
    background-image: 
    repeating-linear-gradient(to right, rgba(0,180,255,0.15) 0, rgba(0,180,255,0.15) 2px, transparent 2px, transparent 12px),
    repeating-linear-gradient(to bottom, rgba(0,180,255,0.15) 0, rgba(0,180,255,0.15) 2px, transparent 2px, transparent 12px);
    }

    .dashboard {
    width:95%; height:95%;
    border-radius:0;
    background:linear-gradient(180deg, rgba(2,8,18,0.95), rgba(6,12,25,0.95));
    border:1px solid rgba(120,160,255,0.06);
    box-shadow:0 10px 30px rgba(0,0,0,0.8), inset 0 1px 0 rgba(255,255,255,0.02);
    display:grid;
    grid-template-columns:1fr 1fr 1fr;
    gap:0 !important; padding:0 !important;
    align-items:center;
    position:relative;
    }

    .right-gauge-container {
    grid-column: 3;
    }
              
    .gauge-container {
    display:flex;
    flex-direction:column;
    align-items:center;
    justify-content:center;
    }

    .gauge-label-top {
    color:#ffffff;
    font-weight:bold;
    font-size:18px;
    text-align:center;
    margin-bottom:-140px;
    }

    .gauge-label-bottom {
    color:#b8d7ff;
    opacity:0.85;
    font-size:16px;
    text-align:center;
    margin-top:-10px;
    }
              
    .collisionmessages {
    grid-column: 3;
    justify-self: end;
    margin-right: 35px;  
    font-size: 20px;
    opacity:0.85;
    text-align:center;
    margin-top:10px;       /* distance depuis le texte au-dessus */
    position:relative;
    width: fit-content;
    min-width: unset;
    max-width: fit-content;
    display: inline-block;
      padding:3px 8px;
      background:rgb(184, 77, 0);
      border:2px solid rgba(255,255,255,0.5);
      border-radius:5px;
      backdrop-filter:blur(5px);
      font-weight:bold;
      color:#ffffff;
      text-shadow:0 0 0px black;
      z-index:860;
      display:flex;
      justify-content:center;
      align-items:center;
      animation:popupBlink 2s infinite;
    }

    .tick { stroke:rgba(255,255,255,0.09); stroke-linecap:round; }
    .arc-bg { stroke:rgba(255,255,255,0.06); stroke-width:2.4; fill:none; stroke-linecap:round; }
    .arc-fill { stroke-linecap:round; stroke-width:5; fill:none; }

    .center-panel {
    display:flex; flex-direction:column;
    align-items:center; justify-content:center;
    gap:6px; z-index:2; position:relative;
    }
    .speed-value { font-size:120px; font-weight:700; color:#fff; text-shadow:0 2px 18px rgba(0,160,255,0.5); }
    .unit { font-size:24px; color:#b8d7ff; opacity:0.85; }

    .generaltempandpressure{
      margin-top: 10px;        
      padding: 6px 10px;         
      display: inline-block;   
      text-align: center;
    } 
    .tempandpressure {
      font-size:14px;
      color:#b8d7ff;
      opacity:0.85;
      line-height:18px;
      margin:0;
      padding:0;
    }

    .status-row {
    grid-column:1/4;
    display:flex;
    justify-content:center;
    gap:20px;
    font-size:18px;
    margin-top:20px;
    font-family:'Segoe UI', sans-serif;
    letter-spacing:1px;
    }

    .indicator {
    color:#5a5f68;
    text-shadow:0 0 4px rgba(255,255,255,0.05);
    font-weight:bold;
    transition:all 0.3s ease-in-out;
    user-select:none;
    }

    /* Voyants actifs (rouge néon) */
    .indicator.active {
    color:#ff3b3b;
    text-shadow:0 0 10px rgba(255,50,50,0.9), 0 0 25px rgba(255,0,0,0.6);
    animation:glowBlink 1.2s ease-in-out infinite alternate;
    }

    @keyframes glowBlink {
    0% { 
    opacity:0.6; 
    text-shadow:0 0 5px rgba(255,60,60,0.5); 
    }
    50% { 
    opacity:1; 
    text-shadow:0 0 20px rgba(255,50,50,1), 0 0 40px rgba(255,0,0,0.8); 
    }
    100% { 
    opacity:0.6; 
    text-shadow:0 0 5px rgba(255,60,60,0.5); 
    }
    }

    @keyframes flash {
    0% { background-color: rgba(255,0,0,0.1); }
    50% { background-color: rgba(255,0,0,0.9); }
    100% { background-color: rgba(255,0,0,0.1); }
    }
    .airbag-overlay {
    position:absolute;
    top:0; left:0;
    width:100%; height:100%;
    animation: flash 1s infinite;
    display:flex;
    justify-content:center;
    align-items:center;
    z-index:999;
    font-size:60px;
    font-weight:bold;
    color:white;
    text-shadow:0 0 25px black;
    }
  @keyframes popupBlink {
          0%   { opacity:0.5; transform:translateY(0); }
          50%  { opacity:1; transform:translateY(-4px); }
          100% { opacity:0.5; transform:translateY(0); }
      }              
  .ldw-overlay {
      position:relative;
      bottom:40px;
      left:40px;
      min-width:380px;
      padding:20px 28px;
      background:rgba(255,165,0,0.25);
      border:2px solid rgba(255,165,0,0.5);
      border-radius:14px;
      backdrop-filter:blur(4px);
      font-size:32px;
      font-weight:bold;
      color:orange;
      text-shadow:0 0 12px black;
      z-index:850;
      display:flex;
      justify-content:center;
      align-items:center;
      animation:popupBlink 1.1s infinite;
  }

      
      </style>
      """, unsafe_allow_html=True)

  html = generate_dashboard_html(
      shared_data=st.session_state.shared_data,
      adas_data=st.session_state.adas_data,
      power_val=120,
  )

  st.markdown(html, unsafe_allow_html=True)
  st_autorefresh(interval=800)
    