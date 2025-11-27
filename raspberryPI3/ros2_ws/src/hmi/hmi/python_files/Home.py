import streamlit as st
import base64



def get_base64(file_path):
    with open(file_path, "rb") as f:
        data = f.read()
    return base64.b64encode(data).decode()


def generate_home():

    IMAGE_PATH = "test.jpeg"
    IMAGE_PATH_2 = "page2.jpg"
    b64 = get_base64(IMAGE_PATH)
    b64_2 = get_base64(IMAGE_PATH_2)

    HOME_CSS = f"""
    <style>
    body, .stApp {{
    margin: 0;
    padding: 0;
    height: auto;
    overflow: auto;
    background: #000;
    }}

    .home-wrap {{
    width:100vw; height:100vh;
    display:flex;
    align-items:center;
    justify-content:center;
    background: url("data:image/jpeg;base64,{b64}") center/cover no-repeat fixed;
    position:relative;
    }}

    .home-wrap2 {{
    width:100vw; height:100vh;
    display:flex;
    align-items:center;
    justify-content:center;
    background: url("data:image/jpeg;base64,{b64_2}") center/cover no-repeat fixed;
    position:relative;
    }}

    .block-container {{
    padding: 0 !important;
    margin: 0 !important;
    }}
    .home-overlay {{
    background: rgba(0, 0, 0, 0.55);
    width:100%; height:100%;
    display:flex; flex-direction:column;
    align-items:center; justify-content:center;
    animation: fadeIn 2s ease-in-out;
    }}

    .home-title {{
    font-size:90px;
    color:#00BFFF;
    font-weight:900;
    text-shadow: 0 0 40px rgba(0,191,255,0.8), 0 0 80px rgba(0,128,255,0.6);
    margin-bottom:15px;
    }}

    .home-title2 {{
    font-size:60px;
    color:#00BFAA;
    font-weight:900;
    text-shadow: 0 0 40px rgba(0,191,255,0.8), 0 0 80px rgba(0,128,255,0.6);
    margin-bottom:15px;
    }}

    .home-sub {{
    font-size:22px;
    color:#e0f5ff;
    margin-bottom:30px;
    text-shadow:0 0 10px rgba(0,150,255,0.7);
    }}

    .stButton>button {{
    background: linear-gradient(90deg,#0078ff,#00d4ff) !important;
    color:white !important;
    padding:15px 36px !important;
    border-radius:100px !important;
    border:none !important;
    font-size:20px !important;
    cursor:pointer !important;
    box-shadow:0 0 25px rgba(0,160,255,0.5) !important;
    transition: all 0.3s ease !important;
    position: relative;  /* position absolue dans le conteneur */
    bottom: 1150px;        /* distance par rapport au bas */
    left: 410%;           /* centré horizontalement */
    transform: translateX(0%);  /* pour centrer parfaitement */
    }}

    .stButton>button:hover {{
    transform: translateX(-10%) scale(1.05) !important;
    box-shadow:0 0 40px rgba(0,180,255,0.8) !important;
    }}

    @keyframes fadeIn {{
    0% {{ opacity: 0; transform: scale(1.05); }}
    100% {{ opacity: 1; transform: scale(1); }}
    }}
    </style>
    """
    st.markdown(HOME_CSS, unsafe_allow_html=True)

    st.markdown(f"""
        <div class="home-wrap">
        <div class="home-overlay">
            <div class="home-title">CarConnect</div>
            <div class="home-sub">Get real time feedback about your car</div>
        </div>
        </div>
        """, unsafe_allow_html=True)
    st.markdown(f"""
        <div class="home-wrap2">
        <div class="home-overlay">
            <div class="home-title2">Explore Features</div>
            <div class="home-sub">Discover ADAS options and live feedback</div>
        </div>
        </div>
        """, unsafe_allow_html=True)