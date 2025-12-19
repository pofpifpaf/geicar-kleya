import streamlit as st
import requests

BACKEND_ADAS_URL = "http://localhost:8000/adas"

@st.cache_data(ttl=5)
def load_config():
    try:
        return requests.get(BACKEND_ADAS_URL, timeout=0.2).json()
    except requests.exceptions.RequestException:
        return {"Collision": False, "Airbag": False, "ESP": False}

def save_config(data):
    try:
        requests.post(BACKEND_ADAS_URL, json=data, timeout=0.2)
    except requests.exceptions.RequestException:
        st.error("Backend unreachable")

def generate_options():
    # --- Init once ---
    if "adas_value" not in st.session_state:
        st.session_state.adas_value = load_config()
        for k, v in st.session_state.adas_value.items():
            st.session_state[f"adas_{k}"] = v

    st.title("ADAS OPTIONS")
    st.write("Activate or configure ADAS features")

    st.checkbox("Anti-Collision", key="adas_Collision")
    st.checkbox("Airbag Deployment", key="adas_Airbag")
    st.checkbox("ESP (Trajectory Control Assistance)", key="adas_ESP")

    # Sync UI → config
    st.session_state.adas_value["Collision"] = st.session_state.adas_Collision
    st.session_state.adas_value["Airbag"] = st.session_state.adas_Airbag
    st.session_state.adas_value["ESP"] = st.session_state.adas_ESP

    st.divider()

    if st.button("Save ADAS Settings"):
        save_config(st.session_state.adas_value)
        st.success("Saved")

