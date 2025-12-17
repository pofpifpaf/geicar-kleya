import streamlit as st
import requests

def save_config(data):
    """Send ADAS configuration to backend."""
    try:
        requests.post("http://localhost:8000/adas", json=data, timeout=1)
    except requests.exceptions.RequestException as e:
        st.error(f"Failed to save config: {e}")

def load_config():
    """Load ADAS configuration from backend."""
    try:
        r = requests.get("http://localhost:8000/adas", timeout=1)
        return r.json()
    except requests.exceptions.RequestException as e:
        st.error(f"Failed to load config: {e}")
        # Return default values if backend is unreachable
        return {"Collision": False, "Airbag": False, "ESP": False}



def generate_options():

    if "adas_value" not in st.session_state:
        st.session_state.adas_value = load_config()

    st.markdown("""
    <style>
    body, .stApp {
        margin: 0;
        padding: 0;
        height: auto;
        overflow: auto;
        background: linear-gradient(135deg, #0d0a36, #0b2555, #3b003a);
    }
    </style>
    """, unsafe_allow_html=True)
    st.title("ADAS OPTIONS")
    st.write("Activate or configure the ADAS features : ACC, ESP, Airbag ...")

    col1, col2, col3 = st.columns(3)
    with col1:
        st.image("../pictures/LCA.jpg", caption="Collision", use_container_width=True)
        new_coll = st.checkbox("Anti-Collision", value=st.session_state.adas_value["Collision"])
        st.session_state.adas_value["Collision"]= new_coll
    with col2:
        st.image("../pictures/ACC.jpg", caption="Airbag", use_container_width=True)
        new_abg = st.checkbox("Airbag Deployment", value=st.session_state.adas_value["Airbag"])
        st.session_state.adas_value["Airbag"]= new_abg
    with col3:
        st.image("../pictures/LDW.jpg", caption="ESP", use_container_width=True)
        new_esp = st.checkbox("ESP (Trajectory Control Assistance )", value=st.session_state.adas_value["ESP"])
        st.session_state.adas_value["ESP"]= new_esp
    
    save_config(st.session_state.adas_value)



