import streamlit as st
import requests
from pathlib import Path

# -------------------------
# Backend communication
# -------------------------
BACKEND_ADAS_URL = "http://localhost:8000/adas"

def save_config(data):
    """Send ADAS configuration to backend."""
    try:
        requests.post(BACKEND_ADAS_URL, json=data, timeout=0.2)
    except requests.exceptions.RequestException as e:
        st.error(f"Failed to save config: {e}")

def load_config():
    """Load ADAS configuration from backend."""
    try:
        r = requests.get(BACKEND_ADAS_URL, timeout=0.2)
        return r.json()
    except requests.exceptions.RequestException as e:
        st.error(f"Failed to load config: {e}")
        # Default fallback
        return {"Collision": False, "Airbag": False, "ESP": False}

# -------------------------
# ADAS Options UI
# -------------------------
def generate_options():
    # Initialize state
    if "adas_value" not in st.session_state:
        st.session_state.adas_value = load_config()

    # Page styling
    st.markdown("""
    <style>
    body, .stApp {
        margin: 0;
        padding: 0;
        height: auto;
        overflow: auto;
        background: linear-gradient(135deg, #0d0a36, #0b2555, #3b003a);
    }
    .stButton>button {
        background-color: #1E3A8A;
        color: white;
        font-weight: bold;
        padding: 8px 16px;
        border-radius: 6px;
        border: none;
        transition: background-color 0.3s ease;
    }
    .stButton>button:hover {
        background-color: #3B82F6;
    }
    </style>
    """, unsafe_allow_html=True)

    st.title("ADAS OPTIONS")
    st.write("Activate or configure the ADAS features: ACC, ESP, Airbag ...")

    # Define features with their image paths and labels
    features = [
        {"key": "Collision", "label": "Anti-Collision", "image": "../pictures/LCA.jpg", "caption": "Collision"},
        {"key": "Airbag", "label": "Airbag Deployment", "image": "../pictures/ACC.jpg", "caption": "Airbag"},
        {"key": "ESP", "label": "ESP (Trajectory Control Assistance)", "image": "../pictures/LDW.jpg", "caption": "ESP"}
    ]

    cols = st.columns(len(features))
    for col, feature in zip(cols, features):
        with col:
            # Use width=None to make image responsive (non-deprecated)
            img_path = Path(feature["image"])
            if img_path.exists():
                st.image(str(img_path), caption=feature["caption"], width='stretch')
            else:
                st.warning(f"Image not found: {feature['image']}")
            
            # Checkbox
            st.session_state.adas_value[feature["key"]] = st.checkbox(
                feature["label"],
                value=st.session_state.adas_value.get(feature["key"], False)
            )

    # Save button instead of saving automatically
    if st.button("Save ADAS Settings"):
        save_config(st.session_state.adas_value)
        st.success("ADAS settings saved successfully!")
