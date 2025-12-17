import streamlit as st
import requests

def save_config(data):
    requests.post("http://localhost:8000/adas", json=data)


def load_config():
    r = requests.get("http://localhost:8000/adas")
    return r.json()

adas_value = load_config()

def generate_options():
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
        new_coll = st.checkbox("Anti-Collision", value=adas_value["Collision"])
        adas_value["Collision"]= new_coll
    with col2:
        st.image("../pictures/ACC.jpg", caption="Airbag", use_container_width=True)
        new_abg = st.checkbox("Airbag Deployment", value=adas_value["Airbag"])
        adas_value["Airbag"]= new_abg
    with col3:
        st.image("../pictures/LDW.jpg", caption="ESP", use_container_width=True)
        new_esp = st.checkbox("ESP (Trajectory Control Assistance )", value=adas_value["ESP"])
        adas_value["ESP"]= new_esp
    
    save_config(adas_value)



