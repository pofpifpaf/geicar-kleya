import streamlit as st
from streamlit_extras.switch_page_button import switch_page
import json


def save_config(data, path="../data/data.json"):
    with open(path, "w") as f:
        json.dump(data, f, indent=4)

def load_config(path="../data/data.json"):
    with open(path, "r") as f:
        return json.load(f)
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
    st.write("Activate or configure the ADAS features : LCA, ACC, LDW ...")

    col1, col2, col3 = st.columns(3)
    with col1:
        st.image("../pictures/LCA.jpg", caption="LCA", use_container_width=True)
        new_lca = st.checkbox("LCA (Lane Centering Assist)", value=adas_value["LCA"])
        adas_value["LCA"]= new_lca
    with col2:
        st.image("../pictures/ACC.jpg", caption="ACC", use_container_width=True)
        new_acc = st.checkbox("ACC (Adaptive Cruise Control)", value=adas_value["ACC"])
        adas_value["ACC"]= new_acc
    with col3:
        st.image("../pictures/LDW.jpg", caption="LDW", use_container_width=True)
        new_ldw = st.checkbox("LDW (Lane Departure Warning)", value=adas_value["LDW"])
        adas_value["LDW"]= new_ldw
    
    save_config(adas_value)



