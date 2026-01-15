import streamlit as st
import adas_options, Dashboard

st.set_page_config(page_title="Tableau de bord", layout="wide")

page = st.sidebar.radio("Menu", ["Dashboard", "ADAS OPTIONS"], key="main_menu")



if page == "ADAS OPTIONS":
    adas_options.generate_options()

elif page == "Dashboard" :
    Dashboard.generate_dashboard()
