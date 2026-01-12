import streamlit as st
import Home, adas_options, contactus, Dashboard

st.set_page_config(page_title="Tableau de bord", layout="wide")

if 'go_to_dashboard' not in st.session_state:
    st.session_state['go_to_dashboard'] = False


page = st.sidebar.radio("Menu", ["Home", "Dashboard", "ADAS OPTIONS","contact us"], key="main_menu")

if st.session_state['go_to_dashboard']:
    page = "Dashboard"
    st.session_state['go_to_dashboard'] = False


if page == "Home" :
    Home.generate_home()
  
    if st.button("Start live drive"):
        st.session_state['go_to_dashboard'] = True
        st.rerun()


elif page == "ADAS OPTIONS":
  adas_options.generate_options()

elif page == "Dashboard" :
    Dashboard.generate_dashboard()

elif page == "contact us":
    contactus.generate_contact()