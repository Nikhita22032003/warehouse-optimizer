import streamlit as st
import time
import matplotlib.pyplot as plt
from src.simulation import simulate_multi_robot_streamlit

st.title("🚀 Warehouse Robot Optimizer")

# Load frames only once
if "frames" not in st.session_state:
    st.session_state.frames = simulate_multi_robot_streamlit()

# Restart button
if st.button("Restart Simulation"):
    st.session_state.frames = simulate_multi_robot_streamlit()

placeholder = st.empty()

# ✅ Smooth animation WITHOUT rerun
for fig in st.session_state.frames:
    placeholder.pyplot(fig)
    plt.close(fig)
    time.sleep(0.2)