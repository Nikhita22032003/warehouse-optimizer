import streamlit as st
import time
from src.simulation import simulate_multi_robot_streamlit

st.title("🚀 Warehouse Robot Optimizer")

st.write("Multi-robot warehouse simulation using A* pathfinding")

if st.button("Run Simulation"):

    placeholder = st.empty()

    frames = simulate_multi_robot_streamlit()

    for fig in frames:
        placeholder.pyplot(fig)
        time.sleep(0.2)