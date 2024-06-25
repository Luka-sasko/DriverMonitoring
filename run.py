import os
import subprocess
import sys

# Determine if the script is running as a PyInstaller bundle
if getattr(sys, 'frozen', False):
    script_dir = sys._MEIPASS
else:
    script_dir = os.path.dirname(os.path.abspath(__file__))

# Path to the main Streamlit script
streamlit_script = os.path.join(script_dir, "main.py")

# Command to run Streamlit
command = f"streamlit run {streamlit_script}"

# Run the command in a new process without opening a new console window
subprocess.Popen(command, shell=True, stdin=None, stdout=None, stderr=None, close_fds=True)
