# shared_state.py
import threading

# Set when a hand is detected
pause_event = threading.Event()