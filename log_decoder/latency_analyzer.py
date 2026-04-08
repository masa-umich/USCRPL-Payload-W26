import sys
import struct
import os
import numpy as np

# PyQt6 Imports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QFileDialog, QTextEdit)
from PyQt6.QtCore import Qt

# Matplotlib embedded in PyQt6
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure


class LatencyAnalyzerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teensy Telemetry Latency Analyzer")
        self.resize(1000, 800)
        
        # --- Create Main Layout ---
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # --- Top Controls ---
        controls_layout = QHBoxLayout()
        self.btn_load = QPushButton("Select .BIN File")
        self.btn_load.clicked.connect(self.load_file)
        
        self.lbl_file = QLabel("No file selected")
        self.lbl_file.setStyleSheet("color: gray; font-style: italic;")
        
        controls_layout.addWidget(self.btn_load)
        controls_layout.addWidget(self.lbl_file)
        controls_layout.addStretch() # Pushes the button to the left
        layout.addLayout(controls_layout)
        
        # --- Stats Output Console ---
        self.text_output = QTextEdit()
        self.text_output.setReadOnly(True)
        self.text_output.setMaximumHeight(180)
        self.text_output.setStyleSheet("font-family: Consolas, monospace; background-color: #f4f4f4;")
        layout.addWidget(self.text_output)
        
        # --- Matplotlib Canvas ---
        self.fig = Figure(figsize=(8, 5))
        self.canvas = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.canvas, self) # Adds zoom/pan tools
        
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        
        # Add the plot axis
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Waiting for data...")
        self.ax.grid(True, which="both", ls="-", alpha=0.2)
        
    def log(self, text):
        """Helper to print to the GUI text box."""
        self.text_output.append(text)
        QApplication.processEvents() # Force UI to update during heavy processing

    def load_file(self):
        """Opens a file dialog to select the BIN file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self, 
            "Select Telemetry File", 
            "", 
            "Binary Files (*.BIN);;All Files (*)"
        )
        
        if file_path:
            self.lbl_file.setText(file_path)
            self.lbl_file.setStyleSheet("color: black; font-weight: bold;")
            self.text_output.clear()
            self.analyze_telemetry(file_path)

    def analyze_telemetry(self, file_path):
        """Parses the binary file and updates the UI."""
        header_fmt = '<BBcB'
        sch_fmt = '<QI6h'
        adxl_fmt = '<QI3i'
        bno_fmt = '<QIB3f'

        sch_deltas, adxl_deltas = [], []
        sch_timestamps, adxl_timestamps = [], []

        self.log(f"Parsing file: {file_path}")
        self.log("Please wait, parsing binary data...")
        
        file_size = os.path.getsize(file_path)
        if file_size == 0:
            self.log("ERROR: File is empty.")
            return

        try:
            with open(file_path, "rb") as f:
                while True:
                    header_data = f.read(4)
                    if not header_data or len(header_data) < 4:
                        break
                    
                    sync1, sync2, packet_type, phase = struct.unpack(header_fmt, header_data)
                    
                    # Sliding window for sync bytes
                    if sync1 != 0xAA or sync2 != 0xBB:
                        f.seek(-3, os.SEEK_CUR)
                        continue
                        
                    packet_type = packet_type.decode('ascii', errors='ignore')

                    if packet_type == 'S':
                        payload = f.read(struct.calcsize(sch_fmt))
                        if len(payload) == struct.calcsize(sch_fmt):
                            timestamp, delta_t, _, _, _, _, _, _ = struct.unpack(sch_fmt, payload)
                            sch_timestamps.append(timestamp / 1e6)
                            sch_deltas.append(delta_t / 1e3)
                    elif packet_type == 'A':
                        payload = f.read(struct.calcsize(adxl_fmt))
                        if len(payload) == struct.calcsize(adxl_fmt):
                            timestamp, delta_t, _, _, _ = struct.unpack(adxl_fmt, payload)
                            adxl_timestamps.append(timestamp / 1e6)
                            adxl_deltas.append(delta_t / 1e3)
                    elif packet_type == 'B':
                        payload = f.read(struct.calcsize(bno_fmt))
                    else:
                        f.seek(-3, os.SEEK_CUR)
        except Exception as e:
            self.log(f"\nERROR reading file: {e}")
            return

        self.log("\n--- LATENCY REPORT ---")
        
        # Convert to numpy arrays, ignoring the first massive transition delta
        sch_d = np.array(sch_deltas[1:])
        adxl_d = np.array(adxl_deltas[1:])
        
        self.log(f"SCH16T Samples Found: {len(sch_d)}")
        if len(sch_d) > 0:
            self.log(f"  Expected delta: ~0.678 ms (1.475 kHz)")
            self.log(f"  Average delta:  {np.mean(sch_d):.3f} ms")
            self.log(f"  Max latency spike: {np.max(sch_d):.3f} ms")
            self.log(f"  Stalls > 2ms: {np.sum(sch_d > 2.0)} occurrences")

        self.log(f"\nADXL359 Samples Found: {len(adxl_d)}")
        if len(adxl_d) > 0:
            self.log(f"  Expected delta: 1.00 ms (1.0 kHz)")
            self.log(f"  Average delta:  {np.mean(adxl_d):.3f} ms")
            self.log(f"  Max latency spike: {np.max(adxl_d):.3f} ms")
            self.log(f"  Stalls > 2ms: {np.sum(adxl_d > 2.0)} occurrences")

        # --- Update the Plot ---
        self.ax.clear() # Clear the previous plot
        
        # Draw baselines
        self.ax.axhline(y=1.0, color='r', linestyle='--', alpha=0.5, label='ADXL Expected (1ms)')
        self.ax.axhline(y=0.678, color='b', linestyle='--', alpha=0.5, label='SCH Expected (0.678ms)')

        if len(adxl_timestamps) > 1:
            self.ax.plot(adxl_timestamps[1:], adxl_d, 'r.', markersize=2, alpha=0.7, label='ADXL359 Delta')
        if len(sch_timestamps) > 1:
            self.ax.plot(sch_timestamps[1:], sch_d, 'b.', markersize=2, alpha=0.7, label='SCH16T Delta')

        self.ax.set_yscale('log')
        self.ax.set_xlabel('Flight Time (Seconds)')
        self.ax.set_ylabel('Loop Latency / Delta T (Milliseconds)')
        self.ax.set_title('Teensy 4.0 Main Loop Profiler (SD Card Block Detection)')
        self.ax.legend(loc='upper right')
        self.ax.grid(True, which="both", ls="-", alpha=0.2)
        
        self.fig.tight_layout()
        self.canvas.draw() # Render the new plot


if __name__ == '__main__':
    # Required setup for any PyQt6 application
    app = QApplication(sys.argv)
    app.setStyle('Fusion') # Gives it a clean, modern look on Windows
    
    window = LatencyAnalyzerApp()
    window.show()
    
    # Start the event loop
    sys.exit(app.exec())