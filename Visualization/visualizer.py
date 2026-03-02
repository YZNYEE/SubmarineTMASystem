import sys
import numpy as np
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QHBoxLayout, QLabel, QLineEdit
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.patches as patches

class TMAVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TMA (Target Motion Analysis) Visualizer")
        self.setGeometry(100, 100, 1200, 800)

        # Main Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Left Panel: Controls
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_panel.setFixedWidth(250)
        
        # Add some controls
        control_layout.addWidget(QLabel("Algorithm Settings"))
        
        self.noise_input = QLineEdit("0.5")
        control_layout.addWidget(QLabel("Bearing Noise (deg):"))
        control_layout.addWidget(self.noise_input)
        
        self.btn_run = QPushButton("Run Simulation")
        self.btn_run.clicked.connect(self.run_simulation)
        control_layout.addWidget(self.btn_run)
        
        control_layout.addStretch()
        main_layout.addWidget(control_panel)

        # Right Panel: Plot
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("TMA Scenario")
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")
        self.ax.grid(True)
        
        main_layout.addWidget(self.canvas)

        # Initial Plot
        self.plot_dummy_data()

    def plot_dummy_data(self):
        self.ax.clear()
        self.ax.set_title("TMA Scenario (Demo)")
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")
        self.ax.grid(True)

        # Observer Path
        obs_x = np.linspace(0, 5000, 100)
        obs_y = np.zeros_like(obs_x)
        self.ax.plot(obs_x, obs_y, 'b-', label='Observer')
        self.ax.plot(obs_x[-1], obs_y[-1], 'bo') # Current pos

        # Target Path
        tgt_x = np.linspace(5000, 4000, 100)
        tgt_y = np.linspace(8000, 7500, 100)
        self.ax.plot(tgt_x, tgt_y, 'r--', label='True Target')
        self.ax.plot(tgt_x[-1], tgt_y[-1], 'rx') # Current pos

        # Estimated Position with Error Ellipse
        est_x = tgt_x[-1] + np.random.randn() * 100
        est_y = tgt_y[-1] + np.random.randn() * 100
        self.ax.plot(est_x, est_y, 'g*', markersize=10, label='Estimate')

        # Draw Ellipse
        ellipse = patches.Ellipse((est_x, est_y), width=1000, height=300, angle=45, 
                                  edgecolor='g', facecolor='none', linestyle='--')
        self.ax.add_patch(ellipse)

        self.ax.legend()
        self.canvas.draw()

    def run_simulation(self):
        # Here you would call your C++ backend or load data
        print("Running simulation with noise:", self.noise_input.text())
        self.plot_dummy_data()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TMAVisualizer()
    window.show()
    sys.exit(app.exec())
