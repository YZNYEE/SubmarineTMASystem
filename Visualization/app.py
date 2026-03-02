import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QDockWidget, QStatusBar, QToolBar,
                               QLabel, QPushButton, QGroupBox, QFormLayout, 
                               QLineEdit, QComboBox, QCheckBox, QTabWidget,
                               QSlider, QDial)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QIcon
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Polygon
import numpy as np
from platform import Platform

class TrajectoryView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0) # 去除布局边距
        layout.setSpacing(0)
        
        self.figure = Figure(figsize=(5, 4), dpi=100)
        # 调整子图布局，使其填满整个 Figure 区域
        self.figure.subplots_adjust(left=0, right=1, top=1, bottom=0)
        
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        
        self.ax = self.figure.add_subplot(111)
        
        # 初始化平台对象
        self.observer = Platform(x=0, y=0, speed=10, heading=0) # 己方
        self.target = Platform(x=5000, y=5000, speed=15, heading=270) # 目标
        
        # 记录轨迹点
        self.obs_traj_x = [self.observer.x]
        self.obs_traj_y = [self.observer.y]
        self.tgt_traj_x = [self.target.x]
        self.tgt_traj_y = [self.target.y]
        
        self.setup_plot()

    def update_simulation(self, dt=1.0):
        """推进仿真一步"""
        self.observer.update(dt)
        self.target.update(dt)
        
        # 记录轨迹
        self.obs_traj_x.append(self.observer.x)
        self.obs_traj_y.append(self.observer.y)
        self.tgt_traj_x.append(self.target.x)
        self.tgt_traj_y.append(self.target.y)
        
        self.setup_plot()

    def create_wedge_marker(self, x, y, heading, size=500, color='blue'):
        """
        创建一个楔形 (三角形) 标记，尖端指向航向
        Args:
            x, y: 中心坐标
            heading: 航向角 (度), 0=北, 90=东
            size: 大小 (从中心到顶点的距离)
        """
        # 转换为数学角度 (0度=东, 逆时针增加)
        # 我们的 heading: 0=北(Y+), 90=东(X+) -> math_angle = 90 - heading
        rad = np.radians(90 - heading)
        
        # 定义三角形顶点 (相对于中心)
        # 顶点1 (尖端): 指向航向
        tip = (size * np.cos(rad), size * np.sin(rad))
        
        # 底部宽度半角
        half_angle = np.radians(140) # 尾部开角
        
        # 顶点2 (左后)
        p2 = (size * 0.6 * np.cos(rad + half_angle), size * 0.6 * np.sin(rad + half_angle))
        
        # 顶点3 (右后)
        p3 = (size * 0.6 * np.cos(rad - half_angle), size * 0.6 * np.sin(rad - half_angle))
        
        # 绝对坐标
        vertices = [
            (x + tip[0], y + tip[1]),
            (x + p2[0], y + p2[1]),
            (x + p3[0], y + p3[1])
        ]
        
        return Polygon(vertices, closed=True, color=color, alpha=0.8, ec='black')

    def setup_plot(self):
        self.ax.clear()
        
        # 1. 设置整个绘图区背景为浅蓝 (#E0F0FF)
        self.figure.patch.set_facecolor('#E0F0FF')
        self.ax.set_facecolor('#E0F0FF')
        
        # 2. 设置原点在正中心
        limit = 10000 
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        
        # 3. 关闭坐标轴显示 (但保留绘图逻辑)
        self.ax.axis('off')
        
        # 4. 强制宽高比一致 (确保正方形区域)
        self.ax.set_aspect('equal')
        
        # 绘制十字准星
        self.ax.plot([-limit, limit], [0, 0], color='white', linestyle='--', linewidth=0.5, alpha=0.5)
        self.ax.plot([0, 0], [-limit, limit], color='white', linestyle='--', linewidth=0.5, alpha=0.5)
        
        # 绘制历史轨迹 (虚线)
        if len(self.obs_traj_x) > 1:
            self.ax.plot(self.obs_traj_x, self.obs_traj_y, color='blue', linestyle=':', linewidth=1, alpha=0.6)
        if len(self.tgt_traj_x) > 1:
            self.ax.plot(self.tgt_traj_x, self.tgt_traj_y, color='red', linestyle=':', linewidth=1, alpha=0.6)
        
        # 5. 绘制平台 (楔形图标)
        # 己方 (蓝色)
        obs_wedge = self.create_wedge_marker(
            self.observer.x, self.observer.y, self.observer.heading, 
            size=600, color='blue'
        )
        self.ax.add_patch(obs_wedge)
        self.ax.text(self.observer.x + 800, self.observer.y, "Observer", color='blue', fontsize=9)
        
        # 目标 (红色)
        tgt_wedge = self.create_wedge_marker(
            self.target.x, self.target.y, self.target.heading, 
            size=600, color='red'
        )
        self.ax.add_patch(tgt_wedge)
        self.ax.text(self.target.x + 800, self.target.y, "Target", color='red', fontsize=9)
        
        self.canvas.draw()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("TMA Analysis Platform")
        self.resize(1200, 800)
        
        # 仿真定时器
        self.sim_timer = QTimer(self)
        self.sim_timer.setInterval(1000) # 1000ms = 1s
        self.sim_timer.timeout.connect(self.run_simulation_step)
        
        # 1. 设置中心部件 (绘图区域占位)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # 使用 TabWidget 来管理不同的视图 (例如: 2D 轨迹, 误差分析, 原始数据)
        self.view_tabs = QTabWidget()
        self.main_layout.addWidget(self.view_tabs)
        
        # 示例 Tab 1: 轨迹视图
        self.tab_trajectory = TrajectoryView()
        self.view_tabs.addTab(self.tab_trajectory, "Trajectory View")
        
        # 示例 Tab 2: 分析图表
        self.tab_analysis = QWidget()
        self.view_tabs.addTab(self.tab_analysis, "Error Analysis")
        
        # 2. 设置菜单栏
        self.create_menus()
        
        # 3. 设置工具栏
        self.create_toolbar()
        
        # 4. 设置状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")
        
        # 5. 设置停靠窗口 (控制面板)
        self.create_dock_panels()
        
    def create_menus(self):
        menu_bar = self.menuBar()
        
        # File Menu
        file_menu = menu_bar.addMenu("&File")
        exit_action = QAction("E&xit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # View Menu
        view_menu = menu_bar.addMenu("&View")
        # (Actions to toggle docks can be added here)
        
        # Help Menu
        help_menu = menu_bar.addMenu("&Help")
        about_action = QAction("&About", self)
        help_menu.addAction(about_action)

    def create_toolbar(self):
        toolbar = QToolBar("Main Toolbar")
        self.addToolBar(toolbar)
        
        self.run_action = QAction("Run", self)
        self.run_action.setStatusTip("Run Simulation")
        self.run_action.triggered.connect(self.start_simulation)
        toolbar.addAction(self.run_action)
        
        self.stop_action = QAction("Stop", self)
        self.stop_action.setStatusTip("Stop Simulation")
        self.stop_action.triggered.connect(self.stop_simulation)
        self.stop_action.setEnabled(False) # Initially disabled
        toolbar.addAction(self.stop_action)
        
        toolbar.addSeparator()
        
        # 仿真速度选择
        toolbar.addWidget(QLabel(" Sim Speed: "))
        self.combo_speed = QComboBox()
        self.combo_speed.addItems(["1x", "2x", "5x", "10x"])
        self.combo_speed.currentTextChanged.connect(self.update_sim_speed)
        toolbar.addWidget(self.combo_speed)
        
        toolbar.addSeparator()
        
        settings_action = QAction("Settings", self)
        toolbar.addAction(settings_action)

    def update_sim_speed(self, text):
        """Update simulation timer interval based on selection"""
        # Speed 1x -> 1000ms, 2x -> 500ms, 5x -> 200ms, 10x -> 100ms
        speed = int(text.replace('x', ''))
        interval = int(1000 / speed)
        
        self.sim_timer.setInterval(interval)
        # 如果正在运行，无需重启，setInterval 会在下一次生效 (PySide6特性)
        # 但为了立即响应，可以重启一下
        if self.sim_timer.isActive():
            self.sim_timer.start()

    def start_simulation(self):
        """Start the simulation timer"""
        # Get current speed setting
        speed_text = self.combo_speed.currentText()
        speed = int(speed_text.replace('x', ''))
        interval = int(1000 / speed)
        self.sim_timer.start(interval)
        
        self.run_action.setEnabled(False)
        self.stop_action.setEnabled(True)
        self.status_bar.showMessage(f"Simulation Running ({speed}x)...")
        
    def stop_simulation(self):
        """Stop the simulation timer"""
        self.sim_timer.stop()
        self.run_action.setEnabled(True)
        self.stop_action.setEnabled(False)
        self.status_bar.showMessage("Simulation Stopped.")

    def run_simulation_step(self):
        """Execute one simulation step"""
        # Advance simulation by 1 second
        self.tab_trajectory.update_simulation(dt=1.0)
        
        # Log status (optional)
        current_time = len(self.tab_trajectory.obs_traj_x) - 1
        # self.log_message(f"Step {current_time}: Obs={self.tab_trajectory.observer.position}, Tgt={self.tab_trajectory.target.position}")

    def create_dock_panels(self):
        # --- Configuration Dock ---
        self.config_dock = QDockWidget("Configuration", self)
        self.config_dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        
        dock_content = QWidget()
        dock_layout = QVBoxLayout(dock_content)
        
        # 1. Observer Settings Group (Real-time Control)
        obs_group = QGroupBox("Observer Control (Real-time)")
        obs_layout = QVBoxLayout()
        
        # Speed Control
        speed_layout = QHBoxLayout()
        speed_label = QLabel("Speed (m/s):")
        self.slider_obs_speed = QSlider(Qt.Horizontal)
        self.slider_obs_speed.setRange(0, 50) # 0-50 m/s
        self.slider_obs_speed.setValue(10)
        self.input_obs_speed = QLineEdit("10")
        self.input_obs_speed.setFixedWidth(40)
        self.input_obs_speed.setReadOnly(True) # Read-only, control via slider
        
        speed_layout.addWidget(speed_label)
        speed_layout.addWidget(self.slider_obs_speed)
        speed_layout.addWidget(self.input_obs_speed)
        obs_layout.addLayout(speed_layout)
        
        # Heading Control (Using Dial for intuitive rotation)
        heading_layout = QHBoxLayout()
        heading_label = QLabel("Heading (deg):")
        self.dial_obs_heading = QDial()
        self.dial_obs_heading.setRange(0, 359)
        self.dial_obs_heading.setValue(0)
        self.dial_obs_heading.setNotchesVisible(True)
        self.dial_obs_heading.setWrapping(True) # Allow 359 -> 0 transition
        
        self.input_obs_heading = QLineEdit("0")
        self.input_obs_heading.setFixedWidth(40)
        self.input_obs_heading.setReadOnly(True)
        
        heading_layout.addWidget(heading_label)
        heading_layout.addWidget(self.dial_obs_heading)
        heading_layout.addWidget(self.input_obs_heading)
        obs_layout.addLayout(heading_layout)
        
        # Connect signals
        self.slider_obs_speed.valueChanged.connect(self.update_observer_speed)
        self.dial_obs_heading.valueChanged.connect(self.update_observer_heading)
        
        obs_group.setLayout(obs_layout)
        dock_layout.addWidget(obs_group)
        
        # 2. Target Settings Group
        target_group = QGroupBox("Target State (Initial)")
        target_layout = QFormLayout()
        self.input_tgt_x = QLineEdit("5000.0")
        self.input_tgt_y = QLineEdit("5000.0")
        self.input_tgt_speed = QLineEdit("15.0")
        self.input_tgt_heading = QLineEdit("270.0")
        
        target_layout.addRow("Pos X (m):", self.input_tgt_x)
        target_layout.addRow("Pos Y (m):", self.input_tgt_y)
        target_layout.addRow("Speed (m/s):", self.input_tgt_speed)
        target_layout.addRow("Heading (deg):", self.input_tgt_heading)
        target_group.setLayout(target_layout)
        dock_layout.addWidget(target_group)
        
        # 3. Algorithm Settings Group
        algo_group = QGroupBox("Algorithm Selection")
        algo_layout = QVBoxLayout()
        self.combo_algo = QComboBox()
        self.combo_algo.addItems(["MLE (Maximum Likelihood)", "UKF (Unscented Kalman)", "PF (Particle Filter)", "EKELUND"])
        algo_layout.addWidget(self.combo_algo)
        
        self.check_crlb = QCheckBox("Show CRLB")
        self.check_crlb.setChecked(True)
        algo_layout.addWidget(self.check_crlb)
        
        algo_group.setLayout(algo_layout)
        dock_layout.addWidget(algo_group)
        
        # 4. Action Buttons (Mainly for Target Reset/Apply)
        btn_layout = QHBoxLayout()
        self.btn_apply = QPushButton("Apply Target")
        self.btn_reset = QPushButton("Reset All")
        btn_layout.addWidget(self.btn_apply)
        btn_layout.addWidget(self.btn_reset)
        dock_layout.addLayout(btn_layout)
        
        # Connect buttons
        self.btn_apply.clicked.connect(self.apply_settings)
        self.btn_reset.clicked.connect(self.reset_settings)
        
        dock_layout.addStretch() # Push everything up
        
        self.config_dock.setWidget(dock_content)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.config_dock)

        # --- Log/Output Dock (Bottom) ---
        self.log_dock = QDockWidget("System Log", self)
        self.log_dock.setAllowedAreas(Qt.BottomDockWidgetArea)
        self.log_widget = QWidget() # Placeholder for log text area
        self.log_dock.setWidget(self.log_widget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)

    def update_observer_speed(self, value):
        """Real-time update for observer speed"""
        self.input_obs_speed.setText(str(value))
        self.tab_trajectory.observer.speed = float(value)
        # Immediate visual feedback (optional, maybe too heavy if dragged fast, but for simple plot it's fine)
        # self.tab_trajectory.setup_plot() 

    def update_observer_heading(self, value):
        """Real-time update for observer heading"""
        self.input_obs_heading.setText(str(value))
        self.tab_trajectory.observer.heading = float(value)
        # Immediate visual feedback: rotate the wedge even if paused
        self.tab_trajectory.setup_plot()

    def apply_settings(self):
        """Apply the configuration (Target mainly)"""
        try:
            # Target (Apply only when button clicked)
            tgt_x = float(self.input_tgt_x.text())
            tgt_y = float(self.input_tgt_y.text())
            tgt_speed = float(self.input_tgt_speed.text())
            tgt_heading = float(self.input_tgt_heading.text())
            
            self.tab_trajectory.target.x = tgt_x
            self.tab_trajectory.target.y = tgt_y
            self.tab_trajectory.target.speed = tgt_speed
            self.tab_trajectory.target.heading = tgt_heading
            
            # Redraw
            self.tab_trajectory.setup_plot()
            
            self.status_bar.showMessage("Target Settings Applied Successfully", 3000)
            
        except ValueError:
            self.status_bar.showMessage("Error: Invalid input values.", 5000)

    def reset_settings(self):
        """Reset to default values"""
        # Reset Observer Controls
        self.slider_obs_speed.setValue(10)
        self.dial_obs_heading.setValue(0)
        
        # Reset Target Inputs
        self.input_tgt_x.setText("5000.0")
        self.input_tgt_y.setText("5000.0")
        self.input_tgt_speed.setText("15.0")
        self.input_tgt_heading.setText("270.0")
        
        self.apply_settings()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Optional: Set style
    app.setStyle("Fusion")
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())
