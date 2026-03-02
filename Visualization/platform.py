import math

class Platform:
    """
    平台类 (Platform)
    用于记录平台的二维坐标 (x, y)、速度 (speed) 和航向 (heading)。
    坐标系定义:
        - 北 (North) 为 Y 轴正方向 (Heading = 0°)
        - 东 (East) 为 X 轴正方向 (Heading = 90°)
        - 航向角顺时针增加
    """
    def __init__(self, x=0.0, y=0.0, speed=0.0, heading=0.0):
        """
        初始化平台状态。
        
        Args:
            x (float): 初始 X 坐标 (米)
            y (float): 初始 Y 坐标 (米)
            speed (float): 速度大小 (m/s)
            heading (float): 航向角 (度), 正北为 0 度
        """
        self.x = float(x)
        self.y = float(y)
        self.speed = float(speed)
        self.heading = float(heading)

    @property
    def position(self):
        """返回当前坐标 (x, y)"""
        return (self.x, self.y)

    @property
    def velocity(self):
        """
        计算速度分量 (vx, vy)。
        基于航向角 (0°=北, 90°=东):
        vx = speed * sin(heading)
        vy = speed * cos(heading)
        """
        rad = math.radians(self.heading)
        vx = self.speed * math.sin(rad)
        vy = self.speed * math.cos(rad)
        return (vx, vy)

    def update(self, dt):
        """
        根据当前速度和时间步长更新位置。
        
        Args:
            dt (float): 时间步长 (秒)
        """
        vx, vy = self.velocity
        self.x += vx * dt
        self.y += vy * dt

    def set_heading(self, heading):
        """设置航向角 (自动归一化到 0-360 度)"""
        self.heading = float(heading) % 360.0

    def set_speed(self, speed):
        """设置速度"""
        self.speed = float(speed)

    def __repr__(self):
        return f"Platform(x={self.x:.2f}, y={self.y:.2f}, speed={self.speed:.2f}, heading={self.heading:.2f})"
