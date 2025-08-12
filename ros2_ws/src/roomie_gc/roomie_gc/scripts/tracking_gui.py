#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from roomie_msgs.msg import Tracking

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
)


class TrackingPublisher(Node):
    def __init__(self):
        super().__init__('tracking_gui_publisher')
        self.pub = self.create_publisher(Tracking, '/vs/tracking', QoSProfile(depth=10))

    def publish_event(self, event: int):
        msg = Tracking()
        msg.id = 0
        msg.event = event
        self.pub.publish(msg)
        name = {0: 'NONE', 1: 'SLOW_DOWN', 2: 'LOST', 3: 'RESUME'}.get(event, str(event))
        self.get_logger().info(f'Publish tracking: event={event}({name})')


class TrackingGUI(QWidget):
    def __init__(self, ros_node: TrackingPublisher):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle('VS Tracking Test GUI')
        layout = QVBoxLayout()
        layout.addWidget(QLabel('Publish /vs/tracking events'))

        btn_none = QPushButton('NONE (0)')
        btn_slow = QPushButton('SLOW_DOWN (1)')
        btn_lost = QPushButton('LOST (2)')
        btn_resume = QPushButton('RESUME (3)')

        btn_none.clicked.connect(lambda: self.node.publish_event(0))
        btn_slow.clicked.connect(lambda: self.node.publish_event(1))
        btn_lost.clicked.connect(lambda: self.node.publish_event(2))
        btn_resume.clicked.connect(lambda: self.node.publish_event(3))

        layout.addWidget(btn_none)
        layout.addWidget(btn_slow)
        layout.addWidget(btn_lost)
        layout.addWidget(btn_resume)

        self.setLayout(layout)


def main():
    rclpy.init()
    node = TrackingPublisher()
    app = QApplication(sys.argv)
    gui = TrackingGUI(node)
    gui.show()

    # Spin rclpy in a basic way using a Qt timer
    from PyQt5.QtCore import QTimer
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


