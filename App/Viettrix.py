from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QGridLayout, QMainWindow
)
from PyQt6.QtGui import QPixmap, QIcon
from PyQt6.QtCore import Qt, QSize
import sys
import GUI, Camera  # Assuming these are PyQt6-compatible modules
from MenuBar import Menubar  # Assuming this sets up a QMenuBar
from PIL import Image

class HomeWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.initUI()

    def initUI(self):
        layout = QGridLayout()
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)

        # Load and resize shutdown icon
        shutdown_image = Image.open("icon/shutdown.png")
        shutdown_image = shutdown_image.resize((30, 30), Image.Resampling.LANCZOS)
        shutdown_image.save("icon/shutdown_resized.png")  # Save resized image for QPixmap
        shutdown_icon = QIcon("icon/shutdown_resized.png")

        # Shutdown button
        shutdown_button = QPushButton()
        shutdown_button.setIcon(shutdown_icon)
        shutdown_button.setIconSize(QSize(30, 30))
        shutdown_button.clicked.connect(self.shutdown)
        layout.addWidget(shutdown_button, 0, 2, alignment=Qt.AlignmentFlag.AlignRight)

        # Camera button
        camera_button = QPushButton("Camera")
        camera_button.clicked.connect(self.open_camera)
        layout.addWidget(camera_button, 0, 0)

        self.setLayout(layout)

    def shutdown(self):
        QApplication.instance().quit()

    def open_camera(self):
        self.clear_window()
        from Camera import CameraWidget
        self.parent().setCentralWidget(CameraWidget(self.parent()))

    def clear_window(self):
        for i in reversed(range(self.layout().count())):
            widget = self.layout().itemAt(i).widget()
            if widget:
                widget.setParent(None)

# Main application
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app_window = GUI.create_window()  # Should return a QMainWindow or QWidget
    Menubar(app_window)  # Assuming this sets up the menu bar
    home_widget = HomeWidget(app_window)
    app_window.setCentralWidget(home_widget)
    app_window.show()
    sys.exit(app.exec())