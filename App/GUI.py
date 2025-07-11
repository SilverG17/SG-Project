from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtCore import QSize

DEFAULT_RESOLUTION = QSize(1280, 720)

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Viettrix Camera App")
        self.resize(DEFAULT_RESOLUTION)
        self.showMaximized()  # Equivalent to state('zoomed')

def create_window():
    return Window()