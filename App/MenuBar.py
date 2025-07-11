from PyQt6.QtWidgets import (
    QMenuBar, QMenu, QDialog, QVBoxLayout, QComboBox, QPushButton, QLabel
)
from PyQt6.QtGui import QAction  
from PyQt6.QtCore import QSize
import GUI

def Menubar(window):
    """Create and configure the menu bar."""
    menubar = QMenuBar(window)
    window.setMenuBar(menubar)

    # Account Menu
    account_menu = QMenu("Account", window)
    account_menu.addAction(QAction("Account", window))
    account_menu.addAction(QAction("Change Password", window))
    menubar.addMenu(account_menu)

    # Edit Menu
    edit_menu = QMenu("Edit", window)
    edit_menu.addAction(QAction("test", window))
    edit_menu.addAction(QAction("test", window))
    edit_menu.addSeparator()
    edit_menu.addAction(QAction("test", window))
    edit_menu.addAction(QAction("test", window))
    edit_menu.addAction(QAction("test", window))
    menubar.addMenu(edit_menu)

    # System Menu
    sys_menu = QMenu("System", window)

    resolution_action = QAction("Resolution", window)
    resolution_action.triggered.connect(lambda: ResolutionWindow(window))
    sys_menu.addAction(resolution_action)

    # Mode Submenu
    mode_menu = QMenu("Mode", window)

    mode_menu.addAction(QAction("Full Screen", window, triggered=lambda: set_fullscreen(window)))
    mode_menu.addAction(QAction("Windowed", window, triggered=lambda: set_windowed(window)))
    mode_menu.addAction(QAction("Borderless", window, triggered=lambda: set_borderless(window)))

    sys_menu.addMenu(mode_menu)

    sys_menu.addAction(QAction("Forced Exit", window, triggered=lambda: window.close()))
    menubar.addMenu(sys_menu)

def ResolutionWindow(parent):
    """Create a dialog for selecting resolution."""
    dialog = QDialog(parent)
    dialog.setWindowTitle("Select Resolution")
    dialog.setFixedSize(250, 200)

    layout = QVBoxLayout()

    resolutions = [
        "1920x1080", "1600x900", "1440x900", "1366x768",
        "1280x720", "1024x768", "800x600", "640x480"
    ]

    label = QLabel("Choose Resolution:")
    layout.addWidget(label)

    resolution_menu = QComboBox()
    resolution_menu.addItems(resolutions)
    resolution_menu.setCurrentText(GUI.DEFAULT_RESOLUTION)
    layout.addWidget(resolution_menu)

    def validate_resolution():
        res = resolution_menu.currentText()
        if 'x' in res:
            width, height = res.split('x')
            if width.isdigit() and height.isdigit():
                GUI.update_resolution(parent, res)
                dialog.accept()
                return
        resolution_menu.setCurrentText(GUI.DEFAULT_RESOLUTION)

    apply_button = QPushButton("Apply")
    apply_button.clicked.connect(validate_resolution)
    layout.addWidget(apply_button)

    close_button = QPushButton("Exit")
    close_button.clicked.connect(dialog.reject)
    layout.addWidget(close_button)

    dialog.setLayout(layout)
    dialog.exec()

def set_fullscreen(window):
    """Set the window to full screen mode."""
    window.showFullScreen()

def set_windowed(window):
    """Set the window to windowed mode."""
    window.showNormal()
    window.resize(QSize(1280, 720))  # Or use current size if needed

def set_borderless(window):
    """Set the window to borderless mode."""
    window.setWindowFlags(window.windowFlags() | window.FramelessWindowHint)
    window.show()