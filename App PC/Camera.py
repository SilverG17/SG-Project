from PyQt6.QtWidgets import (
    QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QFileDialog, QMessageBox, QWidget, QGridLayout
)
from PyQt6.QtGui import QPixmap, QImage
from PyQt6.QtCore import Qt, QTimer
import cv2
from PIL import Image
from datetime import datetime
from pyfirmata2 import Arduino
import Viettrix

destPath = ''
imagePath = ''
led_pin = None
board = None

class CameraWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)
        self.initUI()

    def initUI(self):
        layout = QGridLayout(self)

        self.cameraLabel = QLabel("Camera Feed")
        self.cameraLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.cameraLabel, 0, 0, 1, 2)

        capture_btn = QPushButton("Capture")
        capture_btn.clicked.connect(self.capture_image)
        layout.addWidget(capture_btn, 1, 0)

        back_btn = QPushButton("Back")
        back_btn.clicked.connect(self.go_back)
        layout.addWidget(back_btn, 1, 1)

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.cameraLabel.setPixmap(pixmap.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio))

    def capture_image(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Image", "", "JPEG Files (*.jpg)")
        if filename:
            ret, frame = self.cap.read()
            if ret:
                cv2.imwrite(filename, frame)
                QMessageBox.information(self, "Saved", f"Image saved to {filename}")

    def go_back(self):
        self.timer.stop()
        self.cap.release()
        self.parent().setCentralWidget(Viettrix.HomeWidget(self.parent()))

def createwidgets(window: QWidget):
    # Setup layout
    layout = QGridLayout()
    window.setLayout(layout)

    # Webcam feed label
    window.cameraLabel = QLabel("WEBCAM FEED")
    window.cameraLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
    layout.addWidget(window.cameraLabel, 0, 0, 1, 2)

    # Image preview label
    window.imageLabel = QLabel("IMAGE PREVIEW")
    window.imageLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
    layout.addWidget(window.imageLabel, 0, 2, 1, 2)

    # Buttons
    browse_btn = QPushButton("Browse")
    browse_btn.clicked.connect(lambda: imageBrowse(window))
    layout.addWidget(browse_btn, 1, 3)

    capture_btn = QPushButton("Capture")
    capture_btn.clicked.connect(lambda: Capture(window))
    layout.addWidget(capture_btn, 2, 0)

    stop_btn = QPushButton("Stop Camera")
    stop_btn.clicked.connect(lambda: StopCAM(window))
    layout.addWidget(stop_btn, 2, 1)

    back_btn = QPushButton("Back")
    back_btn.clicked.connect(lambda: BackToHome(window))
    layout.addWidget(back_btn, 2, 3)

    # Arrow buttons
    up_btn = QPushButton("UP")
    up_btn.clicked.connect(led_on)
    layout.addWidget(up_btn, 3, 2)

    down_btn = QPushButton("DOWN")
    down_btn.clicked.connect(led_off)
    layout.addWidget(down_btn, 4, 2)

    left_btn = QPushButton("LEFT")
    left_btn.clicked.connect(led_on)
    layout.addWidget(left_btn, 4, 1)

    right_btn = QPushButton("RIGHT")
    right_btn.clicked.connect(led_off)
    layout.addWidget(right_btn, 4, 3)

    # Start camera
    window.cap = cv2.VideoCapture(0)
    window.timer = QTimer()
    window.timer.timeout.connect(lambda: ShowFeed(window))
    window.timer.start(30)

def ShowFeed(window):
    ret, frame = window.cap.read()
    if ret:
        frame = cv2.flip(frame, 1)
        cv2.putText(frame, datetime.now().strftime('%d/%m/%Y %H:%M:%S'),
                    (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        window.cameraLabel.setPixmap(pixmap.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio))

def imageBrowse(window):
    global imagePath
    file, _ = QFileDialog.getOpenFileName(window, "Select Image")
    if file:
        imagePath = file
        image = Image.open(file).resize((640, 480))
        qt_image = QPixmap(file)
        window.imageLabel.setPixmap(qt_image.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio))

def Capture(window):
    global destPath
    if not destPath:
        destPath = QFileDialog.getExistingDirectory(window, "Select Save Directory")
        if not destPath:
            QMessageBox.critical(window, "Error", "No directory selected!")
            return

    image_name = datetime.now().strftime('%d-%m-%Y %H-%M-%S')
    img_path = f"{destPath}/{image_name}.jpg"
    ret, frame = window.cap.read()
    if ret:
        cv2.putText(frame, datetime.now().strftime('%d/%m/%Y %H:%M:%S'),
                    (430, 460), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))
        cv2.imwrite(img_path, frame)
        window.imageLabel.setPixmap(QPixmap(img_path).scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio))
        QMessageBox.information(window, "Success", f"Image saved to {img_path}")

def led_on():
    global led_pin, board
    try:
        if not board:
            board = Arduino('COM3')
        if not led_pin:
            led_pin = board.get_pin('d:13:o')
        led_pin.write(1)
    except Exception as e:
        QMessageBox.critical(None, "Arduino Error", str(e))

def led_off():
    global led_pin, board
    try:
        if not board:
            board = Arduino('COM3')
        if not led_pin:
            led_pin = board.get_pin('d:13:o')
        led_pin.write(0)
    except Exception as e:
        QMessageBox.critical(None, "Arduino Error", str(e))

def StopCAM(window):
    window.timer.stop()
    window.cap.release()
    window.cameraLabel.setText("OFF CAM")

def BackToHome(window):
    window.timer.stop()
    window.cap.release()
    for i in reversed(range(window.layout().count())):
        widget = window.layout().itemAt(i).widget()
        if widget:
            widget.setParent(None)
    Viettrix.HomeWidget(window)