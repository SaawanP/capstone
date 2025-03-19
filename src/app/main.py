import sys
import os
from PySide6.QtCore import Qt, QPoint
from PySide6.QtWidgets import QApplication, QMainWindow
from ui.new_interface import Ui_MainWindow  # Import the generated UI file
from src.gui_functionality import GuiFunctions
from src.backend import Backend
from Custom_Widgets import *
from Custom_Widgets.QAppSettings import QAppSettings
import _icons_rc  # Make sure you have the resources loaded

# def on_menu_button_click():
#     """ This function will be called when the menu button is clicked. """
#     print("Menu button clicked!")  # Example action
#     # You can replace this with any action you need, such as opening a menu.

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()  # Create an instance of your UI class
        self.ui.setupUi(self)  # Set up the UI
        
        # Connect the menu button's clicked signal to your custom slot
        # self.ui.menuBtn.clicked.connect(on_menu_button_click)

        # Apply JSON stylesheet if needed (using your custom function)

        # Enable window dragging
        self.dragging = False
        self.offset = QPoint()

        # Apply frameless window hint
        self.setWindowFlags(Qt.FramelessWindowHint)

        # Allow transparent background if needed
        self.setAttribute(Qt.WA_TranslucentBackground)

        # Make a specific widget draggable (Header container from JSON)
        self.ui.headerContainer.mousePressEvent = self.mousePressEvent
        self.ui.headerContainer.mouseMoveEvent = self.mouseMoveEvent

        json_file_path = os.path.join(os.path.dirname(__file__), "json-styles", "style.json")
        loadJsonStyle(self, self.ui, jsonFiles={json_file_path})

        # Show the main window
        self.show()

        # Update app settings if necessary
        QAppSettings.updateAppSettings(self)

        self.app_functions = GuiFunctions(self)
        self.backend = Backend()

    def sassCompilationProgress(self, n):
        self.ui.activityProgress.setValue(n)

    def mousePressEvent(self, event):
        """ Capture the initial position when the mouse is pressed """
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.offset = event.globalPos() - self.pos()
            event.accept()

    def mouseMoveEvent(self, event):
        """ Move the window when dragging """
        if self.dragging and event.buttons() == Qt.LeftButton:
            self.move(event.globalPos() - self.offset)
            event.accept()

    def mouseReleaseEvent(self, event):
        """ Stop dragging when mouse is released """
        self.dragging = False
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())
