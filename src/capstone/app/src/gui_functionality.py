from Custom_Widgets import *
from Custom_Widgets.QAppSettings import QAppSettings
from Custom_Widgets.QCustomTipOverlay import QCustomTipOverlay
from Custom_Widgets.QCustomLoadingIndicators import QCustom3CirclesLoader
from PySide6.QtCore import QSettings, QTimer
from PySide6.QtGui import QColor, QFont, QFontDatabase
from PySide6.QtWidgets import QGraphicsDropShadowEffect

class GuiFunctions():
    def __init__(self, MainWindow):
        self.main = MainWindow # Store the mainwindow instance
        self.ui = MainWindow.ui # Store the ui instance

        self.loadProductSansFont()
        # init app theme
        self.initializeAppTheme()

        self.connectMenuButtons()
    
    # connect menu buttons
    def connectMenuButtons (self):
        """Connect buttons to expand/collapse menu widgets."""
        # Expand right menu widget
        self.ui.settingsBtn.clicked.connect(lambda: self.ui.rightMenuContainer.expandMenu())
        self.ui.infoBtn.clicked.connect(lambda: self.ui.rightMenuContainer.expandMenu())
        self.ui.helpBtn.clicked.connect(lambda: self.ui.rightMenuContainer.expandMenu())
        self.ui.notificationBtn.clicked.connect(lambda: self.ui.rightMenuContainer.expandMenu())
        self.ui.moreMenuBtn.clicked.connect(lambda: self.ui.rightMenuContainer.expandMenu())
        self.ui.profileMenuBtn.clicked. connect (lambda: self.ui.rightMenuContainer.expandMenu())
        # Close right menu widget
        self.ui.closeRightMenuBtn.clicked.connect(lambda: self.ui.rightMenuContainer.collapseMenu())

    
    # initialize app them
    def initializeAppTheme (self):
        """Initialize the application theme from settings"""
        settings = QSettings ()
        current_theme = settings.value("THEME")
        # print("Current theme is: ", current_theme)

        # Add theme to the theme list
        self.populateThemeList (current_theme)
        self.ui.themeList.currentTextChanged.connect(self.changeAppTheme)
    
    def changeAppTheme (self):
        settings = QSettings()
        selected_theme = self.ui.themeList.currentData()
        current_theme = settings.value("THEME")
        if current_theme != selected_theme:
            # apply new theme
            settings.setValue("THEME", selected_theme)
            QAppSettings.updateAppSettings(self.main, reloadJson=True)
    
    def populateThemeList(self, current_theme):
        """Populate the list from available app themes"""
        theme_count = -1
        for theme in self.ui.themes:
            self.ui.themeList.addItem(theme.name, theme.name)
            # check default theme/current theme
            if theme.defaultTheme or theme.name == current_theme:
                self.ui.themeList.setCurrentIndex(theme_count)

    def loadProductSansFont(self):
        """Load and apply product sans font"""
        font_id = QFontDatabase.addApplicationFont("./fonts/google-sans-cufonfonts/ProductSans-Regular.ttf")
        # if font loaded
        if font_id == -1:
            print("Failed to load Product Sans font")
            return
        # Apply font
        font_family = QFontDatabase. applicationFontFamilies (font_id)
        if font_family:
            product_sans = QFont (font_family[0])
        else:
            product_sans = QFont("Sans Serif")
        # apply to main window
        self.main.setFont(product_sans)               

    