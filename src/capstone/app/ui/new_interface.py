# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'new_interface.ui'
##
## Created by: Qt User Interface Compiler version 6.8.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QHBoxLayout,
    QLabel, QMainWindow, QProgressBar, QPushButton,
    QScrollArea, QSizePolicy, QSpacerItem, QVBoxLayout,
    QWidget)

from Custom_Widgets.QCustomQStackedWidget import QCustomQStackedWidget
from Custom_Widgets.QCustomSlideMenu import QCustomSlideMenu
import _icons_rc

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1045, 626)
        font = QFont()
        font.setPointSize(10)
        MainWindow.setFont(font)
        MainWindow.setStyleSheet(u"")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.centralwidget.setMinimumSize(QSize(1045, 626))
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(5, 5, 5, 5)
        self.leftMenuContainer = QCustomSlideMenu(self.centralwidget)
        self.leftMenuContainer.setObjectName(u"leftMenuContainer")
        self.leftMenuContainer.setMinimumSize(QSize(200, 0))
        self.leftMenuContainer.setMaximumSize(QSize(200, 16777215))
        self.verticalLayout = QVBoxLayout(self.leftMenuContainer)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.leftMenuSubContainer = QWidget(self.leftMenuContainer)
        self.leftMenuSubContainer.setObjectName(u"leftMenuSubContainer")
        self.verticalLayout_2 = QVBoxLayout(self.leftMenuSubContainer)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 5, 0)
        self.frame_2 = QFrame(self.leftMenuSubContainer)
        self.frame_2.setObjectName(u"frame_2")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_2.sizePolicy().hasHeightForWidth())
        self.frame_2.setSizePolicy(sizePolicy)
        self.frame_2.setFrameShape(QFrame.NoFrame)
        self.verticalLayout_3 = QVBoxLayout(self.frame_2)
        self.verticalLayout_3.setSpacing(5)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 15, 0, 10)
        self.menuBtn = QPushButton(self.frame_2)
        self.menuBtn.setObjectName(u"menuBtn")
        icon = QIcon()
        icon.addFile(u":/feather/Qss/Icons/feather/menu.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.menuBtn.setIcon(icon)
        self.menuBtn.setIconSize(QSize(16, 16))

        self.verticalLayout_3.addWidget(self.menuBtn)

        self.homeBtn = QPushButton(self.frame_2)
        self.homeBtn.setObjectName(u"homeBtn")
        icon1 = QIcon()
        icon1.addFile(u":/feather/Qss/Icons/feather/home.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.homeBtn.setIcon(icon1)
        self.homeBtn.setIconSize(QSize(16, 16))

        self.verticalLayout_3.addWidget(self.homeBtn)

        self.dataBtn = QPushButton(self.frame_2)
        self.dataBtn.setObjectName(u"dataBtn")
        icon2 = QIcon()
        icon2.addFile(u":/material_design/Qss/Icons/material_design/analytics.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.dataBtn.setIcon(icon2)
        self.dataBtn.setIconSize(QSize(16, 16))

        self.verticalLayout_3.addWidget(self.dataBtn)

        self.reportBtn = QPushButton(self.frame_2)
        self.reportBtn.setObjectName(u"reportBtn")
        icon3 = QIcon()
        icon3.addFile(u":/feather/Qss/Icons/feather/printer.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.reportBtn.setIcon(icon3)
        self.reportBtn.setIconSize(QSize(16, 16))

        self.verticalLayout_3.addWidget(self.reportBtn)


        self.verticalLayout_2.addWidget(self.frame_2)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.verticalLayout_2.addItem(self.verticalSpacer)

        self.frame_3 = QFrame(self.leftMenuSubContainer)
        self.frame_3.setObjectName(u"frame_3")
        self.frame_3.setFrameShape(QFrame.NoFrame)
        self.verticalLayout_4 = QVBoxLayout(self.frame_3)
        self.verticalLayout_4.setSpacing(5)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 10, 0, 10)
        self.settingsBtn = QPushButton(self.frame_3)
        self.settingsBtn.setObjectName(u"settingsBtn")
        icon4 = QIcon()
        icon4.addFile(u":/feather/Qss/Icons/feather/settings.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.settingsBtn.setIcon(icon4)
        self.settingsBtn.setIconSize(QSize(16, 16))

        self.verticalLayout_4.addWidget(self.settingsBtn)

        self.infoBtn = QPushButton(self.frame_3)
        self.infoBtn.setObjectName(u"infoBtn")
        icon5 = QIcon()
        icon5.addFile(u":/feather/Qss/Icons/feather/info.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.infoBtn.setIcon(icon5)
        self.infoBtn.setIconSize(QSize(16, 16))

        self.verticalLayout_4.addWidget(self.infoBtn)

        self.helpBtn = QPushButton(self.frame_3)
        self.helpBtn.setObjectName(u"helpBtn")
        icon6 = QIcon()
        icon6.addFile(u":/feather/Qss/Icons/feather/help-circle.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.helpBtn.setIcon(icon6)
        self.helpBtn.setIconSize(QSize(16, 16))

        self.verticalLayout_4.addWidget(self.helpBtn)


        self.verticalLayout_2.addWidget(self.frame_3)


        self.verticalLayout.addWidget(self.leftMenuSubContainer)


        self.horizontalLayout.addWidget(self.leftMenuContainer)

        self.mainBodyContainer = QWidget(self.centralwidget)
        self.mainBodyContainer.setObjectName(u"mainBodyContainer")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.mainBodyContainer.sizePolicy().hasHeightForWidth())
        self.mainBodyContainer.setSizePolicy(sizePolicy1)
        self.mainBodyContainer.setStyleSheet(u"")
        self.verticalLayout_10 = QVBoxLayout(self.mainBodyContainer)
        self.verticalLayout_10.setSpacing(0)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.verticalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.headerContainer = QWidget(self.mainBodyContainer)
        self.headerContainer.setObjectName(u"headerContainer")
        self.horizontalLayout_5 = QHBoxLayout(self.headerContainer)
        self.horizontalLayout_5.setSpacing(5)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(5, 5, 5, 5)
        self.frame_6 = QFrame(self.headerContainer)
        self.frame_6.setObjectName(u"frame_6")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy2.setHorizontalStretch(1)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.frame_6.sizePolicy().hasHeightForWidth())
        self.frame_6.setSizePolicy(sizePolicy2)
        self.frame_6.setFrameShape(QFrame.NoFrame)
        self.horizontalLayout_6 = QHBoxLayout(self.frame_6)
        self.horizontalLayout_6.setSpacing(5)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.horizontalLayout_6.setContentsMargins(5, 5, 5, 5)
        self.notificationBtn = QPushButton(self.frame_6)
        self.notificationBtn.setObjectName(u"notificationBtn")
        icon7 = QIcon()
        icon7.addFile(u":/feather/Qss/Icons/feather/bell.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.notificationBtn.setIcon(icon7)
        self.notificationBtn.setIconSize(QSize(16, 16))

        self.horizontalLayout_6.addWidget(self.notificationBtn)

        self.moreMenuBtn = QPushButton(self.frame_6)
        self.moreMenuBtn.setObjectName(u"moreMenuBtn")
        icon8 = QIcon()
        icon8.addFile(u":/feather/Qss/Icons/feather/more-horizontal.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.moreMenuBtn.setIcon(icon8)
        self.moreMenuBtn.setIconSize(QSize(16, 16))

        self.horizontalLayout_6.addWidget(self.moreMenuBtn)

        self.profileMenuBtn = QPushButton(self.frame_6)
        self.profileMenuBtn.setObjectName(u"profileMenuBtn")
        icon9 = QIcon()
        icon9.addFile(u":/feather/Qss/Icons/feather/user.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.profileMenuBtn.setIcon(icon9)
        self.profileMenuBtn.setIconSize(QSize(16, 16))

        self.horizontalLayout_6.addWidget(self.profileMenuBtn)


        self.horizontalLayout_5.addWidget(self.frame_6)

        self.frame_5 = QFrame(self.headerContainer)
        self.frame_5.setObjectName(u"frame_5")
        sizePolicy2.setHeightForWidth(self.frame_5.sizePolicy().hasHeightForWidth())
        self.frame_5.setSizePolicy(sizePolicy2)
        self.frame_5.setFrameShape(QFrame.NoFrame)
        self.horizontalLayout_7 = QHBoxLayout(self.frame_5)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)

        self.horizontalLayout_5.addWidget(self.frame_5)

        self.frame_7 = QFrame(self.headerContainer)
        self.frame_7.setObjectName(u"frame_7")
        sizePolicy2.setHeightForWidth(self.frame_7.sizePolicy().hasHeightForWidth())
        self.frame_7.setSizePolicy(sizePolicy2)
        self.frame_7.setFrameShape(QFrame.NoFrame)
        self.horizontalLayout_4 = QHBoxLayout(self.frame_7)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.minimizeBtn = QPushButton(self.frame_7)
        self.minimizeBtn.setObjectName(u"minimizeBtn")
        icon10 = QIcon()
        icon10.addFile(u":/feather/Qss/Icons/feather/minus.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.minimizeBtn.setIcon(icon10)

        self.horizontalLayout_4.addWidget(self.minimizeBtn)

        self.restoreBtn = QPushButton(self.frame_7)
        self.restoreBtn.setObjectName(u"restoreBtn")
        icon11 = QIcon()
        icon11.addFile(u":/feather/Qss/Icons/feather/square.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.restoreBtn.setIcon(icon11)

        self.horizontalLayout_4.addWidget(self.restoreBtn)

        self.closeBtn = QPushButton(self.frame_7)
        self.closeBtn.setObjectName(u"closeBtn")
        icon12 = QIcon()
        icon12.addFile(u":/feather/Qss/Icons/feather/x.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.closeBtn.setIcon(icon12)

        self.horizontalLayout_4.addWidget(self.closeBtn)


        self.horizontalLayout_5.addWidget(self.frame_7)


        self.verticalLayout_10.addWidget(self.headerContainer)

        self.mainBodyContent = QWidget(self.mainBodyContainer)
        self.mainBodyContent.setObjectName(u"mainBodyContent")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.mainBodyContent.sizePolicy().hasHeightForWidth())
        self.mainBodyContent.setSizePolicy(sizePolicy3)
        self.mainBodyContent.setMinimumSize(QSize(555, 315))
        self.horizontalLayout_8 = QHBoxLayout(self.mainBodyContent)
        self.horizontalLayout_8.setSpacing(0)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.scrollArea = QScrollArea(self.mainBodyContent)
        self.scrollArea.setObjectName(u"scrollArea")
        self.scrollArea.setWidgetResizable(True)
        self.scrollAreaWidgetContents_4 = QWidget()
        self.scrollAreaWidgetContents_4.setObjectName(u"scrollAreaWidgetContents_4")
        self.scrollAreaWidgetContents_4.setGeometry(QRect(0, 0, 416, 543))
        self.horizontalLayout_20 = QHBoxLayout(self.scrollAreaWidgetContents_4)
        self.horizontalLayout_20.setSpacing(0)
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.horizontalLayout_20.setContentsMargins(5, 0, 0, 0)
        self.mainPages = QCustomQStackedWidget(self.scrollAreaWidgetContents_4)
        self.mainPages.setObjectName(u"mainPages")
        self.home_page = QWidget()
        self.home_page.setObjectName(u"home_page")
        self.horizontalLayout_3 = QHBoxLayout(self.home_page)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.video_label = QLabel(self.home_page)
        self.video_label.setObjectName(u"video_label")

        self.horizontalLayout_3.addWidget(self.video_label)

        self.picture_label = QLabel(self.home_page)
        self.picture_label.setObjectName(u"picture_label")

        self.horizontalLayout_3.addWidget(self.picture_label)

        self.mainPages.addWidget(self.home_page)
        self.gantt = QWidget()
        self.gantt.setObjectName(u"gantt")
        self.verticalLayout_28 = QVBoxLayout(self.gantt)
        self.verticalLayout_28.setSpacing(5)
        self.verticalLayout_28.setObjectName(u"verticalLayout_28")
        self.verticalLayout_28.setContentsMargins(5, 5, 5, 5)
        self.label_9 = QLabel(self.gantt)
        self.label_9.setObjectName(u"label_9")

        self.verticalLayout_28.addWidget(self.label_9)

        self.mainPages.addWidget(self.gantt)
        self.data_page = QWidget()
        self.data_page.setObjectName(u"data_page")
        self.verticalLayout_29 = QVBoxLayout(self.data_page)
        self.verticalLayout_29.setSpacing(5)
        self.verticalLayout_29.setObjectName(u"verticalLayout_29")
        self.verticalLayout_29.setContentsMargins(5, 5, 5, 5)
        self.label_13 = QLabel(self.data_page)
        self.label_13.setObjectName(u"label_13")

        self.verticalLayout_29.addWidget(self.label_13)

        self.mainPages.addWidget(self.data_page)
        self.reports_page = QWidget()
        self.reports_page.setObjectName(u"reports_page")
        self.verticalLayout_30 = QVBoxLayout(self.reports_page)
        self.verticalLayout_30.setSpacing(5)
        self.verticalLayout_30.setObjectName(u"verticalLayout_30")
        self.verticalLayout_30.setContentsMargins(5, 5, 5, 5)
        self.label_16 = QLabel(self.reports_page)
        self.label_16.setObjectName(u"label_16")

        self.verticalLayout_30.addWidget(self.label_16)

        self.mainPages.addWidget(self.reports_page)

        self.horizontalLayout_20.addWidget(self.mainPages)

        self.mainContentsContainer = QWidget(self.scrollAreaWidgetContents_4)
        self.mainContentsContainer.setObjectName(u"mainContentsContainer")
        self.verticalLayout_24 = QVBoxLayout(self.mainContentsContainer)
        self.verticalLayout_24.setSpacing(0)
        self.verticalLayout_24.setObjectName(u"verticalLayout_24")
        self.verticalLayout_24.setContentsMargins(5, 0, 0, 0)

        self.horizontalLayout_20.addWidget(self.mainContentsContainer)

        self.scrollArea.setWidget(self.scrollAreaWidgetContents_4)

        self.horizontalLayout_8.addWidget(self.scrollArea)

        self.rightMenuContainer = QCustomSlideMenu(self.mainBodyContent)
        self.rightMenuContainer.setObjectName(u"rightMenuContainer")
        self.rightMenuContainer.setMinimumSize(QSize(0, 0))
        self.rightMenuContainer.setMaximumSize(QSize(1000, 1000))
        self.verticalLayout_6 = QVBoxLayout(self.rightMenuContainer)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.frame_17 = QFrame(self.rightMenuContainer)
        self.frame_17.setObjectName(u"frame_17")
        self.frame_17.setFrameShape(QFrame.NoFrame)
        self.horizontalLayout_19 = QHBoxLayout(self.frame_17)
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.label_10 = QLabel(self.frame_17)
        self.label_10.setObjectName(u"label_10")
        sizePolicy1.setHeightForWidth(self.label_10.sizePolicy().hasHeightForWidth())
        self.label_10.setSizePolicy(sizePolicy1)

        self.horizontalLayout_19.addWidget(self.label_10)

        self.closeRightMenuBtn = QPushButton(self.frame_17)
        self.closeRightMenuBtn.setObjectName(u"closeRightMenuBtn")
        icon13 = QIcon()
        icon13.addFile(u":/feather/Qss/Icons/feather/x-circle.png", QSize(), QIcon.Mode.Normal, QIcon.State.Off)
        self.closeRightMenuBtn.setIcon(icon13)
        self.closeRightMenuBtn.setIconSize(QSize(16, 16))

        self.horizontalLayout_19.addWidget(self.closeRightMenuBtn)


        self.verticalLayout_6.addWidget(self.frame_17)

        self.verticalSpacer_5 = QSpacerItem(20, 40, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.verticalLayout_6.addItem(self.verticalSpacer_5)

        self.rightMenuPages = QCustomQStackedWidget(self.rightMenuContainer)
        self.rightMenuPages.setObjectName(u"rightMenuPages")
        self.profile_page = QWidget()
        self.profile_page.setObjectName(u"profile_page")
        self.verticalLayout_33 = QVBoxLayout(self.profile_page)
        self.verticalLayout_33.setObjectName(u"verticalLayout_33")
        self.label_17 = QLabel(self.profile_page)
        self.label_17.setObjectName(u"label_17")

        self.verticalLayout_33.addWidget(self.label_17)

        self.rightMenuPages.addWidget(self.profile_page)
        self.notification_page = QWidget()
        self.notification_page.setObjectName(u"notification_page")
        self.verticalLayout_11 = QVBoxLayout(self.notification_page)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.label_5 = QLabel(self.notification_page)
        self.label_5.setObjectName(u"label_5")

        self.verticalLayout_11.addWidget(self.label_5)

        self.rightMenuPages.addWidget(self.notification_page)
        self.more_page = QWidget()
        self.more_page.setObjectName(u"more_page")
        self.label_18 = QLabel(self.more_page)
        self.label_18.setObjectName(u"label_18")
        self.label_18.setGeometry(QRect(9, 9, 41, 16))
        self.rightMenuPages.addWidget(self.more_page)
        self.page = QWidget()
        self.page.setObjectName(u"page")
        self.rightMenuPages.addWidget(self.page)
        self.settings_page = QWidget()
        self.settings_page.setObjectName(u"settings_page")
        self.verticalLayout_7 = QVBoxLayout(self.settings_page)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.label_2 = QLabel(self.settings_page)
        self.label_2.setObjectName(u"label_2")

        self.verticalLayout_7.addWidget(self.label_2)

        self.frame_13 = QFrame(self.settings_page)
        self.frame_13.setObjectName(u"frame_13")
        self.frame_13.setFrameShape(QFrame.NoFrame)
        self.horizontalLayout_14 = QHBoxLayout(self.frame_13)
        self.horizontalLayout_14.setSpacing(0)
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.horizontalLayout_14.setContentsMargins(0, 0, 0, 0)
        self.label_6 = QLabel(self.frame_13)
        self.label_6.setObjectName(u"label_6")

        self.horizontalLayout_14.addWidget(self.label_6)

        self.themeList = QComboBox(self.frame_13)
        self.themeList.setObjectName(u"themeList")

        self.horizontalLayout_14.addWidget(self.themeList)


        self.verticalLayout_7.addWidget(self.frame_13)

        self.rightMenuPages.addWidget(self.settings_page)
        self.help_page = QWidget()
        self.help_page.setObjectName(u"help_page")
        self.verticalLayout_9 = QVBoxLayout(self.help_page)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.label_4 = QLabel(self.help_page)
        self.label_4.setObjectName(u"label_4")

        self.verticalLayout_9.addWidget(self.label_4)

        self.rightMenuPages.addWidget(self.help_page)
        self.info_page = QWidget()
        self.info_page.setObjectName(u"info_page")
        self.verticalLayout_8 = QVBoxLayout(self.info_page)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.label_3 = QLabel(self.info_page)
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout_8.addWidget(self.label_3)

        self.rightMenuPages.addWidget(self.info_page)

        self.verticalLayout_6.addWidget(self.rightMenuPages)

        self.verticalSpacer_4 = QSpacerItem(20, 40, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.verticalLayout_6.addItem(self.verticalSpacer_4)


        self.horizontalLayout_8.addWidget(self.rightMenuContainer)


        self.verticalLayout_10.addWidget(self.mainBodyContent)

        self.footerContainter = QWidget(self.mainBodyContainer)
        self.footerContainter.setObjectName(u"footerContainter")
        self.horizontalLayout_11 = QHBoxLayout(self.footerContainter)
        self.horizontalLayout_11.setSpacing(0)
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.horizontalLayout_11.setContentsMargins(0, 7, 0, 0)
        self.frame_10 = QFrame(self.footerContainter)
        self.frame_10.setObjectName(u"frame_10")
        self.frame_10.setFrameShape(QFrame.NoFrame)
        self.horizontalLayout_12 = QHBoxLayout(self.frame_10)
        self.horizontalLayout_12.setSpacing(0)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.horizontalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.label_15 = QLabel(self.frame_10)
        self.label_15.setObjectName(u"label_15")

        self.horizontalLayout_12.addWidget(self.label_15)


        self.horizontalLayout_11.addWidget(self.frame_10)

        self.frame_14 = QFrame(self.footerContainter)
        self.frame_14.setObjectName(u"frame_14")
        self.frame_14.setFrameShape(QFrame.NoFrame)
        self.horizontalLayout_15 = QHBoxLayout(self.frame_14)
        self.horizontalLayout_15.setSpacing(0)
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.horizontalLayout_15.setContentsMargins(0, 0, 0, 0)
        self.activityLabel = QLabel(self.frame_14)
        self.activityLabel.setObjectName(u"activityLabel")
        font1 = QFont()
        font1.setPointSize(9)
        self.activityLabel.setFont(font1)

        self.horizontalLayout_15.addWidget(self.activityLabel)

        self.activityProgress = QProgressBar(self.frame_14)
        self.activityProgress.setObjectName(u"activityProgress")
        self.activityProgress.setMinimumSize(QSize(0, 10))
        self.activityProgress.setMaximumSize(QSize(16777215, 10))
        self.activityProgress.setValue(100)
        self.activityProgress.setInvertedAppearance(False)

        self.horizontalLayout_15.addWidget(self.activityProgress)


        self.horizontalLayout_11.addWidget(self.frame_14)

        self.sizeGrip = QFrame(self.footerContainter)
        self.sizeGrip.setObjectName(u"sizeGrip")
        self.sizeGrip.setMinimumSize(QSize(20, 20))
        self.sizeGrip.setMaximumSize(QSize(20, 20))
        self.sizeGrip.setFrameShape(QFrame.NoFrame)

        self.horizontalLayout_11.addWidget(self.sizeGrip)


        self.verticalLayout_10.addWidget(self.footerContainter)


        self.horizontalLayout.addWidget(self.mainBodyContainer)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.mainPages.setCurrentIndex(0)
        self.rightMenuPages.setCurrentIndex(4)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
#if QT_CONFIG(tooltip)
        self.menuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Menu", None))
#endif // QT_CONFIG(tooltip)
        self.menuBtn.setText("")
#if QT_CONFIG(tooltip)
        self.homeBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Home", None))
#endif // QT_CONFIG(tooltip)
        self.homeBtn.setText(QCoreApplication.translate("MainWindow", u"Home", None))
#if QT_CONFIG(tooltip)
        self.dataBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Data Analysis", None))
#endif // QT_CONFIG(tooltip)
        self.dataBtn.setText(QCoreApplication.translate("MainWindow", u"Analysis", None))
#if QT_CONFIG(tooltip)
        self.reportBtn.setToolTip(QCoreApplication.translate("MainWindow", u"View Reports", None))
#endif // QT_CONFIG(tooltip)
        self.reportBtn.setText(QCoreApplication.translate("MainWindow", u"Reports", None))
#if QT_CONFIG(tooltip)
        self.settingsBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Go to settings", None))
#endif // QT_CONFIG(tooltip)
        self.settingsBtn.setText(QCoreApplication.translate("MainWindow", u"Settings", None))
#if QT_CONFIG(tooltip)
        self.infoBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Information about the app", None))
#endif // QT_CONFIG(tooltip)
        self.infoBtn.setText(QCoreApplication.translate("MainWindow", u"Info", None))
#if QT_CONFIG(tooltip)
        self.helpBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Get more help", None))
#endif // QT_CONFIG(tooltip)
        self.helpBtn.setText(QCoreApplication.translate("MainWindow", u"Help", None))
#if QT_CONFIG(tooltip)
        self.notificationBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Notifications", None))
#endif // QT_CONFIG(tooltip)
        self.notificationBtn.setText("")
#if QT_CONFIG(tooltip)
        self.moreMenuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"More", None))
#endif // QT_CONFIG(tooltip)
        self.moreMenuBtn.setText("")
#if QT_CONFIG(tooltip)
        self.profileMenuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Profile", None))
#endif // QT_CONFIG(tooltip)
        self.profileMenuBtn.setText("")
#if QT_CONFIG(tooltip)
        self.minimizeBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Minimize Window", None))
#endif // QT_CONFIG(tooltip)
        self.minimizeBtn.setText("")
#if QT_CONFIG(tooltip)
        self.restoreBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Restore Window", None))
#endif // QT_CONFIG(tooltip)
        self.restoreBtn.setText("")
#if QT_CONFIG(tooltip)
        self.closeBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Close Window", None))
#endif // QT_CONFIG(tooltip)
        self.closeBtn.setText("")
        self.video_label.setText(QCoreApplication.translate("MainWindow", u"Live", None))
        self.picture_label.setText(QCoreApplication.translate("MainWindow", u"Captured", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Charts", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"Data Analysis", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"Reports", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Right Menu", None))
#if QT_CONFIG(tooltip)
        self.closeRightMenuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Close Menu", None))
#endif // QT_CONFIG(tooltip)
        self.closeRightMenuBtn.setText("")
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"Profile", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Notifications", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"More...", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Settings", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Theme", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Help", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Information", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"Copyright Inspector Gadget", None))
        self.activityLabel.setText(QCoreApplication.translate("MainWindow", u"Connection Progress", None))
    # retranslateUi

