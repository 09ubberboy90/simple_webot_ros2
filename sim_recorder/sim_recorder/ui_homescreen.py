# -*- coding: utf-8 -*-

################################################################################
# Form generated from reading UI file 'homescreen.ui'
##
# Created by: Qt User Interface Compiler version 5.15.2
##
# WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

try:
    from resource_monitor import CpuFreqGraph
    from combo_box import CheckableComboBox
except:
    from .resource_monitor import CpuFreqGraph
    from .combo_box import CheckableComboBox


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1072, 710)
        MainWindow.setMinimumSize(QSize(1072, 710))
        font = QFont()
        font.setPointSize(11)
        MainWindow.setFont(font)
        icon = QIcon()
        icon.addFile(u"fpd_explorer/frontend/res/icon.png",
                     QSize(), QIcon.Normal, QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout = QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.button = QPushButton(self.centralwidget)
        self.button.setObjectName(u"button")

        self.gridLayout.addWidget(self.button, 0, 1, 1, 1)

        self.button2 = QPushButton(self.centralwidget)
        self.button2.setObjectName(u"button2")

        self.gridLayout.addWidget(self.button2, 0, 2, 1, 1)

        self.graph = CpuFreqGraph(self.centralwidget)
        self.graph.setObjectName(u"graph")
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.graph.sizePolicy().hasHeightForWidth())
        self.graph.setSizePolicy(sizePolicy)

        self.gridLayout.addWidget(self.graph, 2, 0, 1, 4)

        self.process = CheckableComboBox(self.centralwidget)
        self.process.setObjectName(u"process")
        sizePolicy1 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(
            self.process.sizePolicy().hasHeightForWidth())
        self.process.setSizePolicy(sizePolicy1)

        self.gridLayout.addWidget(self.process, 0, 0, 1, 1)

        self.dump = QPushButton(self.centralwidget)
        self.dump.setObjectName(u"dump")

        self.gridLayout.addWidget(self.dump, 0, 3, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.button.clicked.connect(MainWindow.change_proc)
        self.button2.clicked.connect(MainWindow.update_proc_list)
        self.button2.pressed.connect(self.process.clear)
        self.dump.clicked.connect(MainWindow.dump_selected)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(
            QCoreApplication.translate("MainWindow", u"Grapher", None))
        self.button.setText(QCoreApplication.translate(
            "MainWindow", u"Confirm", None))
        self.button2.setText(QCoreApplication.translate(
            "MainWindow", u"Update Proccess", None))
# if QT_CONFIG(tooltip)
        self.process.setToolTip(QCoreApplication.translate(
            "MainWindow", u"Processes", None))
#endif // QT_CONFIG(tooltip)
# if QT_CONFIG(statustip)
        self.process.setStatusTip(QCoreApplication.translate(
            "MainWindow", u"Processes", None))
#endif // QT_CONFIG(statustip)
# if QT_CONFIG(whatsthis)
        self.process.setWhatsThis(QCoreApplication.translate(
            "MainWindow", u"Processes", None))
#endif // QT_CONFIG(whatsthis)
        self.process.setCurrentText("")
        self.dump.setText(QCoreApplication.translate(
            "MainWindow", u"Dump Selected", None))
    # retranslateUi
