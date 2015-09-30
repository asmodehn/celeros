# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'turtlecmd.ui'
#
# Created: Wed Sep 30 14:12:29 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(800, 600)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.widget_2 = QtGui.QWidget(self.centralwidget)
        self.widget_2.setGeometry(QtCore.QRect(20, 390, 751, 141))
        self.widget_2.setObjectName(_fromUtf8("widget_2"))
        self.ANY_lbl = QtGui.QLabel(self.widget_2)
        self.ANY_lbl.setGeometry(QtCore.QRect(10, 10, 58, 17))
        self.ANY_lbl.setObjectName(_fromUtf8("ANY_lbl"))
        self.TODO_lbl = QtGui.QLabel(self.widget_2)
        self.TODO_lbl.setGeometry(QtCore.QRect(210, 50, 121, 17))
        self.TODO_lbl.setObjectName(_fromUtf8("TODO_lbl"))
        self.tabWidget = QtGui.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(20, 10, 751, 361))
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 27))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.ANY_lbl.setText(_translate("MainWindow", "ANY", None))
        self.TODO_lbl.setText(_translate("MainWindow", "TODO : Actions", None))

