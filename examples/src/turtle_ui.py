# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'turtle.ui'
#
# Created: Wed Sep 30 14:12:15 2015
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

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(434, 300)
        self.pose_y_lcd = QtGui.QLCDNumber(Form)
        self.pose_y_lcd.setGeometry(QtCore.QRect(20, 70, 64, 23))
        self.pose_y_lcd.setObjectName(_fromUtf8("pose_y_lcd"))
        self.pose_angular_lcd = QtGui.QLCDNumber(Form)
        self.pose_angular_lcd.setGeometry(QtCore.QRect(20, 220, 64, 23))
        self.pose_angular_lcd.setObjectName(_fromUtf8("pose_angular_lcd"))
        self.pose_linear_lcd = QtGui.QLCDNumber(Form)
        self.pose_linear_lcd.setGeometry(QtCore.QRect(20, 180, 64, 23))
        self.pose_linear_lcd.setObjectName(_fromUtf8("pose_linear_lcd"))
        self.pose_x_lbl = QtGui.QLabel(Form)
        self.pose_x_lbl.setGeometry(QtCore.QRect(90, 30, 58, 17))
        self.pose_x_lbl.setObjectName(_fromUtf8("pose_x_lbl"))
        self.pose_angular_lbl = QtGui.QLabel(Form)
        self.pose_angular_lbl.setGeometry(QtCore.QRect(90, 220, 58, 17))
        self.pose_angular_lbl.setObjectName(_fromUtf8("pose_angular_lbl"))
        self.cmd_angular_y_sbx = QtGui.QSpinBox(Form)
        self.cmd_angular_y_sbx.setGeometry(QtCore.QRect(320, 100, 52, 27))
        self.cmd_angular_y_sbx.setObjectName(_fromUtf8("cmd_angular_y_sbx"))
        self.cmd_linear_y_sbx = QtGui.QSpinBox(Form)
        self.cmd_linear_y_sbx.setGeometry(QtCore.QRect(180, 100, 52, 27))
        self.cmd_linear_y_sbx.setObjectName(_fromUtf8("cmd_linear_y_sbx"))
        self.pose_tetha_lbl = QtGui.QLabel(Form)
        self.pose_tetha_lbl.setGeometry(QtCore.QRect(90, 110, 58, 17))
        self.pose_tetha_lbl.setObjectName(_fromUtf8("pose_tetha_lbl"))
        self.pose_theta_lcd = QtGui.QLCDNumber(Form)
        self.pose_theta_lcd.setGeometry(QtCore.QRect(20, 110, 64, 23))
        self.pose_theta_lcd.setObjectName(_fromUtf8("pose_theta_lcd"))
        self.cmd_linear_y_lbl = QtGui.QLabel(Form)
        self.cmd_linear_y_lbl.setGeometry(QtCore.QRect(240, 100, 58, 17))
        self.cmd_linear_y_lbl.setObjectName(_fromUtf8("cmd_linear_y_lbl"))
        self.pose_linear_lbl = QtGui.QLabel(Form)
        self.pose_linear_lbl.setGeometry(QtCore.QRect(90, 180, 58, 17))
        self.pose_linear_lbl.setObjectName(_fromUtf8("pose_linear_lbl"))
        self.cmd_linear_z_sbx = QtGui.QSpinBox(Form)
        self.cmd_linear_z_sbx.setGeometry(QtCore.QRect(180, 140, 52, 27))
        self.cmd_linear_z_sbx.setObjectName(_fromUtf8("cmd_linear_z_sbx"))
        self.cmd_angular_z_lbl = QtGui.QLabel(Form)
        self.cmd_angular_z_lbl.setGeometry(QtCore.QRect(380, 140, 58, 17))
        self.cmd_angular_z_lbl.setObjectName(_fromUtf8("cmd_angular_z_lbl"))
        self.cmd_linear_lbl = QtGui.QLabel(Form)
        self.cmd_linear_lbl.setGeometry(QtCore.QRect(180, 20, 58, 17))
        self.cmd_linear_lbl.setObjectName(_fromUtf8("cmd_linear_lbl"))
        self.pose_x_lcd = QtGui.QLCDNumber(Form)
        self.pose_x_lcd.setGeometry(QtCore.QRect(20, 30, 64, 23))
        self.pose_x_lcd.setObjectName(_fromUtf8("pose_x_lcd"))
        self.pose_y_lbl = QtGui.QLabel(Form)
        self.pose_y_lbl.setGeometry(QtCore.QRect(90, 70, 58, 17))
        self.pose_y_lbl.setObjectName(_fromUtf8("pose_y_lbl"))
        self.cmd_angular_lbl = QtGui.QLabel(Form)
        self.cmd_angular_lbl.setGeometry(QtCore.QRect(380, 100, 58, 17))
        self.cmd_angular_lbl.setObjectName(_fromUtf8("cmd_angular_lbl"))
        self.cmd_angular_z_sbx = QtGui.QSpinBox(Form)
        self.cmd_angular_z_sbx.setGeometry(QtCore.QRect(320, 140, 52, 27))
        self.cmd_angular_z_sbx.setObjectName(_fromUtf8("cmd_angular_z_sbx"))
        self.cmd_angular_x_sbx = QtGui.QSpinBox(Form)
        self.cmd_angular_x_sbx.setGeometry(QtCore.QRect(320, 60, 52, 27))
        self.cmd_angular_x_sbx.setObjectName(_fromUtf8("cmd_angular_x_sbx"))
        self.cmd_Angular_lbl = QtGui.QLabel(Form)
        self.cmd_Angular_lbl.setGeometry(QtCore.QRect(320, 20, 58, 17))
        self.cmd_Angular_lbl.setObjectName(_fromUtf8("cmd_Angular_lbl"))
        self.cmd_angular_x_lbl = QtGui.QLabel(Form)
        self.cmd_angular_x_lbl.setGeometry(QtCore.QRect(380, 60, 58, 17))
        self.cmd_angular_x_lbl.setObjectName(_fromUtf8("cmd_angular_x_lbl"))
        self.send_cmd_btn = QtGui.QPushButton(Form)
        self.send_cmd_btn.setGeometry(QtCore.QRect(220, 210, 121, 31))
        self.send_cmd_btn.setObjectName(_fromUtf8("send_cmd_btn"))
        self.cmd_linear_z_lbl = QtGui.QLabel(Form)
        self.cmd_linear_z_lbl.setGeometry(QtCore.QRect(240, 140, 58, 17))
        self.cmd_linear_z_lbl.setObjectName(_fromUtf8("cmd_linear_z_lbl"))
        self.cmd_linear_x_sbx = QtGui.QSpinBox(Form)
        self.cmd_linear_x_sbx.setGeometry(QtCore.QRect(180, 60, 52, 27))
        self.cmd_linear_x_sbx.setObjectName(_fromUtf8("cmd_linear_x_sbx"))
        self.cmd_linea_x_lbl = QtGui.QLabel(Form)
        self.cmd_linea_x_lbl.setGeometry(QtCore.QRect(240, 60, 58, 17))
        self.cmd_linea_x_lbl.setObjectName(_fromUtf8("cmd_linea_x_lbl"))

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.pose_x_lbl.setText(_translate("Form", "X", None))
        self.pose_angular_lbl.setText(_translate("Form", "angular", None))
        self.pose_tetha_lbl.setText(_translate("Form", "theta", None))
        self.cmd_linear_y_lbl.setText(_translate("Form", "Y", None))
        self.pose_linear_lbl.setText(_translate("Form", "linear", None))
        self.cmd_angular_z_lbl.setText(_translate("Form", "Z", None))
        self.cmd_linear_lbl.setText(_translate("Form", "Linear", None))
        self.pose_y_lbl.setText(_translate("Form", "Y", None))
        self.cmd_angular_lbl.setText(_translate("Form", "Y", None))
        self.cmd_Angular_lbl.setText(_translate("Form", "Angular", None))
        self.cmd_angular_x_lbl.setText(_translate("Form", "X", None))
        self.send_cmd_btn.setText(_translate("Form", "Send Command", None))
        self.cmd_linear_z_lbl.setText(_translate("Form", "Z", None))
        self.cmd_linea_x_lbl.setText(_translate("Form", "X", None))

