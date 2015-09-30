from __future__ import absolute_import

import sys
from PyQt4 import QtCore, QtGui

from turtle_ui import Ui_Form
from turtlecmd_ui import Ui_MainWindow

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


class TurtleWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.turtle_ui = Ui_Form()
        self.turtle_ui.setupUi(self)
        self.turtle_ui.send_cmd_btn.clicked.connect(self.send_cmd)

    def send_cmd(self):
        print 'Send Command pressed.'


class MyWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # setting up tabs from widget
        self.tab = TurtleWidget()
        self.tab.setObjectName(_fromUtf8("turtle"))
        self.ui.tabWidget.addTab(self.tab, _fromUtf8(""))

        self.retranslateUi(parent)

    def retranslateUi(self, MainWindow):
        self.ui.tabWidget.setTabText(self.ui.tabWidget.indexOf(self.tab), _translate("MainWindow", "Tab 1", None))
        #self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Tab 2", None))


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = MyWindow()
    myapp.show()
    sys.exit(app.exec_())
