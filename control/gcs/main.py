from PyQt5.QtWidgets import QApplication,QMainWindow
from send import Gui2Ros
import sys

if __name__== '__main__':
    app = QApplication(sys.argv)
    ui = Gui2Ros()
    ui.show()
    sys.exit(app.exec_())






