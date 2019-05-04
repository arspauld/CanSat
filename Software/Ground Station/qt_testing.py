import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QWidget, QLCDNumber, QSlider, 
    QVBoxLayout, QApplication)
import PyQt5


class Example(QWidget):
    
    def __init__(self):
        super().__init__()
        
        self.initUI()
        
    def print_(self, event):
        print(self.sld.value())
        
    def initUI(self):
        
        lcd = QLCDNumber(self)
        self.sld = QSlider(Qt.Horizontal, self)

        vbox = QVBoxLayout()
        vbox.addWidget(lcd)
        vbox.addWidget(self.sld)

        self.setLayout(vbox)
        self.sld.valueChanged.connect(lcd.display)
        self.sld.valueChanged.connect(self.print_)

        
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Signal and slot')
        self.show()
        

if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())