__author__ = 'nick'

from PyQt4 import QtCore, QtGui
import sys
import astar_gui

class AstarApp(QtGui.QMainWindow, astar_gui.Ui_MainWindow):
    def __init__(self, parent=None):
        super(AstarApp, self).__init__(parent)
        self.setupUi(self)
        self.start = None
        self.end = None
        self.bttnGenerateGrid.pressed.connect(self.handleGeneratePressed)

    def handleGeneratePressed(self):
        self.tblMap.clear()
        self.tblMap.setRowCount(0)
        num_cells = self.worldSize_spinbox.value()
        print(num_cells)
        section_size = 400/num_cells
        print(section_size)
        self.tblMap.horizontalHeader().setDefaultSectionSize(section_size)
        self.tblMap.verticalHeader().setDefaultSectionSize(section_size)
        self.tblMap.setRowCount(num_cells)
        self.tblMap.setColumnCount(num_cells)
        for row in range(num_cells):
            for column in range(num_cells):
                item = QtGui.QTableWidgetItem()
                item.setBackground(QtCore.Qt.white)
                self.tblMap.setItem(row, column, item)


def main():
    app = QtGui.QApplication(sys.argv)
    form = AstarApp()
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()
