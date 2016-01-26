__author__ = 'nick'

from PyQt4 import QtGui, QtCore

class Window(QtGui.QWidget):
    def __init__(self, rows, columns):
        QtGui.QWidget.__init__(self)
        self.table = QtGui.QTableWidget(rows, columns, self)
        self.table.setSelectionMode(QtGui.QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        header = self.table.horizontalHeader()
        header.setResizeMode(QtGui.QHeaderView.Fixed)
        header.setDefaultSectionSize(25)
        header.hide()
        header = self.table.verticalHeader()
        header.setResizeMode(QtGui.QHeaderView.Fixed)
        header.setDefaultSectionSize(25)
        header.hide()
        for row in range(rows):
            # item = QtGui.QTableWidgetItem('0x00')
            # self.table.setVerticalHeaderItem(row, item)
            for column in range(columns):
                item = QtGui.QTableWidgetItem()
                item.setBackground(QtCore.Qt.white)
                self.table.setItem(row, column, item)
        self.table.itemPressed.connect(self.handleItemPressed)
        layout = QtGui.QVBoxLayout(self)
        layout.addWidget(self.table)

    def handleItemPressed(self, item):
        if item.background().color() == QtCore.Qt.black:
            item.setBackground(QtCore.Qt.white)
        else:
            item.setBackground(QtCore.Qt.black)

if __name__ == '__main__':

    import sys
    app = QtGui.QApplication(sys.argv)
    window = Window(4, 8)
    window.resize(300, 150)
    window.show()
    sys.exit(app.exec_())