from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from PIL import Image
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy

class PlotCanvas(FigureCanvas):

    def __init__(self, parent=None, Map='indoor1'):
        self.fig, self.ax = plt.subplots()
        self.Image_PreProcessing(Map)
        height, width, channels = self.imBackground.shape
        self.fig.set_size_inches(width/100.0, height/100.0)
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())
        plt.subplots_adjust(top=1, bottom=0, left=0, right=1, hspace=0, wspace=0)
        plt.margins(0, 0)
        plt.axis('off')
        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,  QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def canvas_update(self, Map):
        self.Image_PreProcessing(Map)
        # img = plt.imread('./images/open.jpg')
        # self.ax.imshow(img, extent=[-50, 100, -50, 50])
        self.draw()

    def Image_PreProcessing(self,Map):
        if Map == 'robocup':
            self.imBackground = plt.imread('./images/robocup_resize.jpg')  # image size:875*587
            self.ax.imshow(self.imBackground, extent=[-50, 150, -67.09, 67.09])  #time: 875/150+50
        elif Map == 'indoor1':
            self.imBackground = plt.imread('./images/indoor1_resize.jpg')
            self.ax.imshow(self.imBackground, extent=[-16.89, 18.89, -14, 10])  #[-14, 10, -8.9, 6.9]
        elif Map == 'indoor2':
            self.imBackground = plt.imread('./images/indoor2_resize.jpg')
            self.ax.imshow(self.imBackground, extent=[-9.925, 13.925, -6, 10])
        elif Map == 'indoor3':
            self.imBackground = plt.imread('./images/indoor3_resize.jpg')
            self.ax.imshow(self.imBackground, extent=[-11.16, 17.16, -3, 16])
        elif Map == 'outdoor1':
            self.imBackground = plt.imread('./images/outdoor1_resize.jpg')
            self.ax.imshow(self.imBackground, extent=[-50, 150, -67.09, 67.09])
        elif Map == 'outdoor2':
            self.imBackground = plt.imread('./images/outdoor2_resize.jpg')
            self.ax.imshow(self.imBackground, extent=[-2235, 2235, -1500, 1500])
        elif Map == 'outdoor3':
            self.imBackground = plt.imread('./images/outdoor3_resize.jpg')
            self.ax.imshow(self.imBackground, extent=[-1129.5, 465.5, -380, 690])
        
        
