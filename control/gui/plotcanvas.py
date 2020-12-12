from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from PIL import Image
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy

class PlotCanvas(FigureCanvas):

    def __init__(self, parent=None, Map='indoor1'):
        self.Image_PreProcessing(Map)
        img = plt.imread('./images/open.jpg')
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(img, extent=[-50, 100, -50, 50])
        height, width, channels = img.shape
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
        img = plt.imread('./images/open.jpg')
        self.ax.imshow(img, extent=[-50, 100, -50, 50])
        self.draw()

    def Image_PreProcessing(self,Map):
        if Map == 'robocup':
            im = Image.open('./images/robocup_world.jpg')
            print('robocup')
        else:
            im = Image.open('./images/xt.jpg')
        imBackground = im.resize((876, 587))  #701 470 840 564
        imBackground.save('./images/open.jpg')
