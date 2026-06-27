import matplotlib.pyplot as plt
import numpy as np

class DraggablePointInterface:
    def __init__(self, p):
        self.point = p # p is patch, aka a circle or something
        self.dragging = False
    
    def near_point(self, x1, y1, x2, y2, threshold=0.1):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2) <= threshold
    
    def connect(self):
        'connect to all the events we need'
        self.cidpress = self.point.figure.canvas.mpl_connect(
            'button_press_event', self.on_press)
        self.cidrelease = self.point.figure.canvas.mpl_connect(
            'button_release_event', self.on_release)
        self.cidmotion = self.point.figure.canvas.mpl_connect(
            'motion_notify_event', self.on_motion)
    def disconnect(self):
        'disconnect all the stored connection ids'
        self.point.figure.canvas.mpl_disconnect(self.cidpress)
        self.point.figure.canvas.mpl_disconnect(self.cidrelease)
        self.point.figure.canvas.mpl_disconnect(self.cidmotion)
    def on_press(self, event):
        if event.inaxes != self.point.axes: return
        if self.near_point(event.xdata, event.ydata, self.point.center[0], self.point.center[1]):
            self.dragging = True

        else:
            return
    def on_motion(self, event):
        if not self.dragging: return
        if event.inaxes != self.point.axes: return
        x, y = event.xdata, event.ydata
        x, y = self._clamp_to_bounds(x,y)
#        update arm with fwd km on clamped axis
#       self.point.center = (event.xdata, event.ydata)
#       change all angles based on inv km
#       self.point.figure.canvas.draw()


        
    def on_release(self, event):
        self.dragging = False