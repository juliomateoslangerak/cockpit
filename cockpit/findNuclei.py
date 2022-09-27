#!/usr/bin/python3
# Short example script of attaching to cockpit to find nuclei in DAPI images.
#Copyright Ian Dobbie 2021

# use opencv hough circle routine to find nuclei
import cv2
import wx
# some imports to get at cockpit functions
from cockpit import events
from cockpit.interfaces import stageMover
from cockpit.gui.mosaic import window as mosaicWindow
from cockpit import depot

# find nuceli class
class findNuclei():
    def __init__(self,*args, **kwargs):
    # useful values form cockpit objects
        self.pixelsize=wx.GetApp().Objectives.GetPixelSize()
        self.siteColor= mosaicWindow.window.siteColor
        self.siteSize = mosaicWindow.window.crosshairBoxSize
        self.camera = depot.getDeviceWithName('camera')

    #start finding Nuclei in images.
    def start(self):
        #subscribe to new image event and call onImage
        events.subscribe(events.NEW_IMAGE % self.camera.name, self.onImage)
 
    ## Receive a new image and process it to find nuclei
    def onImage(self, data, *args):
        #grab image data and stage position
        img=data
        position=stageMover.getPosition()

        #blur and then binarise bin
        ret,bin=cv2.threshold(cv2.GaussianBlur(img,(15,15),3),
                              60,255,cv2.THRESH_BINARY)    
        #use opencv to find relevant sized circles
        circles = cv2.HoughCircles(bin,cv2.HOUGH_GRADIENT,1,100,
                                   param1=200,param2=10,minRadius=80,
                                   maxRadius=150)
        #for each found circle mark a positon
        if circles is not None:
            #loop over cicrles found in image 
            for cell in circles[0]:
                #covert pos to microns and add stage pos.
                cellpos=position[:2]+([img.shape[0]/2,
                                       img.shape[1]/2]-
                                      cell[:2])*self.pixelsize
                #append Z positon to get xyz pos.
                sitepos=[cellpos[0],cellpos[1],position[2]]
                # save site to marked point list.
                stageMover.saveSite(stageMover.Site(sitepos,None,self.siteColor,
                                               size=self.siteSize))
    # stop finding Nuclei in images. 
    def stop(self):
        events.unsubscribe(events.NEW_IMAGE % self.camera.name, self.onImage)


