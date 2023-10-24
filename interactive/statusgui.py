from tkinter import *

from robot import Robot
from navigation.world import World

class StatusGUI:
    def __init__(self, robot: Robot, world: World):
        self.robot = robot
        self.world = world


        self.root = Tk()
        self.root.geometry('1000x1000')

        self.statusLabel = StringVar()
        self.macAddrsToListenTo = []
        self.statusLabel.set('hello')

        Label(self.root, textvariable=self.statusLabel, width= 1000, height=100)\
            .pack(side="left", fill="y")

        button = Button(self.root, overrelief="solid", width=150, height=20,command=self.rotate, repeatdelay=1000,
                                repeatinterval=100, text="rotate 15")
        button.pack(side="right", fill="y")
        button = Button(self.root, overrelief="solid", width=150, height=20, command=self.go, repeatdelay=1000,
                                repeatinterval=100, text="go 50")
        button.pack(side="right", fill="y")


        self.root.after(100, self.updateGUI)
        self.root.mainloop()
    def go(self):
        self.robot.platform.goForward(50)
    def rotate(self):
        self.robot.platform.rotateCW(15)


    def updateGUI(self):
        currrouters = '\n'.join([str(router) for router in self.robot.routers])
        targetrouters = '\n'.join(self.macAddrsToListenTo)
        self.statusLabel.set(f'Orientation: {self.robot.orientation}\nFloor: {self.robot.ultrasonic}\nRouters\n{currrouters}\n\nTarget:{targetrouters}')
        self.root.after(100, self.updateGUI)