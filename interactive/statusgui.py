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
        self.statusLabel.set('hello')

        Label(self.root, textvariable=self.statusLabel, width= 500, height=10).pack()

        self.root.after(500, self.updateGUI)
        self.root.mainloop()

    def updateGUI(self):
        str = ''
        for router in self.robot.routers:
            str = str + '\n' + router
        self.statusLabel.set(f'Robot: {self.robot.orientation}\nRouters\n{str}')
        self.root.after(500, self.updateGUI)