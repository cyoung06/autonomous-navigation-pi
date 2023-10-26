from threading import Thread

import pygame
from pygame import DOUBLEBUF, OPENGL

from robot import Robot
from navigation.world import World
from OpenGL.GL import *
from OpenGL.GLU import *

class StatusGUI:
    def __init__(self, world: World):
        self.world = world
        self.focusedCell = None

        Thread(target=self.run, daemon=True).start()

    def focus(self, cell):
        self.focusedCell = cell

    def run(self):
        pygame.init()

        display = (800, 600)
        pygame.display.set_mode(display)

        screen = pygame.display.set_mode([800, 600], DOUBLEBUF | OPENGL)
        gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
        # Set up the drawing window

        # Run until the user asks to quit
        running = True
        glTranslatef(0.0, 0.0, -5)
        while running:
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

            # Did the user click the window close button?
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            # Fill the background with white
            if self.focusedCell == None:
                continue

            visited = []
            def drawTriangle(cell):
                if cell in visited:
                    return
                visited.append(cell)
                glBegin(GL_TRIANGLE_STRIP)
                glVertex2d(0, 5)
                glVertex2d(3, 9)
                glVertex2d(-3, -9)
                glEnd()
                for neighbor in cell.neighbors:
                    glPushMatrix()
                    glRotate(neighbor.rel_pos.rot, 0, 0, 1)
                    glTranslate(neighbor.rel_pos.dx, neighbor.rel_pos.dy / 50, 0)
                    drawTriangle(neighbor.target)
                    glPopMatrix()
            drawTriangle(self.focusedCell)

            # Flip the display
            pygame.display.flip()
            pygame.time.wait(10)


        # Done! Time to quit.
        pygame.quit()
