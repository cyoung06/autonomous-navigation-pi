import json
import logging
import time
from threading import Thread

import pygame
from pygame import DOUBLEBUF, OPENGL
from websocket_server import WebsocketServer

from robot import Robot
from navigation.world import World
from OpenGL.GL import *
from OpenGL.GLU import *


class StatusGUI:
    def __init__(self, robot: Robot, world: World):
        self.world = world
        self.robot = robot

        self.server = WebsocketServer(host='0.0.0.0', port=13254, loglevel=logging.INFO)
        self.server.set_fn_new_client(self.new_client)
        self.server.run_forever()

        self.nodeMap = {}

        Thread(target=self.server.run_forever, daemon=True).start()
        Thread(target=self.sendStuff, daemon=True).start()

    def sendStuff(self):
        while True:
            self.sendMeasurements()
            time.sleep(0.5)


    def newCell(self, cell):
        self.server.send_message_to_all(json.dumps({
            "type": "newCell",
            "data": {
                "id": self.getNodeID(cell),
                "vector": cell.position.tolist()
            }
        }))

    def newConnection(self, connection):
        arr = []
        for conn in connection:
            arr.append( {
                "origin": self.getNodeID(conn.origin),
                "target": self.getNodeID(conn.target),
                "relPos": [conn.rel_pos.dx, conn.rel_pos.dy, conn.rel_pos.rot]
            })
        self.server.send_message_to_all(json.dumps({
            "type": "newConnection",
            "data": arr
        }))

    def focus(self, cell):
        self.server.send_message_to_all(json.dumps({
            "type": "focus",
            "data": self.getNodeID(cell)
        }))

    def new_client(self, client, server):
        self.sendNodes()

    def sendNodes(self):
        array = []
        for node in self.world.nodes:
            nodeJSON = {
                "id": self.getNodeID(node),
                "vector": node.position.tolist()
            }
            array.append(nodeJSON)
        array2 = []
        for node in self.world.nodes:
            for neighbor in node.neighbors:
                neighborJSON = {
                    "origin": self.getNodeID(neighbor.origin),
                    "target": self.getNodeID(neighbor.target),
                    "relPos": [neighbor.rel_pos.dx, neighbor.rel_pos.dy, neighbor.rel_pos.rot]
                }
                array2.append(neighborJSON)
        self.server.send_message_to_all(json.dumps({
            "type": "nodes",
            "data": {
                "nodes": array,
                "connections": array2
            }
        }))

    def sendMeasurements(self):
        self.server.send_message_to_all(json.dumps({
            "type": "measurements",
            "data": {
                "orientation": self.robot.orientation,
                "routers": self.robot.routers,
                "ultrasonic": self.robot.ultrasonic
            }
        }))


    def getNodeID(self, node):
        if node in self.nodeMap:
            return self.nodeMap[node]
        self.nodeMap[node] = len(self.nodeMap)
        return self.nodeMap[node]
