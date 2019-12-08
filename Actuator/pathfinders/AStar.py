import numpy as np

class map2d:
    def __init__(self,cm):
        self.data = cm
        self.w = cm.shape[0]
        self.h = cm.shape[1]
        self.passTag = 0
        self.pathTag = 7

    def isPass(self, point):
        if (point.x < 0 or point.x > self.h - 1) or (point.y < 0 or point.y > self.w - 1):
            return False
        if self.data[point.x][point.y] == self.passTag:
            return True

class Point:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

class Node:     
    def __init__(self, point, g = 0, h = 0):  
        self.point = point         
        self.father = None        
        self.g = g                
        self.h = h                  

    def manhattan(self, endNode):
        self.h = (abs(endNode.point.x - self.point.x) + abs(endNode.point.y - self.point.y))*10 

    def setG(self, g):
        self.g = g

    def setFather(self, node):
        self.father = node

class AStar:
    def __init__(self, cm):
        self.openList = []
        self.closeList = []
        self.map2d = map2d(cm)
    
    def findPath(self,sp, ep):
        startNode=Node(Point(sp[0],sp[1]))
        endNode=Node(Point(ep[0],ep[1]))
        self.startNode = startNode
        self.endNode = endNode 
        self.currentNode = startNode
        self.pathlist = []
        self.start()
        path=[]
        for i in self.pathlist:
            path.append([i.point.x,i.point.y])
        self.path=path

    def getMinFNode(self):
        nodeTemp = self.openList[0]  
        for node in self.openList:  
            if node.g + node.h < nodeTemp.g + nodeTemp.h:  
                nodeTemp = node  
        return nodeTemp

    def nodeInOpenlist(self,node):
        for nodeTmp in self.openList:  
            if nodeTmp.point.x == node.point.x \
            and nodeTmp.point.y == node.point.y:  
                return True  
        return False

    def nodeInCloselist(self,node):
        for nodeTmp in self.closeList:  
            if nodeTmp.point.x == node.point.x \
            and nodeTmp.point.y == node.point.y:  
                return True  
        return False

    def endNodeInOpenList(self):  
        for nodeTmp in self.openList:  
            if nodeTmp.point.x == self.endNode.point.x \
            and nodeTmp.point.y == self.endNode.point.y:  
                return True  
        return False

    def getNodeFromOpenList(self,node):  
        for nodeTmp in self.openList:  
            if nodeTmp.point.x == node.point.x \
            and nodeTmp.point.y == node.point.y:  
                return nodeTmp  
        return None

    def searchOneNode(self,node):
        if self.map2d.isPass(node.point) != True:  
            return  
        if self.nodeInCloselist(node):  
            return  
        if abs(node.point.x - self.currentNode.point.x) == 1 and abs(node.point.y - self.currentNode.point.y) == 1:  
            gTemp = 14  
        else:  
            gTemp = 10  

        if self.nodeInOpenlist(node) == False:
            node.setG(gTemp)
            node.manhattan(self.endNode)
            self.openList.append(node)
            node.father = self.currentNode
        else:
            nodeTmp = self.getNodeFromOpenList(node)
            if self.currentNode.g + gTemp < nodeTmp.g:
                nodeTmp.g = self.currentNode.g + gTemp  
                nodeTmp.father = self.currentNode  
        return

    def searchNear(self):
        if self.map2d.isPass(Point(self.currentNode.point.x - 1, self.currentNode.point.y)) and \
        self.map2d.isPass(Point(self.currentNode.point.x, self.currentNode.point.y -1)):
            self.searchOneNode(Node(Point(self.currentNode.point.x - 1, self.currentNode.point.y - 1)))

        self.searchOneNode(Node(Point(self.currentNode.point.x - 1, self.currentNode.point.y)))

        if self.map2d.isPass(Point(self.currentNode.point.x - 1, self.currentNode.point.y)) and \
        self.map2d.isPass(Point(self.currentNode.point.x, self.currentNode.point.y + 1)):
            self.searchOneNode(Node(Point(self.currentNode.point.x - 1, self.currentNode.point.y + 1)))

        self.searchOneNode(Node(Point(self.currentNode.point.x, self.currentNode.point.y - 1)))
        self.searchOneNode(Node(Point(self.currentNode.point.x, self.currentNode.point.y + 1)))

        if self.map2d.isPass(Point(self.currentNode.point.x, self.currentNode.point.y - 1)) and \
        self.map2d.isPass(Point(self.currentNode.point.x + 1, self.currentNode.point.y)):
            self.searchOneNode(Node(Point(self.currentNode.point.x + 1, self.currentNode.point.y - 1)))

        self.searchOneNode(Node(Point(self.currentNode.point.x + 1, self.currentNode.point.y)))

        if self.map2d.isPass(Point(self.currentNode.point.x + 1, self.currentNode.point.y)) and \
        self.map2d.isPass(Point(self.currentNode.point.x, self.currentNode.point.y + 1)):
            self.searchOneNode(Node(Point(self.currentNode.point.x + 1, self.currentNode.point.y + 1)))
        return

    def start(self):
        self.startNode.manhattan(self.endNode)
        self.startNode.setG(0)
        self.openList.append(self.startNode)

        while True:
            self.currentNode = self.getMinFNode()
            self.closeList.append(self.currentNode)
            self.openList.remove(self.currentNode)
            self.searchNear()
            if self.endNodeInOpenList():
                nodeTmp = self.getNodeFromOpenList(self.endNode)
                while True:
                    self.pathlist.append(nodeTmp)
                    if nodeTmp.father != None:
                        nodeTmp = nodeTmp.father
                    else:
                        return True
            elif len(self.openList) == 0:
                return False
        return True
