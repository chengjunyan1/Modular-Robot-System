import numpy as np

g_dir = [[1, 0], [0, 1], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]]
class JPSNode:
    def __init__(self, parent, pos, g, h):
        self.parent = parent
        self.pos = pos
        self.g = g
        self.h = h
        self.f = g + h
    def get_direction(self):
        return self.parent and [self.pos[0] != self.parent.pos[0] and (self.pos[0] - self.parent.pos[0]) / abs(self.pos[0] - self.parent.pos[0]) or 0, self.pos[1] != self.parent.pos[1] and (self.pos[1] - self.parent.pos[1]) / abs(self.pos[1] - self.parent.pos[1]) or 0] or [0, 0]

class JPS:
    def __init__(self, cm):
        self.s_pos = None
        self.e_pos = JPSNode
        self.map = cm
        self.width = cm.shape[0]
        self.height = cm.shape[1]
        self.open = []
        self.close = []
        self.path = []
        
    def findPath(self, s_pos, e_pos):
        self.s_pos, self.e_pos = s_pos, e_pos
        p = JPSNode(None, self.s_pos, 0, abs(self.s_pos[0]-self.e_pos[0]) + abs(self.s_pos[1]-self.e_pos[1]))
        self.open.append(p)
        while True:
            if not self.open:
                return
            idx, p = self.get_min_f_node()
            if self.is_target(p):
                self.make_path(p)
                return
            self.extend_round(p)
            self.close.append(p)
            del self.open[idx]
            
    def prune_neighbours(self, c):
        nbs = []
        if c.parent:
            dir = c.get_direction()
            if self.is_pass(c.pos[0] + dir[0], c.pos[1] + dir[1]):
                nbs.append([c.pos[0] + dir[0], c.pos[1] + dir[1]])
            if dir[0] != 0 and dir[1] != 0:
                if self.is_pass(c.pos[0], c.pos[1] + dir[1]):
                    nbs.append([c.pos[0], c.pos[1] + dir[1]])
                if self.is_pass(c.pos[0]+dir[0], c.pos[1]):
                    nbs.append([c.pos[0]+dir[0], c.pos[1]])
                if not self.is_pass(c.pos[0] - dir[0], c.pos[1]) and self.is_pass(c.pos[0], c.pos[1] + dir[1]):
                    nbs.append([c.pos[0] - dir[0], c.pos[1] + dir[1]])
                if not self.is_pass(c.pos[0], c.pos[1]-dir[1]) and self.is_pass(c.pos[0]+dir[0], c.pos[1]):
                    nbs.append([c.pos[0]+dir[0], c.pos[1]-dir[1]])
            else:
                if dir[0] == 0:
                    if not self.is_pass(c.pos[0]+1, c.pos[1]):
                        nbs.append([c.pos[0]+1, c.pos[1]+dir[1]])
                    if not self.is_pass(c.pos[0]-1, c.pos[1]):
                        nbs.append([c.pos[0]-1, c.pos[1]+dir[1]])
                else:                  
                    if not self.is_pass(c.pos[0], c.pos[1]+1):
                         nbs.append([c.pos[0]+dir[0], c.pos[1]+1])
                    if not self.is_pass(c.pos[0], c.pos[1]-1):
                         nbs.append([c.pos[0]+dir[0], c.pos[1]-1])
        else:
            for d in g_dir:
                if self.is_pass(c.pos[0] + d[0], c.pos[1] + d[1]):
                    nbs.append([c.pos[0] + d[0], c.pos[1] + d[1]])
        return nbs

    def jump_node(self, now, pre):
        dir = [a != b and (a - b)/abs(a-b) or 0 for a, b in zip(now, pre)]
        if now == self.e_pos:
            return now
        if self.is_pass(now[0], now[1]) is False:
            return None
        if dir[0] != 0 and dir[1] != 0:
            if (self.is_pass(now[0] - dir[0], now[1] + dir[1]) and not self.is_pass(now[0]-dir[0], now[1])) or (self.is_pass(now[0] + dir[0], now[1] - dir[1]) and not self.is_pass(now[0], now[1]-dir[1])):
                return now
        else:
            if dir[0] != 0:
                if (self.is_pass(now[0] + dir[0], now[1] + 1) and not self.is_pass(now[0], now[1]+1)) or (self.is_pass(now[0] + dir[0], now[1] - 1) and not self.is_pass(now[0], now[1]-1)):
                    return now
            else:
                if (self.is_pass(now[0] + 1, now[1] + dir[1]) and not self.is_pass(now[0]+1, now[1])) or (self.is_pass(now[0] - 1, now[1] + dir[1]) and not self.is_pass(now[0]-1, now[1])):
                    return now
        if dir[0] != 0 and dir[1] != 0:
            t1 = self.jump_node([now[0]+dir[0], now[1]], now)
            t2 = self.jump_node([now[0], now[1] + dir[1]], now)
            if t1 or t2:
                return now
        if self.is_pass(now[0] + dir[0], now[1]) or self.is_pass(now[0], now[1] + dir[1]):
            t = self.jump_node([now[0] + dir[0], now[1] + dir[1]], now)
            if t:
                return t
        return None

    def extend_round(self, c):
        nbs = self.prune_neighbours(c)
        for n in nbs:
            jp = self.jump_node(n,[c.pos[0], c.pos[1]])
            if jp:
                if self.node_in_close(jp):
                    continue
                g = self.get_g(jp, c.pos)
                h = self.get_h(jp, self.e_pos)
                node = JPSNode(c, jp, c.g + g, h)
                i = self.node_in_open(node)
                if i != -1:
                    if self.open[i].g > node.g:
                        self.open[i].parent = c
                        self.open[i].g = node.g
                        self.open[i].f = node.g + self.open[i].h
                    continue
                self.open.append(node)

    def is_pass(self, x, y):
        x=int(x)
        y=int(y)
        return x >= 0 and x < self.width and y >= 0 and y < self.height and (self.map[x][y] != 1 or [x, y] == self.e_pos)

    def make_path(self, p):
        while p:
            if p.parent:
                dir = p.get_direction()
                n = p.pos
                while n != p.parent.pos:
                    self.path.append(n)
                    n = [n[0] - dir[0], n[1] - dir[1]]
            else:
                self.path.append(p.pos)
            p = p.parent
        self.path.reverse()

    def is_target(self, n):
        return n.pos == self.e_pos

    def get_min_f_node(self):
        best = None
        bv = -1
        bi = -1
        for idx, node in enumerate(self.open):
            if bv == -1 or node.f < bv:  
                best = node
                bv = node.f
                bi = idx
        return bi, best

    def get_g(self, pos1, pos2):
        if pos1[0] == pos2[0]:
            return abs(pos1[1] - pos2[1])
        elif pos1[1] == pos2[1]:
            return abs(pos1[0] - pos2[0])
        else:
            return abs(pos1[0] - pos2[0]) * 1.4

    def get_h(self, pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def node_in_close(self, node):
        for i in self.close:
            if node == i.pos:
                return True
        return False

    def node_in_open(self, node):
        for i, n in enumerate(self.open):
            if node == n.pos:
                return i
        return -1