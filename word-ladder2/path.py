class Path:
    def __init__(self, nodes=None, cost=0):
        self._cost = cost
        if nodes is None:
            self._nodes = []
        else:
            self._nodes = nodes
    
    def getLastNode(self):
        if len(self._nodes) == 0:
            return ''
        return self._nodes[len(self._nodes) - 1]
    
    def getNodes(self):
        return self._nodes
    
    def addNode(self, node, cost):
        self._nodes.append(node)
        self._cost += cost
    
    def removeNode(self, node, cost):
        self._nodes.remove(node)
        self._cost -= cost

    def copy(self):
        return Path(list(self._nodes), self._cost)
    
    def cost(self):
        return self._cost
    
    def nodes(self):
        return self._nodes
    
    def reverse(self):
        self._nodes.reverse()
    
    def merge(self, path):
        if self.getLastNode() != path.nodes()[0]:
            raise ContinuityError()
        path_copy = self.copy()
        path_copy._cost += path.cost()
        path_copy._nodes = path_copy.nodes() + path.nodes()[1:]
        return path_copy
    
    def __eq__(self, other): 
        if not isinstance(other, Path):
            return NotImplemented

        return self._cost == other.cost() and self._nodes == other.nodes()
    
class ContinuityError(Exception):
    def __init__(self, message="Last node of current path and first node of target path are not the same. Resulting path doesn't appear to be similar."):
        self.message = message