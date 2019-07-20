class Graph:
    def __init__(self, nodes):
        self._nodes = nodes

    def getEdgeWeight(self, node1, node2):
        if node1 == node2 or node1 == '':
            return 0

        count = self._getWordDifferenceCount(node1, node2)
        if count > 1:
            return None
        else:
            return count

    def nodes(self):
        return self._nodes

    def getNeighbors(self, node):
        return self._getConnectedWords(node, self._nodes)
    
    def _getConnectedWords(self, word, wordList):
        words = {}
        for w in wordList:
            if self._getWordDifferenceCount(w, word) == 1:
                words[w] = 1
        return words
    
    def _getWordDifferenceCount(self, firstWord, secondWord):
        letter_diff_count = 0
        for i in range(len(secondWord)):
            if not(secondWord[i] == firstWord[i]):
                letter_diff_count += 1
        return letter_diff_count