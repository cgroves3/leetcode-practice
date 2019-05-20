class Graph:
    def __init__(self, startWord, endWord, wordList):
        self.graph = self.genGraphDic(startWord, endWord, wordList)
    
    def isEmpty(self):
        return len(self.graph.keys()) == 0

    def genGraphDic(self, startWord, endWord, wordList):
        if not(endWord in wordList):
            return {}
        
        if len(self.getConnectedWords(startWord, wordList)) == 0:
            return {}
        
        graph = { startWord : self.getConnectedWords(startWord,wordList) }
        for word in wordList:
            if not(word in graph.keys()):
                wL_cpy = list(wordList)
                wL_cpy.remove(word)
                graph[word] = self.getConnectedWords(word, wL_cpy)
        return graph

    def getEdgeWeight(self, node1, node2):
        nodeTup = self.graph.get(node1, '')
        if not nodeTup == '':
            return nodeTup.get(node2, 0)
        return 0

    def getNeighbors(self, node):
        neighborDic = self.graph.get(node, '')
        return neighborDic.keys()
    
    def getConnectedWords(self, word, wordList):
        words = {}
        for w in wordList:
            if self.getWordDifferenceCount(w, word) == 1:
                words[w] = 1
        return words
    
    def getWordDifferenceCount(self, firstWord, secondWord):
        letter_diff_count = 0
        for i in range(len(secondWord)):
            if not(secondWord[i] == firstWord[i]):
                letter_diff_count += 1
        return letter_diff_count