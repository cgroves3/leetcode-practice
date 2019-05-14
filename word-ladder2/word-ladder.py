import heapq

def getConnectedWords(word, wordList):
    words = []
    for w in wordList:
        if getWordDifferenceCount(w, word) == 1:
            words.append(w)
    return words
        
def getWordDifferenceCount(firstWord, secondWord):
    letter_diff_count = 0
    for i in range(len(secondWord)):
        if not(secondWord[i] == firstWord[i]):
            letter_diff_count += 1
    return letter_diff_count

def genGraphDic(startWord, endWord, wordList):
    if not(endWord in wordList):
        return {}
    
    if len(getConnectedWords(startWord, wordList)) == 0:
        return {}
    
    graph = {}
    for word in wordList:
        if not(word in graph.keys()):
            wL_cpy = wordList[:]
            wL_cpy.remove(word)
            graph[word] = getConnectedWords(word, wL_cpy)
    return graph

def runDijkstra(graph, startingWord, endWord):
    step_cost = 1.0
    
    if len(graph.keys()) == 0:
        return []

    explored = []
    frontier = [(0, startingWord)]
    paths = {startingWord: (0, [[]])}
    while True:
        nodeTup = heapq.heappop(frontier)
        node = nodeTup[1]
        
        # Get path node
        pathList = paths[node][1]
        for path_list in pathList: 
            path_list.append(node)
        paths[node] = (paths[node][0] + step_cost, pathList)
        pathCostTuple = paths[node]

        explored.append(node)
        if node == endWord:
            return paths
        neighbors = getConnectedWords(node, graph.keys())
        for neighbor in neighbors:
            if not(neighbor in explored):
                if not(neighbor in listify(frontier)):
                    pathsListCpy = []
                    for pL in pathCostTuple[1]:
                        pathsListCpy.append(pL.copy())
                    paths[neighbor] = (pathCostTuple[0], pathsListCpy)
                    heapq.heappush(frontier, (pathCostTuple[0] + step_cost, neighbor))
                else:
                    #Append paths to path list if the cost is equal to current total cost
                    if pathCostTuple[0] + step_cost == paths[neighbor][0] + step_cost:
                        neighborPathList = paths[neighbor][1]
                        paths[neighbor] = (paths[neighbor][0], neighborPathList.copy() + pathList.copy())

        del paths[node]

def listify(heapList):
    items = []
    for tup in heapList:
        items.append(tup[1])
    return items

def main():
    beginWord = "hit"
    endWord = "cog"
    wordList = ["hot","dot","dog","lot","log","cog"]
    graph = genGraphDic(beginWord, endWord, wordList)
    paths = runDijkstra(graph, beginWord, endWord)
    print(paths[endWord][1])


if __name__ == "__main__":
    main()