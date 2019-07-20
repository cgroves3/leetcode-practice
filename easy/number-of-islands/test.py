import unittest
from solution import Solution

class TestSolution(unittest.TestCase):
    def testCase(self):
        grid = [["1","1","1","1","0"],["1","1","0","1","0"],["1","1","0","0","0"],["0","0","0","0","0"]]
        result = 1
        solution = Solution()
        self.assertEqual(result, solution.numIslands(grid))

    def testCase2(self):
        grid = [["1","1","0","0","0"],["1","1","0","0","0"],["0","0","1","0","0"],["0","0","0","1","1"]]
        result = 3
        solution = Solution()
        self.assertEqual(result, solution.numIslands(grid))

if __name__ == '__main__':
    unittest.main()