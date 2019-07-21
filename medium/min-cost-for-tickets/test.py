import unittest
from solution import Solution

class TestSolution(unittest.TestCase):
    def testCase(self):
        days = [1,4,6,7,8,20]
        costs = [2,7,15]
        solution = Solution()
        self.assertEquals(11, solution.mincostTickets(days, costs))

    def testCase2(self):
        days = [1,2,3,4,5,6,7,8,9,10,30,31]
        costs = [2,7,15]
        solution = Solution()
        self.assertEquals(17, solution.mincostTickets(days, costs))

if __name__ == "__main__":
    unittest.main()