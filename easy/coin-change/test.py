import unittest
from solution import Solution

class TestSolution(unittest.TestCase):
    # def testCase(self):
    #     coins = [1,2,5]
    #     amount = 11
    #     solution = Solution()
    #     self.assertEquals(3, solution.coinChange(coins, amount))

    def testCase2(self):
        coins = [2,5]
        amount = 11
        solution = Solution()
        self.assertEquals(4, solution.coinChange(coins, amount))

    def testCase3(self):
        coins = [5]
        amount = 11
        solution = Solution()
        self.assertEquals(-1, solution.coinChange(coins, amount))
    
    def testCase4(self):
        coins = [5]
        amount = 10
        solution = Solution()
        self.assertEquals(2, solution.coinChange(coins, amount))

    def testCase5(self):
        coins = [1,2,5]
        amount = 100
        solution = Solution()
        self.assertEquals(20, solution.coinChange(coins, amount))

if __name__ == "__main__":
    unittest.main()