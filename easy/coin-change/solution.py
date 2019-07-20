import math
import sys

class Solution(object):
    def coinChange(self, coins, amount):

        if amount == 0:
            return 0

        if len(coins) == 0:
            return -1
        
        if min(coins) > amount or len(coins) == 1 and amount % coins[0] > 0:
            return -1

        
        return self.coinChangeCounts(coins, amount, [0] * amount)
    
    def coinChangeCounts(self, coins, amount, counts):
        if amount < 0:
            return -1
        
        if amount == 0:
            return 0
        
        if counts[amount - 1] != 0:
            return counts[amount - 1]
            
        min_coin_count = sys.maxint
        for i in range(len(coins)):
            with_coin = self.coinChangeCounts(coins, amount - coins[i], counts)

            if with_coin >= 0 and with_coin < min_coin_count or min_coin_count == -1:
                min_coin_count = with_coin + 1

        counts[amount - 1] = -1 if (min_coin_count == sys.maxint) else min_coin_count
        return counts[amount - 1]