class Solution(object):

    def mincostTickets(self, days, costs):
        pass_len = [1,7,30]
        dollars = [len(days) * max(costs) + 1] * (len(days) + 1)
        dollars[0] = 0
        for i in range(len(days)):
            for j in range(len(costs)):
                startDate = days[i] - pass_len[j]
                ind = max(self.getNextDay(days, startDate), 0)
                dollars[i + 1] = min(dollars[i + 1], dollars[ind] + costs[j])
        return dollars[len(dollars)-1]
    
    def getNextDay(self, days, startDate):
        for i in range(len(days)):
            if days[i] > startDate:
                return i
        return -1