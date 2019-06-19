class Solution(object):
    def checkPossibility(self, nums):
        """
        :type nums: List[int]
        :rtype: bool
        """
        counter_limit = 1
        max_num = nums[0]
        lower_count = 0
        higher_count = 0
        prev_ind = 0
        for i in range(1, len(nums)):
            if nums[i] > max_num:
                max_num = nums[i] 
            if nums[i] < max_num:
                lower_count += 1
            if nums[i] < nums[i-1]:
                for j in range(prev_ind, i):
                    if nums[j] > nums[i]:
                        higher_count += 1
                prev_ind = i
        return min(higher_count, lower_count) <= counter_limit