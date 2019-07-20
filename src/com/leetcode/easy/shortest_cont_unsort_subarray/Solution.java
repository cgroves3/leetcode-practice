package com.leetcode.easy.shortest_cont_unsort_subarray;

import java.util.Arrays;

public class Solution {
    public int findUnsortedSubarray(int[] nums) {
        int startInd = -1;
        int endInd = -1;

        int i = 0;


        int[] nums_copy = nums.clone();
        Arrays.sort(nums_copy);

        //Get the first number less than the previous number.
        while (i < nums.length - 1 && startInd == -1) {
            if (nums[i] != nums_copy[i]) {
                startInd = i;
            }
            i++;
        }
        if (i == nums.length)
            return 0;

        i = nums.length - 1;
        while (i > -1 && endInd == -1) {
            if (nums[i] != nums_copy[i]) {
                endInd = i;
            }
            i--;
        }

        if (startInd == -1 || endInd == -1)
            return 0;

        int length = endInd - startInd + 1;
        return length;
    }
}
