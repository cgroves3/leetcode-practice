package com.leetcode.easy.shortest_cont_unsort_subarray;

public class Solution {
    public int findUnsortedSubarray(int[] nums) {
        int firstDrop = -1;
        int lastDrop = -1;

        int i = 0;
        int j = nums.length - 1;

        //Get the first number less than the previous number.
        while (i < nums.length - 1 && firstDrop == -1) {
            if (nums[i + 1] < nums[i]){
                firstDrop = i;
            }
            i++;
        }

        //Get the last number lass than the previous number.
        while (j > 0 && lastDrop == -1) {
            if (nums[j - 1] > nums[j]) {
                lastDrop = j;
            }
            j--;
        }

        if (firstDrop == -1 || lastDrop == -1)
            return 0;

        int length = lastDrop - firstDrop + 1;
        return length;
    }
}
