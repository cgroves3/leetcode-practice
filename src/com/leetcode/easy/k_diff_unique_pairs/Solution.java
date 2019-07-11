package com.leetcode.easy.k_diff_unique_pairs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class Solution {

    private boolean hasOtherTarget(int[] nums, int currentIndex, int target) {
        int low = Arrays.binarySearch(nums, 0, currentIndex, target);
        int high = Arrays.binarySearch(nums, currentIndex + 1, nums.length, target);
        return low > -1 || high > -1;
    }

    public int findPairs(int[] nums, int k) {
        int count = 0;
        Arrays.sort(nums);
        List<Integer> paired = new ArrayList<Integer>();

        if (k < 0)
            return 0;

        for (int i = 0; i < nums.length; i++) {
            int number = nums[i];
            int target = number + k;

            //Ignore copies of the same number
            if (!paired.contains(number)) {
                paired.add(number);
                if (hasOtherTarget(nums, i, target)) {
                    if (!paired.contains(target))
                        paired.add(target);
                    count++;
                }
                //Ignore searching for the target twice, since num - k and num + k are the same
                if (k != 0) {
                    target = number - k;
                    if (hasOtherTarget(nums, i, target)) {
                        if (!paired.contains(target))
                            paired.add(target);
                        count++;
                    }
                }
            }
        }

        return count;
    }
}
