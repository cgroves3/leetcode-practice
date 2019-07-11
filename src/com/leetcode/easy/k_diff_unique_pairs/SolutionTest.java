package com.leetcode.easy.k_diff_unique_pairs;


import org.junit.Assert;
import org.junit.Test;

public class SolutionTest {

    @Test
    public void run() {
        Solution solution = new Solution();
        int [] arr = new int[] {3, 1, 4, 1, 5};
//        Assert.assertEquals(2, solution.findPairs(arr, 2));
//        Assert.assertEquals(1, solution.findPairs(arr, 0));
        arr = new int[] {1,2,3,4,5};
        Assert.assertEquals(0, solution.findPairs(arr, -1));
    }

}
