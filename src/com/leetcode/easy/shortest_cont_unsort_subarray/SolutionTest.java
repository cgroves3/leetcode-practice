package com.leetcode.easy.shortest_cont_unsort_subarray;

import org.junit.Assert;
import org.junit.Test;

public class SolutionTest {
    @Test
    public void run() {
        Solution sol = new Solution();

        int[] array = new int[] { 2, 6, 4, 8, 10, 9, 15 };
        Assert.assertEquals(5, sol.findUnsortedSubarray(array));

        array = new int[] { 2, 6, 4, 8, 9, 10, 15 };
        Assert.assertEquals(2, sol.findUnsortedSubarray(array));

        array = new int[] { 2, 4, 6, 8, 9, 10, 15 };
        Assert.assertEquals(0, sol.findUnsortedSubarray(array));

        array = new int[] { 1,2,3,4};
        Assert.assertEquals(0, sol.findUnsortedSubarray(array));

        array = new int[] { 2,1 };
        Assert.assertEquals(2, sol.findUnsortedSubarray(array));
    }
}
