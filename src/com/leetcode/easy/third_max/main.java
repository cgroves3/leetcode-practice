package com.leetcode.easy.third_max;

public class main {
    public static void main(String[] args) {
        Solution sol = new Solution();
        int[] nums = new int[] {1, 2, 3, 4};
        System.out.println(sol.thirdMax(nums));
        nums = new int[] {3, 1, 1};
        System.out.println(sol.thirdMax(nums));
        nums = new int[] {6, 3, 1, 1};
        System.out.println(sol.thirdMax(nums));
        nums = new int[] {1, 2, -2147483648};
        System.out.println(sol.thirdMax(nums));
        nums = new int[] {1, 2};
        System.out.println(sol.thirdMax(nums));
    }
}
