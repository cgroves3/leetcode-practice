package com.leetcode.easy.reverse_integer;

public class main {
    public static void main(String[] args) {
        Solution sol = new Solution();
        int x = -2147483648;
        System.out.println(sol.reverse(x));
        x = -1563847412;
        System.out.println(sol.reverse(x));
    }
}
