package com.leetcode.easy.buddy_strings;

public class main {
    public static void main(String[] args) {
        Solution sol = new Solution();
        String a = "abaa";
        String b = "abaa";
        System.out.println(sol.buddyStrings(a, b));
        b = "baaa";
        System.out.println(sol.buddyStrings(a, b));
        b = "aaab";
        System.out.println(sol.buddyStrings(a, b));
        a = "ab";
        b = "ab";
        System.out.println(sol.buddyStrings(a, b));
        a = "abab";
        b = "abab";
        System.out.println(sol.buddyStrings(a, b));

    }
}
