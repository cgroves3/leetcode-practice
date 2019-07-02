package com.leetcode.easy.excel_column_title;

public class main {
    public static void main(String[] args) {
        Solution solution = new Solution();
        System.out.println(solution.convertToTitle(1));
        System.out.println(solution.convertToTitle(28));
        System.out.println(solution.convertToTitle(52));
        System.out.println(solution.convertToTitle(53));
//        System.out.println(solution.convertToTitle(626));
//        System.out.println(solution.convertToTitle(701));
        System.out.println(solution.convertToTitle((int)Math.pow(26,3)));
    }
}
