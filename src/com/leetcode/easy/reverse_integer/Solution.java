package com.leetcode.easy.reverse_integer;

public class Solution {
    public int reverse(int x) {
        boolean numIsNegative = x < 0;
        if (numIsNegative) {
            x = -x;
        }
        Long reverse = Long.valueOf(0);
        //While the number is greater than 10, perform x % 10 to get each digit, multiply that the existing number by 10
        //to shift decimal places and add the result to 10 times the digit.
        while (x >= 10) {
            int digit = x % 10;
            reverse = reverse * 10 + digit * 10;
            x = x/10;
        }
        //Add the remaining ones values
        reverse += x;
        if (numIsNegative)
            reverse = -reverse;

        if (reverse > Integer.MAX_VALUE)
            return 0;

        return reverse.intValue();
    }
}