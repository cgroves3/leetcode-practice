package com.leetcode.easy.excel_column_title;

import java.util.ArrayList;
import java.util.List;

public class Solution {
    public String convertToTitle(int n) {
        if (n <= 0) {
            return "";
        }
        int number = n;
        int lengthOfAlphabet = 26;
        StringBuilder builder = new StringBuilder();
        if (number > lengthOfAlphabet) {
            int exp = (int)Math.floor(Math.log(number) / Math.log(lengthOfAlphabet));
            int[] digits = new int[exp+1];
            int i = 0;
            while (number > lengthOfAlphabet) {
                //Compute the smallest integer multiplier of number and 26.
                int digit = (int)(Math.floor(number / Math.pow(lengthOfAlphabet, exp)));
                digits[i] = digit;
                number = (int)(number - digit * Math.pow(lengthOfAlphabet, exp));
                exp--;
                i++;
            }
            digits[digits.length-1] = number;
            for (i = 0; i < digits.length; i++) {
                if (digits[i] == 0 && i > 0) {
                    digits[i-1] -= 1;
                    digits[i] = 26;
                }
                if (i > 0 && digits[i-1] > 0) {
                    builder.append((char) ((digits[i-1]) - 1 + 'A'));
                }
            }
            builder.append((char) ((digits[digits.length-1]) - 1 + 'A'));
        }
        else {
            builder.append((char) ((number) - 1 + 'A'));
        }
        return builder.toString();
    }
}
