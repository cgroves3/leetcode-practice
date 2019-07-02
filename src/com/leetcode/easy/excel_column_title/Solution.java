package com.leetcode.easy.excel_column_title;

import java.util.ArrayList;
import java.util.List;

public class Solution {
    public String convertToTitle(int n) {
        if (n <= 0) {
            return "";
        }
        List<Integer> digits = new ArrayList<Integer>();
        int number = n;
        int lengthOfAlphabet = 26;
        int count = 0;
        StringBuilder builder = new StringBuilder();
        int remainder = number % lengthOfAlphabet;
        if (number > lengthOfAlphabet) {
            digits.add(count);
            while ((number / lengthOfAlphabet) > 0) {
//            number -= lengthOfAlphabet;
                number = number / lengthOfAlphabet;
//            Math.log(number) / Math.log(lengthOfAlphabet);
                digits.set(digits.size() - 1, digits.get(digits.size() - 1) + 1);
                if (digits.get(digits.size() - 1) > lengthOfAlphabet) {
                    digits.set(digits.size() - 1, 1);
                    digits.add(1);
                }
            }
        }
        for (int i = digits.size()-1; i >= 0; i--) {
            if (digits.get(i) > 0)
                builder.append((char)(digits.get(i) - 1 + 'A'));
        }

        if (remainder > 0) {
            builder.append((char) ((remainder) - 1 + 'A'));
        }
        else {
            builder.append('Z');
        }
        return builder.toString();
    }
}
