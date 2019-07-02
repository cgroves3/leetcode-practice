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
        if (number > lengthOfAlphabet)
            digits.add(count);
        while (number > lengthOfAlphabet) {
            number -= lengthOfAlphabet;
            digits.set(digits.size()-1, digits.get(digits.size()-1) + 1);
            if (digits.get(digits.size()-1) > lengthOfAlphabet) {
                digits.set(digits.size()-1, 1);
                digits.add(count);
            }
        }
        for (int i = digits.size()-1; i >= 0; i--) {
            if (digits.get(i) > 0)
                builder.append((char)(digits.get(i) - 1 + 'A'));
        }
        builder.append((char)(number -1 + 'A'));
        return builder.toString();
    }
}
