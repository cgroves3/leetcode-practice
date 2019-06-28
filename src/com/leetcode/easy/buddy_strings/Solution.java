package com.leetcode.easy.buddy_strings;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class Solution {
    public boolean buddyStrings(String A, String B) {
        if (A == null || A.length() == 0 || B == null || B.length() == 0) {
            return false;
        }
        int[] count = new int[26];

        if (A.length() != B.length())
            return false;

        HashMap<Character, Integer> a_map = new HashMap<Character, Integer>();
        HashMap<Character, Integer> b_map = new HashMap<Character, Integer>();

        int charDiffCount = 0;
        int charDiffLimit = 2;

        boolean hasRepeatingChar = false;

        // Added character counts to maps
        for (int i = 0; i < A.length(); i++) {
            Character a_i = A.charAt(i);
            Character b_i = B.charAt(i);

            if (a_i != b_i) {
                removeCharacter(a_map, a_i);
                addCharacter(a_map, b_i);
                removeCharacter(b_map, b_i);
                addCharacter(b_map, a_i);
                charDiffCount++;
            }
            else {
                count[a_i - 'a']++;
                hasRepeatingChar = count[a_i - 'a'] > 1;
            }
        }

        // Check if all counts are equal to 0, meaning each letter has the same count
        Iterator<Map.Entry<Character, Integer>> iterA = a_map.entrySet().iterator();
        Iterator<Map.Entry<Character, Integer>> iterB = b_map.entrySet().iterator();
        while (iterA.hasNext() && iterB.hasNext()) {
            if (iterA.next().getValue() != 0 && iterB.next().getValue() != 0) {
                return false;
            }
        }

        // The char difference count should be 2 or 0 meaning one swap or there is at least one repeating char
        return charDiffCount == charDiffLimit || hasRepeatingChar;

    }

    private void addCharacter(HashMap<Character, Integer> a_map, Character a_i) {
        if (!a_map.containsKey(a_i)) {
            a_map.put(a_i, 1);
        }
        else {
            a_map.put(a_i, a_map.get(a_i)+1);
        }
    }

    private void removeCharacter(HashMap<Character, Integer> a_map, Character a_i) {
        if (!a_map.containsKey(a_i)) {
            a_map.put(a_i, -1);
        }
        else {
            a_map.put(a_i, a_map.get(a_i)-1);
        }
    }
}
