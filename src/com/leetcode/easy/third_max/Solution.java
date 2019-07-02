package com.leetcode.easy.third_max;

import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;

public class Solution {
//    public int thirdMax(int[] nums) {
//        int limit = 3;
//        Queue<Integer> queue = new PriorityQueue<>(limit);
//        //int[] max_vals = new int[3];
//        Integer max = Integer.MIN_VALUE;
//
//        //If you've found a new max or the list has space, remove the smallest and adds the current max
//        for (int num : nums) {
//            if (queue.size() < limit && !queue.contains(num)) {
//                queue.add(num);
//            }
//            if (num > queue.peek()) {
//                while (num > queue.peek()) {
//                    Integer lowest = queue.
//                }
//                max = num;
//                if (queue.size() == limit) {
//                    queue.remove();
//                }
//                queue.add(max);
//            }
//        }
//        queue.
//    }
    public int thirdMax(int[] nums) {

        Long max = Long.valueOf(Integer.MIN_VALUE)-1;
        Long second = Long.valueOf(Integer.MIN_VALUE) - 1;
        Long third = Long.valueOf(Integer.MIN_VALUE) - 1;

        //If you've found a new max or the list has space, remove the smallest and adds the current max
        for (int num : nums) {
            if (num > max) {
                third = Long.valueOf(second);
                second = Long.valueOf(max);
                max = Long.valueOf(num);
            }
            if (num < max && num > second) {
                third = Long.valueOf(second);
                second = Long.valueOf(num);
            }
            if (num < second && num > third) {
                third = Long.valueOf(num);
            }
        }
        if (third != Long.valueOf(Integer.MIN_VALUE) - 1)
            return third.intValue();
        else
            return max.intValue();
    }
}
