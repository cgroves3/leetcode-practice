package com.amazon;

import java.util.Comparator;
import java.util.List;

public class Sort {
    SortOrder sortOrder = new SortOrder();

    private class SortOrder implements Comparator<String> {

        private boolean isNonPrime(String order) {
            int firstSpaceInd = order.indexOf(' ');
            for (int i = firstSpaceInd; i < order.length(); i++) {
                try {
                    Integer.parseInt(Character.toString(order.charAt(i)));
                    return true;
                }
                catch (NumberFormatException ex) {
                }
            }
            return false;
        }

        @Override
        public int compare(String s, String t1) {
            //If they are both prime orders compare, lexographically
            if (!isNonPrime(s) && !isNonPrime(t1)) {
                return s.compareTo(t1);
            }
            else if (!isNonPrime(s) && isNonPrime(t1)) {
                return -1;
            }
            else if (isNonPrime(s) && !isNonPrime(t1)) {
                return 1;
            }
            else {
                return 0;
            }
        }
    }

    public List<String> sort(int numOfOrders, List<String> orders){
        orders.sort(sortOrder);
        return orders;
    }
}
