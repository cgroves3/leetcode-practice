package com.amazon;
import java.util.ArrayList;
import java.util.List;

public class main {
    public static void main(String[] args) {
        List<String> list = new ArrayList<String>();
        Sort sort = new Sort();
        list.add("m4as asf dfasdf");
        list.add("m4as sdf df6asdf");
        list.add("m4as sdf dfa7sdf");
        list.add("m4as sdf dfas8df");
        list.add("m4as sdf dfasdf");
        list.add("m4as adf dfasdf");
        System.out.println(sort.sort(list.size(), list));
    }
}
