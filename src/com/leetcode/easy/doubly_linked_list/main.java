package com.leetcode.easy.doubly_linked_list;

public class main {
    public static void main(String[] args) {
        MyLinkedList list = new MyLinkedList();
//        int numToAdd = 3;
//        for (int i = 0; i < numToAdd; i++) {
//            list.addAtHead(i);
//        }
//        for (int i = 0; i < numToAdd; i++) {
//            list.addAtTail(i);
//        }

//        list.addAtIndex(0, 1);
//        list.addAtIndex(1, 2);
//
//        list.addAtIndex(0, 3);
//        list.addAtIndex(0, 4);
//
//        list.deleteAtIndex(list.getLength());
//        list.deleteAtIndex(list.getLength()-1);
//        list.deleteAtIndex();

//        list.addAtHead(0);
//        list.addAtIndex(1,9);
//        list.addAtIndex(1,5);
//        list.addAtTail(7);
//        list.addAtHead(1);
//        list.addAtIndex(5,8);
//        list.addAtIndex(5,2);
//        list.addAtIndex(3,0);
//        list.addAtTail(1);
//        list.addAtTail(0);
//        list.deleteAtIndex(6);

        list.addAtIndex(-1, 0);
        list.get(0);
        list.deleteAtIndex(-1);
    }
}
