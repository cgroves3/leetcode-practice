public class MyLinkedList {
    private Node head;
    private Node tail;
    private int length;

    /**
     * Initialize your data structure here.
     */
    public MyLinkedList() {
    }

    /**
     * Get the value of the index-th node in the linked list. If the index is invalid, return -1.
     */
    public int get(int index) {

        //Index bounds checking and size checking
        if (index < 0 || index >= length || length == 0)
            return -1;

        Node node = getNode(index);

        return node.getNumber();
    }

    /**
     * Add a node of value val before the first element of the linked list. After the insertion, the new node will be the first node of the linked list.
     */
    public void addAtHead(int val) {
        Node nodeToAdd = new Node(val);
        if (head == null) {
            head = nodeToAdd;
            link(head, tail);
        }
        else {
            if (tail == null) {
                tail = head;
                head = nodeToAdd;
                link(head, tail);
            }
            else {
                link(nodeToAdd, head);
                head = nodeToAdd;
            }
        }
        length++;
    }

    /**
     * Append a node of value val to the last element of the linked list.
     */
    public void addAtTail(int val) {
        Node nodeToAdd = new Node(val);
        if (tail == null) {
            tail = nodeToAdd;
            link(head, tail);
        }
        else {
            if (head == null) {
                head = tail;
                tail = nodeToAdd;
                link(head, tail);
            }
            else {
                link(tail, nodeToAdd);
                tail = nodeToAdd;
            }
        }
        length++;
    }

    public void addAtIndex(int index, int val) {
        // Index bounds checking
        if (index < -1 || index > length)
            return;

        Node nodeToAdd = new Node(val);

        // If the list is empty, add the node as the head
        if (length == 0) {
            head = nodeToAdd;
        }
        else {
            // Get the node at the index.
            Node node = getNode(index);
            //If appending and the tail is null, set a new tail and link the head to the tail. Otherwise link the new
            //tail and update the tail reference.
            if (index == length) {
                if (node == null) {
                    tail = nodeToAdd;
                    link(head, tail);
                }
                else {
                    link(tail, nodeToAdd);
                    tail = nodeToAdd;
                }
            }
            //If there is already a head, and we're inserting at the head, update the head to the new head and
            // set the tail to the old head.
            else {
                if (node == head) {
                    head = nodeToAdd;
                    tail = node;
                }
                // If we already have a head and we aren't inserting at the head, link the previous node to the new node.
                else {
                    link(node.getPrevious(), nodeToAdd);
                }
                link(nodeToAdd, node);
            }
        }

        length++;
    }

    /**
     * Delete the index-th node in the linked list, if the index is valid.
     */
    public void deleteAtIndex(int index) {
        if (index < 0 || index >= length || length == 0)
            return;

        Node node = getNode(index);
        if (node == head) {
            head = node.getNext();
        }
        else if (node == tail) {
            tail = node.getPrevious();
        }
        else {
            link(node.getPrevious(), node.getNext());
        }
        node = null;
        length--;
    }

    public int getLength() {
        return length;
    }

    private Node getNode(int index) {
        if (index < 0 || index > length || length == 0)
            return null;

        Node node;

        //If closer to the head, start at the head, otherwise start at the tail
        if (index <= length - index) {
            node = head;
            if (node != null) {
                int i = 0;
                while (node.getNext() != null && i < index) {
                    node = node.getNext();
                    i++;
                }
            }
        }
        else {
            node = tail;
            if (node != null) {
                int i = length - 1;
                while (node.getPrevious() != null && i > index) {
                    node = node.getPrevious();
                    i--;
                }
            }
        }
        return node;
    }

    private void link(Node head, Node tail) {
        if (head != null) {
            head.setNext(tail);
        }
        if (tail != null) {
            tail.setPrevious(head);
        }
    }
}