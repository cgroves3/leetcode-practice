public class Node {
    private Node next;
    private Node prev;
    private int number;

    public Node(int Number) {
        number = Number;
    }

    public Node getNext() {
        return next;
    }

    public Node getPrevious() {
        return prev;
    }

    public int getNumber() {
        return number;
    }

    public void setNext(Node node) {
        this.next = node;
    }

    public void setPrevious(Node node) {
        this.prev = node;
    }
}