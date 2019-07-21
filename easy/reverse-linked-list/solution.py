class Solution(object):
    def reverseList(self, head):
        if head == None:
            return head

        current = None

        while head.next != None:
            temp = head.next
            head.next = current
            current = head
            head = temp
        head.next = current
        return head
                