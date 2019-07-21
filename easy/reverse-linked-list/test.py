import unittest
from solution import Solution

class ListNode(object):
    def __init__(self, x):
        self.val = x
        self.next = None

class TestSolution(unittest.TestCase):
    
    def testCase(self):
        node = ListNode(1)
        head = node
        node.next = ListNode(2)
        node = node.next
        node.next = ListNode(3)
        node = node.next
        node.next = ListNode(4)

        rev = ListNode(4)
        revHead = rev
        rev.next = ListNode(3)
        rev = rev.next
        rev.next = ListNode(2)
        rev = rev.next
        rev.next = ListNode(1)
        solution = Solution()
        revNode = solution.reverseList(head)

        
        while (revHead.next != None and revNode.next != None):
            self.assertEqual(revHead.val, revNode.val)
            revHead = revHead.next
            revNode = revNode.next
            

if __name__ == "__main__":
    unittest.main()