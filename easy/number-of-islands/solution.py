class Solution(object):
    def explore(self, grid, stack):
        while len(stack) > 0:
            row_col_pair = stack.pop()
            row = row_col_pair[0]
            col = row_col_pair[1]

            if row < 0 or col < 0 or row >= len(grid) or col >= len(grid[0]) or grid[row][col] != '1':
                continue
            else:
                grid[row][col] = '0'
            
            stack.append((row+1, col))
            stack.append((row-1, col))
            stack.append((row, col+1))
            stack.append((row, col-1))

    def numIslands(self, grid):
        """
        :type grid: List[List[str]]
        :rtype: int
        """
        count = 0
        for row in range(len(grid)):
            for col in range(len(grid[row])):
                if grid[row][col] == '1':
                    self.explore(grid,[(row,col)])
                    count += 1
        return count