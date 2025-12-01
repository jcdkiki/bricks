import sys

A = []
x = []
b = []

for line in sys.stdin.readlines():
    arr = list(map(float, line.split()))
    A.append(arr[0:-2])
    x.append(arr[-2])
    b.append(arr[-1])

Ab = []
for i in range(len(A)):
    Ab.append(A[i] + [b[i]])

def gaussian_elimination(matrix):
    rows = len(matrix)
    cols = len(matrix[0])
    
    mat = [row[:] for row in matrix]
    
    rank = 0
    pivot_row = 0
    pivot_col = 0
    
    while pivot_row < rows and pivot_col < cols:
        max_row = pivot_row
        for row in range(pivot_row + 1, rows):
            if abs(mat[row][pivot_col]) > abs(mat[max_row][pivot_col]):
                max_row = row
        
        if abs(mat[max_row][pivot_col]) > 1e-10:
            if max_row != pivot_row:
                mat[pivot_row], mat[max_row] = mat[max_row], mat[pivot_row]
            
            for row in range(pivot_row + 1, rows):
                factor = mat[row][pivot_col] / mat[pivot_row][pivot_col]
                for col in range(pivot_col, cols):
                    mat[row][col] -= factor * mat[pivot_row][col]
            
            rank += 1
            pivot_row += 1
        
        pivot_col += 1
    
    return rank

print(f"{gaussian_elimination(Ab)}/{len(A)}")