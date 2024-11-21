import cv2
import numpy as np
import os

# Load the image in grayscale
image = cv2.imread(os.path.expanduser('~/Downloads/robotics.png'), cv2.IMREAD_GRAYSCALE)

# Threshold the image to binary (black and white)
_, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

# Grid dimensions
grid_rows, grid_cols = 44,21
cell_height = binary_image.shape[0] // grid_rows
cell_width = binary_image.shape[1] // grid_cols

# Initialize 2D list for the grid
grid_array = []

# Loop through each grid cell
for row in range(grid_rows):
    grid_row = []
    for col in range(grid_cols):
        # Extract the cell from the binary image
        cell = binary_image[row * cell_height:(row + 1) * cell_height,
                            col * cell_width:(col + 1) * cell_width]
        
        # Determine if the cell is black (filled) or white (empty)
        cell_mean = np.mean(cell)
        grid_row.append('0' if cell_mean < 127 else '1')  # Black is 0, white is 1
    grid_array.append(grid_row)

# Output is a 21x44 list
print(grid_array)
