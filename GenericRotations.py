import numpy as np


PI = np.pi

if __name__ == "__main__":
    
    fixed = True
    order = "ZYX"

    alpha   =   PI/3    #first angle in order
    beta    =   PI/3    #second angle in order
    gamma   =   -PI/2   #third angle in order
    
    angles = [alpha, beta, gamma]
    R = np.eye(3)

    if fixed:
        order = order[::-1]
        angles = angles[::-1]
    
    for c, angle in zip(order, angles):
        if c == "X":            
            R_x = np.array([[1, 0, 0],
                    [0, np.cos(angle), -np.sin(angle)],
                    [0, np.sin(angle), np.cos(angle)]])
            R = np.dot(R, R_x)

        elif c == "Y":
            R_y = np.array([[np.cos(angle), 0, np.sin(angle)],
                    [0, 1, 0],
                    [-np.sin(angle), 0, np.cos(angle)]])
            R = np.dot(R, R_y)

        elif c == "Z":
            R_z = np.array([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1]])
            R = np.dot(R, R_z)

#print R matrix in console with decimal values in matlab style
    print("R = [")
    for row in R:
        for i, col in enumerate(row):
            print("{:.5f}".format(col), end=", " if i < 2 else ";\n")
    print("];")
