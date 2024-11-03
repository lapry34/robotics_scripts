from sympy import symbols, cos, sin, Matrix, pi
import sys
PI = pi

# Define symbolic angles
alpha, beta, gamma = symbols('alpha beta gamma')

if __name__ == "__main__":
    # Set fixed order and symbolic angles
    fixed = True
    order = "ZYX"

    # if you want to define angles directly, uncomment the following lines
    #alpha   =   PI/3    #first angle in order
    #beta    =   PI/3    #second angle in order
    #gamma   =   -PI/2   #third angle in order

    angles = [alpha, beta, gamma]  # Use symbolic angles
    R = Matrix.eye(3)  # Initialize the rotation matrix as a 3x3 identity matrix

    if fixed:
        order = order[::-1]
        angles = angles[::-1]

    # Apply rotation matrices based on order and symbolic angles
    for axis, angle in zip(order, angles):
        if axis == "X":
            R_x = Matrix([
                [1, 0, 0],
                [0, cos(angle), -sin(angle)],
                [0, sin(angle), cos(angle)]
            ])
            R *= R_x

        elif axis == "Y":
            R_y = Matrix([
                [cos(angle), 0, sin(angle)],
                [0, 1, 0],
                [-sin(angle), 0, cos(angle)]
            ])
            R *= R_y

        elif axis == "Z":
            R_z = Matrix([
                [cos(angle), -sin(angle), 0],
                [sin(angle), cos(angle), 0],
                [0, 0, 1]
            ])
            R *= R_z

    # Display the rotation matrix R in symbolic form
    print("R = [")
    for row in R.tolist():
        for i, col in enumerate(row):
            print(f"{col}", end=", " if i < 2 else ";\n")
    print("];")
    sys.exit(0)