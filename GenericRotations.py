from sympy import symbols, Matrix, cos, sin, pi, atan2, acos, simplify, asin
import sys
import numpy as np

#PI symbol
PI = pi

def printAngles(angles, degrees, evalf):
    if evalf:
        if degrees:
            print("alpha = %f °" % (angles[0] * 180 / PI).evalf())
            print("beta = %f °" % (angles[1] * 180 / PI).evalf())
            print("gamma = %f °" % (angles[2] * 180 / PI).evalf())
        else:
            #print the angles in terms of PI and like fraction
            print("alpha = %f PI" % (angles[0] / PI).evalf())
            print("beta = %f PI" % (angles[1] / PI).evalf())
            print("gamma = %f PI" % (angles[2] / PI).evalf())
    else:
        print("alpha = %s" % angles[0])
        print("beta = %s" % angles[1])
        print("gamma = %s" % angles[2])
    

def print_matlab(mat):
    print("[")
    for row in mat.tolist():

        for i, elem in enumerate(row):
            if i == len(row) - 1:
                print(elem, end=";")
            else:
                print(elem, end=",")
        print()
    print("]")

def generate_rotation_matrix(order, angles, fixed):
    """
    Generate rotation matrix based on symbolic angles and specified order.
    """
    R = Matrix.eye(3)  # Initialize the rotation matrix as 3x3 identity matrix

    if fixed:
        order = order[::-1]
        angles = angles[::-1]

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
        R = simplify(R)
    return R

# Define symbolic angles
alpha, beta, gamma = symbols('alpha beta gamma')


# Example usage:
if __name__ == "__main__":
    # Define rotation matrix R (replace with actual matrix values)
    #R = generate_rotation_matrix(order="ZYX", angles=[alpha, beta, gamma], fixed=False)
    R = generate_rotation_matrix(order="ZYX", angles=[-PI/2, -PI/4, PI/4], fixed=True)
    print("\nRotation matrix:")
    print_matlab(R)

    sys.exit(0)