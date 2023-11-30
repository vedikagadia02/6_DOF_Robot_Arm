from sympy import symbols, cos, sin, Matrix, pi, sstr

def getTransformationMatrix(theta, d, a, alpha):
    
    R_z_theta = Matrix([[cos(theta), -sin(theta), 0, 0],
                 [sin(theta), cos(theta), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

    T_z_d = Matrix([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, d],
                 [0, 0, 0, 1]])

    T_x_a = Matrix([[1, 0, 0, a],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

    R_x_alpha = Matrix([[1, 0, 0, 0],
                 [0, cos(alpha), -sin(alpha), 0],
                 [0, sin(alpha), cos(alpha), 0],
                 [0, 0, 0, 1]])

    T = R_z_theta * T_z_d * T_x_a * R_x_alpha
    return T

theta, d, a, alpha = symbols('theta, d, a, alpha')
transformation_matrix = getTransformationMatrix(theta, d, a, alpha)
print("Homogeneous Transformation Matrix: Formula Verification")
print(transformation_matrix)

theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
a1, a2, a3, a4, a5, a6 = symbols('a1 a2 a3 a4 a5 a6')
d1, d2, d3, d4, d5, d6 = symbols('d1 d2 d3 d4 d5 d6')

alpha1 = -pi/2
alpha2 = 0
alpha3 = -pi/2
alpha4 = -pi/2
alpha5 = pi/2
alpha6 = 0

A1 = getTransformationMatrix(theta1, d1, a1, alpha1)
A2 = getTransformationMatrix(theta2, d2, a2, alpha2)
A3 = getTransformationMatrix(theta3, d3, a3, alpha3)
A4 = getTransformationMatrix(theta4, d4, a4, alpha4)
A5 = getTransformationMatrix(theta5, d5, a5, alpha5)
A6 = getTransformationMatrix(theta6, d6, a6, alpha6)

T = A1 * A2 * A3 * A4 * A5 * A6
print("\nTransformation Matrix 0 n [2(c)]")
print(T)

a1 = 0.5
a2 = 2
a3 = -0.6
a4 = 0
a5 = 0
a6 = 0

d1 = 2.9
d2 = 0
d3 = 0
d4 = 0.875
d5 = 0
d6 = 1.3

# Part A (all joint angles zero)
theta1 = theta2 = theta3 = theta4 = theta5 = theta6 = 0
theta2 += -pi/2
A1 = getTransformationMatrix(theta1, d1, a1, alpha1)
A2 = getTransformationMatrix(theta2, d2, a2, alpha2)
A3 = getTransformationMatrix(theta3, d3, a3, alpha3)
A4 = getTransformationMatrix(theta4, d4, a4, alpha4)
A5 = getTransformationMatrix(theta5, d5, a5, alpha5)
A6 = getTransformationMatrix(theta6, d6, a6, alpha6)
T = A1 * A2 * A3 * A4 * A5 * A6
print("\nTransformation Matrix for Configuration 1")
print(sstr(T))


# Part A (all joint angles zero)
theta1 = theta2 = theta3 = theta4 = theta5 = theta6 = 0
theta2 += -pi/2
theta3 += pi/2
theta5+= -pi/2
A1 = getTransformationMatrix(theta1, d1, a1, alpha1)
A2 = getTransformationMatrix(theta2, d2, a2, alpha2)
A3 = getTransformationMatrix(theta3, d3, a3, alpha3)
A4 = getTransformationMatrix(theta4, d4, a4, alpha4)
A5 = getTransformationMatrix(theta5, d5, a5, alpha5)
A6 = getTransformationMatrix(theta6, d6, a6, alpha6)
T = A1 * A2 * A3 * A4 * A5 * A6
print("\nTransformation Matrix for Configuration 2")
print(sstr(T))

# Part A (all joint angles zero)
theta1 = theta2 = theta3 = theta4 = theta5 = theta6 = 0
theta1+=-pi/2
theta6+=pi/2
A1 = getTransformationMatrix(theta1, d1, a1, alpha1)
A2 = getTransformationMatrix(theta2, d2, a2, alpha2)
A3 = getTransformationMatrix(theta3, d3, a3, alpha3)
A4 = getTransformationMatrix(theta4, d4, a4, alpha4)
A5 = getTransformationMatrix(theta5, d5, a5, alpha5)
A6 = getTransformationMatrix(theta6, d6, a6, alpha6)
T = A1 * A2 * A3 * A4 * A5 * A6
print("\nTransformation Matrix for Configuration 3")
print(sstr(T))