import numpy as np
import math
import trimesh

# def rodrigues(axis, theta, mat = None):
#     '''
#     Compute the rodrigues matrix using the given axis and theta
#
#     ## input
#     axis:
#         a 1-by-3 numpy array list
#     theta:
#         angle in degree
#     mat:
#         a 3-by-3 numpy array, rotation matrix if this was not given, users could get it from return
#
#     ## output
#     the mat
#

#     '''
#
#     if mat == None:
#         mat = np.eye(3,3)
#
#     support = "#include <math.h>"
#     code = """
#         double dth = static_cast<double>(theta);
#         dth = dth * 3.1415926/180.0;
#
#         double axis0 = static_cast<double>(axis[0]);
#         double axis1 = static_cast<double>(axis[1]);
#         double axis2 = static_cast<double>(axis[2]);
#         double x = sqrt(axis0*axis0 + axis1*axis1 + axis2*axis2);
#         double a = cos(dth / 2.0);
#         double b = -(axis0 / x) * sin(dth / 2.0);
#         double c = -(axis1 / x) * sin(dth / 2.0);
#         double d = -(axis2 / x) * sin(dth / 2.0);
#
#         mat[0] = a*a + b*b - c*c - d*d;
#         mat[3*1 + 0] = 2 * (b*c - a*d);
#         mat[3*2 + 0] = 2 * (b*d + a*c);
#
#         mat[1] = 2*(b*c+a*d);
#         mat[3*1 + 1] = a*a+c*c-b*b-d*d;
#         mat[3*2 + 1] = 2*(c*d-a*b);
#
#         mat[2] = 2*(b*d-a*c);
#         mat[3*1+ 2] = 2*(c*d+a*b);
#         mat[3*2 + 2] = a*a+d*d-b*b-c*c;
#     """
#
#     weave.inline(code, ['axis', 'theta', 'mat'], support_code = support, libraries = ['m'])
#
#     return mat

def rodrigues(axis, theta):
    """
    Compute the rodrigues matrix using the given axis and theta

    ## input
    axis:
        a 1-by-3 numpy array list
    theta:
        angle in degree
    mat:
        a 3-by-3 numpy array, rotation matrix if this was not given, users could get it from return

    ## output
    the mat


    """

    theta = theta*math.pi/180.0
    axis = np.array([axis[0], axis[1], axis[2]])
    axis = axis/math.sqrt(np.dot(axis, axis))
    if theta > 2*math.pi:
        theta = theta % 2*math.pi
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2.0*(bc+ad), 2.0*(bd-ac)],
                     [2.0*(bc-ad), aa+cc-bb-dd, 2.0*(cd+ab)],
                     [2.0*(bd+ac), 2.0*(cd-ab), aa+dd-bb-cc]])

def hat(a):
    """
    compute the skew symmetric maxtix that corresponds to a cross

    :param a: a 1-by-3 numpy array
    :return: a 3-by-3 numpy array

    """

    return np.array([[0, -a[2], a[1]],
                     [a[2], 0, -a[0]],
                     [-a[1], a[0], 0]])

def rotmatfacet(facetnormal, facetfirstpoint, facetsecondpoint):
    '''
    Compute the rotation matrix of a 3D facet using
    facetnormal and the first two points on the facet
    The function uses the concepts defined by Trimesh

    ## input:
    facetnormal:
        the normal of a facet
    facetfirstpoint:
        the first point of the first triangle on the facet
    facetsecondpoint:
        the second point of the first triangle on the facet

    ## output:
    mat:
        a (3,3) numpy matrix


    '''

    mat = np.eye(3,3)
    mat[2,:] = facetnormal
    mat[0,:] = facetsecondpoint-facetfirstpoint
    mat[0,:] = mat[0,:]/np.linalg.norm(mat[0,:])
    mat[1,:] = np.cross(mat[2,:],mat[0,:])

    return mat

def homoinverse(homomatrix4):
    """
    compute the inverse of a homogeneous transform

    :param homomatrix4:
    :return:


    """

    rotmat = homomatrix4[:3, :3]
    tranvec = homomatrix4[:3, 3]
    invmatrix4 = np.eye(4,4)
    invmatrix4[:3, :3] = np.transpose(rotmat)
    invmatrix4[:3, 3] = -np.dot(np.transpose(rotmat), tranvec)

    return invmatrix4

def transformmat4(matrix4, point):
    """
    do homotransform on point using matrix4

    :param matrix:
    :param point:
    :return:


    """

    point4 = np.array([point[0], point[1], point[2], 1])
    return np.dot(matrix4, point4)

def cvtRngPM180(armjnts):
    """
    change the range of armjnts to +-180

    :param armjnts a numpyarray of jnts

    """

    armjntsarray = np.array(armjnts)
    armjntsnew = armjntsarray.copy()
    for i in range(armjntsnew.shape[0]):
        if armjntsnew[i] < 0:
            armjntsnew[i] = armjntsnew[i] % -360
            if armjntsnew[i] < -180:
                armjntsnew[i] = armjntsnew[i] + 360
        if armjnts[i] > 0:
            armjntsnew[i] = armjntsnew[i] % 360
            if armjntsnew[i] > 180:
                armjntsnew[i] = armjntsnew[i] - 360

    return armjntsnew

def cvtRngPM360(armjnts):
    """
    change the range of armjnts to +-180

    :param armjnts a numpyarray of jnts

    """

    armjntsarray = np.array(armjnts)
    armjntsnew = armjntsarray.copy()
    for i in range(armjntsnew.shape[0]):
        if armjntsnew[i] < 0:
            armjntsnew[i] = armjntsnew[i] % -360
        if armjnts[i] > 0:
            armjntsnew[i] = armjntsnew[i] % 360

    return armjntsnew

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def radian_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def degree_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return math.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

def eulerAnglesToRotationMatrix(theta, format='degree'):
    """
    Calculates Rotation Matrix given euler angles.
    :param theta: 1-by-3 list [rx, ry, rz] angle in degree
    :return:


    """
    if format is 'degree':
        theta = theta * math.pi / 180.0

    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    # assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        # x = math.atan2(R[2, 1], R[2, 2])
        # y = math.atan2(-R[2, 0], sy)
        # z = math.atan2(R[1, 0], R[0, 0])

        x = math.atan(R[2, 1]/R[2, 2])
        y = math.atan(-R[2, 0]/ sy)
        z = math.atan(R[1, 0]/ R[0, 0])

    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    x = x * 180.0 / math.pi
    y = y * 180.0 / math.pi
    z = z * 180.0 / math.pi
    # x = x + 180
    # y = - y - 180
    z = z - 180

    if x > 180:
        x = x - 360
    if x < - 180:
        x = x + 360

    if y > 180:
        y = y - 360
    if y < - 180:
        y = y + 360

    if z > 180:
        z = z - 360
    if z < - 180:
        z = z + 360
    return np.array([x, y, z])