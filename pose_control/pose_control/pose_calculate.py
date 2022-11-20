# initial angle is
# initail position
# [[ 0.          0.          0.          0.        ]
#  [ 0.01248707 -0.01248707  0.01248707 -0.01248707]
#  [-0.28935    -0.28935    -0.28935    -0.28935   ]]
import numpy as np

h = 0.2835 #0.2835     #default = 0.28935


w = 0.12958686166704494 + 0.09238727687245425    #0.222

l = 0.4689596080695508  -0.08900393382783338    # 0.38

b = 0.197 -0.095


l1=  0.211
l2 =  0.19
l3 = (w - b) /2



def get_AB(r,  p , y ):
    posi = np.mat([0.0, 0.0,  h]).T
    rpy = np.array([r, p, y]) * np.pi/180

    R, P, Y = rpy[0], rpy[1], rpy[2]

    rotx = np.mat([[ 1,       0,            0     ],
                  [ 0,       np.cos(R), -np.sin(R)],
                  [ 0,       np.sin(R),  np.cos(R)]])

    roty = np.mat([[ np.cos(P),  0,      -np.sin(P)],
                   [ 0,            1,       0      ],
                   [ np.sin(P),  0,       np.cos(P)]])

    rotz = np.mat([[ np.cos(Y), -np.sin(Y),  0     ],
                   [ np.sin(Y),  np.cos(Y),  0     ],
                   [ 0,            0,        1     ]])

    rot_mat = rotx * roty * rotz

    body_stru = np.mat([
        [l/2,    b/2,   0],
        [l/2,   -b/2,   0],
        [-l/2,   b/2,   0],
        [-l/2,  -b/2,   0] ]).T

    footint_struc = np.mat([[   l / 2,  w / 2,  0],
                              [ l / 2, -w/ 2,  0],
                              [-l / 2,  w / 2,  0],
                              [-l / 2, -w / 2,  0]]).T

    AB = np.mat( np.zeros((3,4)) )
    for i in range(4):
        AB[:,i] = -posi - rot_mat * body_stru[:,i] + footint_struc[:,i]
    return AB



def inverkinematic(x,y,z):
        cos_theta3_rad = (x**2 +z**2 - (l1**2+l2**2) )/ (2 *l1 *l2)
        theta3_rad = np.arccos(cos_theta3_rad)

        cos_alpha_rad = ( l1**2+ x**2 +z**2- l2**2 ) /  ( 2*l1* np.sqrt(x**2+z**2) )
        alpha_rad = np.arccos(cos_alpha_rad)
        theta2_rad = np.pi/2 - alpha_rad - np.arctan2(z , x)
        #
        # gamma = np.abs(z)/(y**2, z**2)
        # dis = np.sqrt( (z**2+y**2) )
        # length = np.sqrt( (dis**2-l3**2) )
        # gama1_rad = np.arctan2(l3,length)
        # gama2_rad = np.arctan2(np.abs(y),z)
        # theta3_rad = gama1_rad-gama2_rad
        # gamma1 = np.arccos(np.abs(y)/np.sqrt( y**2 + z**2) )
        # gamma2 = np.arccos (l3/np.sqrt(y**2+z**2))
        #
        # if y>=0:
        #     theta3_rad = gamma1-gamma2
        #
        # else:
        #     theta1_rad = np.pi - gamma1-gamma2
        theta1_rad =  0

        gamm1 = np.arctan(y/z)
        gamm2 = np.arccos( l3/np.sqrt(z**2+y**2) )

        theta1_rad = np.pi/2- gamm1+gamm2


        return theta1_rad, theta2_rad,theta3_rad

def IK_Leg13(x,y,z):

      D =  (x**2+y**2-l1**2+z**2 -l2**2-l3**2)/(2*l2*l3)
      
      theta3_rad = np.arctan2(-np.sqrt(1-D**2),D)
      
      theta1_rad =  -np.arctan2(z,y)-np.arctan2(np.sqrt(y**2+z**2-l1**2),-l1)

      theta2_rad =  np.arctan2(-x,np.sqrt( y**2 +z**2 -l1**2  ))-np.arctan2(l3*np.sin(theta3_rad), l2+l3*np.cos(theta3_rad))

      return -theta1_rad,theta2_rad,theta3_rad


if __name__ == '__main__':
    # matrix_ABs=get_AB(0,0,0).T
    # print(matrix_ABs)
    # x1, y1 ,z1 = matrix_ABs[0,0],matrix_ABs[0,1],matrix_ABs[0,2]
    # print(x1,y1,z1)

    # matrix_ABs=get_AB(0,0,0)
    # print(matrix_ABs)
    theta1, theta2 ,theta3= inverkinematic(0, 0.012 ,0.2835 )
    print("theta1==={}, theta2==={},theta3=={}".format(theta1*180/np.pi, theta2*180/np.pi ,theta3*180/np.pi))