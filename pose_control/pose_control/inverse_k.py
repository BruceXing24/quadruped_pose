import numpy as np

# initial angle is
# initail position
# [[ 0.          0.          0.          0.        ]
#  [ 0.01248707 -0.01248707  0.01248707 -0.01248707]
#  [-0.28935    -0.28935    -0.28935    -0.28935   ]]
# 45 degree initial coordinate(0, )


b = 0.197 -0.095
w = 0.12958686166704494 + 0.09238727687245425    #0.222
l2=  0.211
l3 =  0.19
l1 = (w - b) /2

def IK_Leg13(x,y,z):

      D =  (x**2+y**2-l1**2+z**2 -l2**2-l3**2)/(2*l2*l3)
      
      theta3_rad = np.arctan2(np.sqrt(1-D**2),D)
      
      theta1_rad =  -np.arctan2(z,y)-np.arctan2(np.sqrt(y**2+z**2-l1**2),-l1)

      theta2_rad =  np.arctan2(x,np.sqrt( y**2 +z**2 -l1**2  ))-np.arctan2(l3*np.sin(theta3_rad), l2+l3*np.cos(theta3_rad))

      return theta1_rad,theta2_rad,theta3_rad


def IK_Leg24(x,y,z):

      D =  (x**2+y**2-l1**2+z**2 -l2**2-l3**2)/(2*l2*l3)
      
      theta3_rad = -np.arctan2(-np.sqrt(1-D**2),D)
      
      theta1_rad =  -np.arctan2(z,y)-np.arctan2(np.sqrt(y**2+z**2-l1**2),-l1)

      theta2_rad =  np.arctan2(-x,np.sqrt( y**2 +z**2 -l1**2  ))-np.arctan2(l3*np.sin(theta3_rad), l2+l3*np.cos(theta3_rad))

      return -theta1_rad,-theta2_rad,-theta3_rad


def inverkinematic(x,y,z):
        cos_theta2_rad = (x**2 +z**2 - (l2**2+l2**2) )/ (2 *l2 *l3)
        theta2_rad = np.arccos(cos_theta2_rad)

        cos_alpha_rad = ( l2**2+ x**2 +z**2- l2**2 ) /  ( 2*l2* np.sqrt(x**2+z**2) )
        alpha_rad = np.arccos(cos_alpha_rad)
        theta1_rad = np.pi/2 - alpha_rad - np.arctan2(z , x)
        gamma1 = np.arccos(np.abs(y)/np.sqrt( y**2 + z**2) )
        gamma2 = np.arccos (l1/np.sqrt(y**2+z**2))

        if y>=0:
            theta3_rad = gamma1-gamma2
        else:
            theta3_rad = np.pi - gamma1-gamma2
        return theta3_rad, theta1_rad, theta2_rad




def rad2degree(theta):
      return theta/np.pi*180

if __name__ == '__main__':
      

      theta1_rad,theta2_rad,theta3_rad = IK_Leg13(0, 0.012,-0.35)
      theta1=rad2degree(theta1_rad)
      theta2=rad2degree(theta2_rad)
      theta3=rad2degree(theta3_rad)

      print("theta1=={},theta2=={},theta3=={}".format(theta1,theta2,theta3))

      theta4_rad,theta5_rad,theta6_rad = inverkinematic(0, 0.012,0.35)
      theta4=rad2degree(theta4_rad)
      theta5=rad2degree(theta5_rad)
      theta6=rad2degree(theta6_rad)

      print("theta4=={},theta5=={},theta6=={}".format(theta4,theta5,theta6))