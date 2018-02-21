
import math

horLinkLength = 0.15
verLinkLength = 0.135
F32_PI = 3.1415927
dutyCycle2DegFactor = 17.0


def rad2deg(rad):
  deg = (rad * 180) / math.pi
  return deg
  
def deg2rad(deg):
  rad = (deg * math.pi) / 180.0
  return rad

def deg2dc(angleDeg):
  dc = (180-angleDeg)/dutyCycle2DegFactor
  return dc
  
def dc2deg(dc):
  angleDeg = 180 - (dc * dutyCycle2DegFactor)
  return angleDeg
  
def calculateInverseKinematics(xPos, yPos):
    c = math.sqrt(xPos*xPos + yPos*yPos)
    gamma = math.asin(yPos/c)
    alpha = math.acos((verLinkLength*verLinkLength + c*c - horLinkLength*horLinkLength)/(2*verLinkLength*c))
    
    joint1Angle = rad2deg(gamma + alpha)
    
    beta = math.acos((verLinkLength*verLinkLength + horLinkLength*horLinkLength - c*c)/(2*verLinkLength*horLinkLength))
    epsilon = 90 - rad2deg(gamma + alpha)
    delta = 90 - epsilon
    omega = 180 - delta - rad2deg(beta)
    
    joint2angle = omega
    
    return joint1Angle,joint2angle

def calculateForwardKinematics(joint1Angle, joint2Angle):
  
  X = verLinkLength * math.cos(deg2rad(joint1Angle)) + horLinkLength * math.cos(deg2rad(joint2Angle))
  Y = verLinkLength * math.sin(deg2rad(joint1Angle)) - horLinkLength * math.sin(deg2rad(joint2Angle))
  
  return X,Y


while 1:
    x = float(raw_input("x in cm: "))
    y = float(raw_input("y in cm: "))
    
    print "x: %.3f --- y: %.3f" % (x,y)
    
    if math.sqrt(x*x + y*y)/100.0 > horLinkLength + verLinkLength:
        print "Invalid point, sqrt(x^2 + y^2) > arm length!"
        continue

    j1,j2 = calculateInverseKinematics(x/100.0,y/100.0)

    servo3NewPosRef = deg2dc(j1)
    servo2NewPosRef = deg2dc(j2)
    
    print "J1 angle: %.2f --- servo DC: %.2f" % (j1, servo3NewPosRef)
    print "J2 angle: %.2f --- servo DC: %.2f" % (j2, servo2NewPosRef)
    
    # j1 = dc2deg(servo3NewPosRef)
    # j2 = dc2deg(servo2NewPosRef)
    
    Xnew, Ynew = calculateForwardKinematics(j1,j2)
    
    print "Backward calculated X: %.3f cm --- Y: %.3f cm" % (Xnew*100, Ynew*100)
    
    print 20*"-"











'''
JointAngles calculateInverseKinematics(float xPos, float yPos){
  
  static volatile float x_temp;
  static volatile float y_temp;
  static float c;
  static float gamma;
  static float alpha;
  static float beta;
  static float epsilon;
  static float delta;
  static float omega;
  
  x_temp = xPos;
  y_temp = yPos;
  
  c = sqrtf(xPos*xPos + yPos*yPos); //c^2=a^2+b^2
  gamma = asinf(yPos/c);
  alpha = acosf((verLinkLength*verLinkLength + c*c - horLinkLength*horLinkLength)/(2*verLinkLength*c));
  
  calculatedAngles.joint1Angle = rad2deg(gamma + alpha);
  
  beta = acosf((verLinkLength*verLinkLength + horLinkLength*horLinkLength - c*c)/(2*verLinkLength*c));
  epsilon = 90 - rad2deg(gamma + alpha);
  delta = 90 - epsilon;
  omega = 180 - delta - rad2deg(beta);
  
  calculatedAngles.joint2angle = omega;
  
  return calculatedAngles;
}
'''
'''
XYCoordinates calculateForwardKinematics(float joint1Angle, float joint2Angle){
  
  coordinates.X = verLinkLength * cosf(deg2rad(joint1Angle)) + horLinkLength * cosf(deg2rad(joint2Angle));
  coordinates.Y = verLinkLength * sinf(deg2rad(joint1Angle)) - horLinkLength * sinf(deg2rad(joint2Angle));
  
  return coordinates;
}
'''
'''
float deg2dc(float angleDeg) {
  float dc = 0;
  dc = (180-angleDeg)/dutyCycle2DegFactor;
  return dc;
}

float dc2deg(float dc) {
  float angleDeg = 0;
  angleDeg = 180 - (dc * dutyCycle2DegFactor);
  return angleDeg;
}
'''
'''
float deg2rad(float deg){
  float rad = 0;
  rad = (deg * F32_PI) / 180.0;
  return rad;
}

float rad2deg(float rad){
  float deg = 0;
  deg = (rad * 180) / F32_PI;
  return deg;
}
'''