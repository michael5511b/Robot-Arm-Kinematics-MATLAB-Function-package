# Robot-Arm-Kinematics-MATLAB-Function-package
List of Usable functions in this package (More details documented in the MATLAB file comments)

### rotX 
- **R = rotX(theta):** 

  Returns a rotation matrix describing a rotation about the X axis (theta in radians).

### rotY 
- **R = rotY(theta):** 

  Returns a rotation matrix describing a rotation about the Y axis (theta in radians).

### rotZ 
- **R = rotZ(theta):** 

  Returns a rotation matrix describing a rotation about the Z axis (theta in radians).

### rpy2Rot
- **R = rpy2Rot (roll, pitch, yaw):**

  Returns a rotation matrix corresponding to a roll, pitch, yaw encoded rotation.

### rot2RPY
- **[roll, pitch, yaw] = rot2RPY(R):**

  Returns the roll, pitch and yaw corresponding to a given rotation matrix.

### cpMatrix
- **X = cpMatrix(w):**

  eturns the matrix packing of the cross product operator. I.E. Given vectors W and V, cpMatrix(W) * V = W x V

### angleAxis2Rot
- **R = angleAxis2Rot(k, theta):**

  Returns the rotation matrix encoded by a rotation of theta radians about the unit vector k axis.

### rot2AngleAxis
- **[k, theta] = rot2AngleAxis(R):**

  Returns the angle and axis corresponding to a rotation matrix.

### twist2Transform
- **H = twist2Transform(t):**

  Returns the homogenous transformation matrix corresponding to a 6 element twist vector.

### transform2Twist
- **t = transform2Twist(H):**

  Returns the twist vector corresponding to the provided homogenous transform matrix.

### dhTransform
- **H = dhTransform(a, d, alpha, theta):**

  Returns the homogenous transform corresponding to the provide DH parameters for a link.

### createLink
- **L = createLink(a, d, alpha, theta, centOfMass, mass, inertia):**

  Creates a **structure** with the following members:

  a – DH parameter a (meters)

  d – DH parameter d (meters)

  alpha – DH parameter alpha (radians)

  theta – DH parameter theta (radians)

  mass – link mass (kg)

  inertia – link mass moment of inertia (kg m^2)

  com – the position of the link’s center of mass

  isRotary – Boolean true if it is a rotary joint false if it is a prismatic joint.

  All vectors and tensors are to be expressed in the Link’s coordinate frame.

### dhFwdKine
- **H = dhFwdKine(linkList, paramList):**

  Returns the forward kinematics of a manipulator with the provided DH parameter set.

  linkList is to be an array of links, each created by createLink

  paramList is to be an array containing the current state of their joint variables.

### velocityJacobian
- **Jv = velocityJacobian(linkList):**

  Returns the velocity jacobian of the manipulator given an array of links created by the **createLink** function.
  
### dhInvKine
- **[paramList, error] = dhInvKine (linkList, desTransform, paramListGuess):**

  Returns the parameter list necessary to achieve a desired homogenous transform and the residual error in that transform.
  
  linkList – a list of the joint parameters created by createLink

