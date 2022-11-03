"""
Kinematikfunktionen des Yuanda Yu und kinematik-bezogene mathematische Funktionen
"""

import numpy as np
import math


def transRot2T(r,R):
    """
    Erstelle homogene Transformationsmatrix [4x4] aus Translation und Rotation
    """
    if R.ndim>2:
        R = R[:,:,0]
    T = np.zeros((4, 4))
    T[3, 3] = 1
    T[0:3, 3] = r
    T[0:3, 0:3] = R
    return T

def rad2deg(qs):
    """
    Die Funktion rechnet die Gelenkwinkel in rad in ° um.

    :param qs: Gelenkwinkel in rad
    :type qs: numpy.array, list

    :rtype: numpy.array, list
    """
    try:
        i=0
        for q in qs:
            qs[i] = q*180/np.pi
            i+=1
    except:
        qs = np.float64(qs*180/np.pi)
    return qs


def deg2rad(qs):
    """
        Die Funktion rechnet die Gelenkwinkel in ° in rad um.

        :param qs: Gelenkwinkel in °
        :type qs: numpy.array, list

        :rtype: numpy.array, list
        """
    i = 0
    for q in qs:
        qs[i] = q * np.pi/180
        i += 1
    return qs


def Rx(theta):
    """
        Die Funktion berechnet die Rotation der Einzeldrehungen mit den Orientierungswinkeln um die jeweiligen Achsen

        :param theta: Orientierungswinkel in rad
        :type theta: numpy.array, list

        :return:Einzeldrehung Rotationsmatrix um eine Achse
        :rtype: numpy.array, list

        """
    return np.matrix([[ 1, 0 , 0 ],
    [ 0, math.cos(theta),-math.sin(theta)],
    [ 0, math.sin(theta), math.cos(theta)]])

def Ry(theta):
    return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
    [ 0 , 1, 0 ],
    [-math.sin(theta), 0, math.cos(theta)]])

def Rz(theta):
    return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
    [ math.sin(theta), math.cos(theta) , 0 ],
    [ 0 , 0 , 1 ]])

def r2eulxyz(R):
    """
    Umwandlung von Rotationsmatrix nach Euler-Darstellung xyz
    :param R: Rotationsmatrix [3x3]

    :return phi: Eulerwinkel xyz in rad[3x1]
    :rtype: numpy.list
    """
    r11=R[0,0]
    r12=R[0,1]
    r13=R[0,2]
    r21=R[1,0]
    r22=R[1,1]
    r23=R[1,2]
    r31=R[2,0]
    r32=R[2,1]
    r33=R[2,2]
    # Umrechnung Rotationsmatrix in Euler-Winkel xyz
    t1 = [math.atan2(r21, -r31), math.atan2(math.sqrt(r21 ** 2 + r31 ** 2), r11), math.atan2(r12, r13)]
    phi = t1  # Eulerwinkel xyz
    return np.vstack(phi)

def r2eulzxz(R):
    """
    Umwandlung von Rotationsmatrix nach Euler-Darstellung zxz in Rad
    :param R: Rotationsmatrix [3x3]

    :return phi: Eulerwinkel zxz in rad[3x1]
    :rtype: numpy.list
    """
    r11=R[0,0]
    r12=R[0,1]
    r13=R[0,2]
    r21=R[1,0]
    r22=R[1,1]
    r23=R[1,2]
    r31=R[2,0]
    r32=R[2,1]
    r33=R[2,2]
    # Umrechnung Rotationsmatrix in Euler-Winkel
    t1 = [math.atan2(r23, r13), math.atan2(math.sqrt(r31 ** 2 + r32 ** 2), r33), math.atan2(r32, -r31)]
    phi = t1
    return np.vstack(phi)

def r2eulxzx(R):
    """
    Umwandlung von Rotationsmatrix nach Euler-Darstellung xzx in rad
    :param R: Rotationsmatrix [3x3]
    :return phi: Eulerwinkel xyz in rad[3x1]
    :rtype: numpy.list
    """
    r11=R[0,0]
    r12=R[0,1]
    r13=R[0,2]
    r21=R[1,0]
    r22=R[1,1]
    r23=R[1,2]
    r31=R[2,0]
    r32=R[2,1]
    r33=R[2,2]
    # Umrechnung Rotationsmatrix in Euler-Winkel xzx
    t1 = [math.atan2(r31, r21), math.atan2(math.sqrt(r21 ** 2 + r31 ** 2), r11), math.atan2(r13, -r12)]
    phi = t1  # Eulerwinkel xzx
    return np.vstack(phi)

def rpy2r(phi):
    """
    Umwandlung von RPY-Darstellung zu Rotationsmatrix
    :param: phi: RPY in rad [3x1]
    :return R: Rotationsmatrix [3x3]
    :rtype: numpy.array
    """
    alpha = phi[0]      #yaw
    beta = phi[1]       #pitch
    gamma = phi[2]      #roll

    ca = math.cos(alpha)
    sa = math.sin(alpha)
    cb = math.cos(beta)
    sb = math.sin(beta)
    cc = math.cos(gamma)
    sc = math.sin(gamma)

    r = np.array([  [cg * cb, cg*sb*sa-sg*ca, cg*sb*ca+sg*sa],
                    [sg*cb, sg*sb*sa+cg*ca, sg*sb*ca-cg*sa],
                    [-sb, cb*sa, cb*ca]])
    return r

def eulzxz2r(phi):
    """
    Umwandlung von Euler-Darstellung xyz zu Rotationsmatrix
    :param: phi: Eulerwinkel zxz in rad [3x1]
    :return R: Rotationsmatrix [3x3]
    :rtype: numpy.array
    """
    phi1 = phi[0]
    phi2 = phi[1]
    phi3 = phi[2]
    t89 =  math.sin(phi1)
    t91 =  math.cos(phi2)
    t95 = t89 * t91
    t87 =  math.sin(phi3)
    t92 =  math.cos(phi1)
    t94 = t92 * t87
    t90 = math.cos(phi3)
    t93 = t92 * t90
    t88 =  math.sin(phi2)
    r = np.array([ [-t87 * t95 + t93, -t90 * t95 - t94, t89 * t88],
                    [t89 * t90 + t91 * t94, -t89 * t87 + t91 * t93, -t92 * t88],
                    [t88 * t87, t88 * t90, t91]])
    return r

def eulxyz2r(phi):
    """
    Umwandlung von Euler-Darstellung xyz zu Rotationsmatrix
    :param: phi: Eulerwinkel xyz in rad [3x1]
    :return R: Rotationsmatrix [3x3]
    :rtype: numpy.array
    """
    phi1 = phi[0]
    phi2 = phi[1]
    phi3 = phi[2]
    t10 = math.sin(phi3)
    t12 = math.sin(phi1)
    t19 = t12 * t10
    t13 = math.cos(phi3)
    t18 = t12 * t13
    t15 = math.cos(phi1)
    t17 = t15 * t10
    t16 = t15 * t13
    t14 = math.cos(phi2)
    t11 = math.sin(phi2)
    r = np.array([ [t14 * t13, -t14 * t10, t11],
                    [t11 * t18 + t17, -t11 * t19 + t16, -t12 * t14],
                    [-t11 * t16 + t19, t11 * t17 + t18, t15 * t14]])
    return r

def eulxzx2r(phi):
    """
    Umwandlung von Euler-Darstellung xzx zu Rotationsmatrix
    :param: phi: Eulerwinkel xzx in rad [3x1]
    :return R: Rotationsmatrix [3x3]
    :rtype: numpy.array
    """
    phi1 = phi[0]
    phi2 = phi[1]
    phi3 = phi[2]
    t22 = math.sin(phi1)
    t24 = math.cos(phi2)
    t28 = t22 * t24
    t20 = math.sin(phi3)
    t25 = math.cos(phi1)
    t27 = t25 * t20
    t23 = math.cos(phi3)
    t26 = t25 * t23
    t21 = math.sin(phi2)
    r = np.array([ [t24, -t21 * t23, t21 * t20],
                    [t25 * t21, -t22 * t20 + t24 * t26, -t22 * t23 - t24 * t27],
                    [t22 * t21, t23 * t28 + t27, -t20 * t28 + t26]])
    return r

def r2quat(R):
    """
    Umwandlung von Rotationsmatrix nach Quaternionen-Darstellung
    :param R: Rotationsmatrix [3x3]

    :return q: Quaterionen: Erste w (Winkel), dann xyz (Rotationsachsen) [4x1]
    :rtype: numpy.list
    """
    # transponieren, damit uebereinstimmend mit Robotik-Skript und Stanford-Paper
    R = R.transpose()
    r11=R[0,0]
    r12=R[0,1]
    r13=R[0,2]
    r21=R[1,0]
    r22=R[1,1]
    r23=R[1,2]
    r31=R[2,0]
    r32=R[2,1]
    r33=R[2,2]
    # Umrechnung Rotationsmatrix in Quaterionen-Darstellung
    tr = r11 + r22 + r33
    if (tr > 0): #% analog zu [2], equ. (3.14)
        S = math.sqrt(tr+1.0) * 2  #% S=4qw
        qw = 0.25 * S
        qx = (r32 - r23) / S
        qy = (r13 - r31) / S
        qz = (r21 - r12) / S
    elif ((r11 > r22) and (r11 > r33)): #% analog zu [2], equ. (3.15)
        S = math.sqrt(1.0 + r11 - r22 - r33) * 2  #% S=4qx
        qw = (r32 - r23) / S
        qx = 0.25 * S
        qy = (r12 + r21) / S
        qz = (r13 + r31) / S
    elif (r22 > r33): #% analog zu [2], equ. (3.16)
        S = math.sqrt(1.0 - r11 + r22 - r33) * 2  #% S=4qy
        qw = (r13 - r31) / S
        qx = (r12+ r21) / S
        qy = 0.25 * S
        qz = (r23 + r32) / S
    else: #% analog zu [2], equ. (3.17)
        S = math.sqrt(1.0 -r11 - r22 + r33) * 2  #% S=4qz
        qw = (r21 - r12) / S
        qx = (r13 + r31) / S
        qy = (r23 + r32) / S
        qz = 0.25 * S
    q = np.array([qw,qx,qy,qz])
    return np.vstack(q)


def r2angvec(R):
    """
    Umwandlung von Rotationsmatrix nach Achse-Winkel-Konvention
    :param R: Rotationsmatrix [3x3]

    :return q: Achse-Winkel-Darstellung der Rotation [theta, n] theta [1x1] und n [3x1]
    :rtype: , numpy.float, numpy.list
    """
    r11=R[0,0]
    r12=R[0,1]
    r13=R[0,2]
    r21=R[1,0]
    r22=R[1,1]
    r23=R[1,2]
    r31=R[2,0]
    r32=R[2,1]
    r33=R[2,2]

    #% [Robotik I, Gl. (2.39)]
    cos_theta = 0.5*(r11+r22+r33-1)
    if cos_theta == 1:
        theta = 0
        n = [0,0,1]
    else:
    #% [Robotik I, Gl. (2.42)]
        sin_theta = 0.5 * math.sqrt((r32-r23)**2+(r13-r31)**2+(r21-r12)**2)
        #% [Robotik I, Gl. (2.43)]
        theta = math.atan2(sin_theta, cos_theta)
        #% [Robotik I, Gl. (2.44)]
        x = np.array([(r32 - r23), (r13 - r31), (r21 - r12)])
        n = 1/(2*sin_theta) * x

    return theta, n#np.vstack(n)


def angvec2r(u, theta):
    """
    Rotationsmatrix aus Achse-Winkel-Darstellung berechnen

    Eingabe:
        Drehwinkel theta (float) [1x1]
        Drehachse u (vector) [3x1]

    Ausgabe:
        Rotationsmatrix R_u [3x3]
    """
    # Init
    u = np.asarray(u)
    theta = np.asarray(theta)

    ## Calculation
    # Hilfsvariabeln
    cth = math.cos(theta)
    sth = math.sin(theta)
    vth = (1 - cth)

    ux = u[0]
    uy = u[1]
    uz = u[2]

    # 3x3 Rotationmatrix
    # Quelle: Skript Robotik I (WS 2021/22), Ortmaier, Uni Hannover, Gl. 2.37
    R_u = np.array([[ux ** 2 * vth + cth, ux * uy * vth - uz * sth, ux * uz * vth + uy * sth],
                    [ux * uy * vth + uz * sth, uy ** 2 * vth + cth, uy * uz * vth - ux * sth],
                    [ux * uz * vth - uy * sth, uy * uz * vth + ux * sth, uz ** 2 * vth + cth]])
    return R_u


def quat2r(quat):
    """
    Rotationsmatrix aus Quaterionen-Darstellung berechnen
    Quelle: Skript Robotik 1 WiSe 22/23

    Eingabe:
        Quaterionenvektor quat [1x4]

    Ausgabe:
        Rotationsmatrix R_q [3x3]

    """

    # Init
    quat = np.asarray(quat)
    # Extract the values from quat
    a = quat[0]  # Angle (Real Part)
    b = quat[1]  # Axis (i)
    c = quat[2]  # Axis (j)
    d = quat[3]  # Axis (k)

    # 3x3 Rotationmatrix
    # Quelle: Skript Robotik I (WS 2021/22), Ortmaier, Uni Hannover, Gl. 2.55
    R_q = np.array([[a ** 2 + b ** 2 - c ** 2 - d ** 2, 2 * b * c - 2 * a * d, 2 * b * d + 2 * a * c],
                    [2 * b * c + 2 * a * d, a ** 2 - b ** 2 + c ** 2 - d ** 2, 2 * c * d - 2 * a * b],
                    [2 * b * d - 2 * a * c, 2 * c * d + 2 * a * b, a ** 2 - b ** 2 - c ** 2 + d ** 2]])
    if R_q.ndim > 2:
        R_q = R_q[:,:,0]
    # Transponierte bilden, damit mit Robotik-Skript (und Stanford-Paper uebereinstimmt)
    R_q = R_q.transpose()
    return R_q


def direct_kinematics(q):
    """
    Die Funktion bestimmt aus den gegeben Gelenkwinkeln des Yus in rad die Positionen der Armenden in x,y,z und die Pose
    in Kardanwinkel-Notation.

    :param q: 6 Gelenkwinkel des Yu in rad
    :type q:  numpy.array, list

    :return: kartesische Positionen und Orientierungen (Kardanwinkel-Notation) Spalten stehen für die verschiedenen
             Armsegmente. Endeffektor ist die letzte Spalte.; 4x4 Transformationsmatrizen zum zu jedem DH-KS (letztes ist Endeffektor)
    :rtype: numpy.array, numpy.array
    """
    l = np.array([0.17495, 0.1791, 0.450, 0.1751, 0.4078, 0.1422, 0.1422, 0.3290]) # Längen des Yu
    DHP=np.array([[q[0],      l[0],       0,      -np.pi/2],
                  [q[1],      l[1],       l[2],    0],
                  [q[2],     -l[3],       l[4],    0],
                  [q[3],     -l[5],       0,       np.pi/2],
                  [q[4],     -l[6],       0,      -np.pi/2],
                  [q[5],     -l[7],       0,       np.pi]]) # DH-Parameter des Yu

    #Initialisierung und Berechnung der Denavit-Hartenberg-Matrizen
    A=np.zeros(shape=(4,4,6))
    for u in range(6):
        A[:,:,u]=np.array([[np.cos(DHP[u,0]),   -np.sin(DHP[u,0])*np.cos(DHP[u,3]),   np.sin(DHP[u,0])*np.sin(DHP[u,3]),    DHP[u,2]*np.cos(DHP[u,0])],
                           [np.sin(DHP[u,0]),   np.cos(DHP[u,0])*np.cos(DHP[u,3]),    -np.cos(DHP[u,0])*np.sin(DHP[u,3]),   DHP[u,2]*np.sin(DHP[u,0])],
                           [0,               np.sin(DHP[u,3]),                  np.cos(DHP[u,3]),                  DHP[u,1]],
                           [0,               0,                              0,                              1]])

    #Berechnung der Transformationsmatrizen
    T = np.zeros(shape=(4,4,6))
    T[:,:,0]=A[:,:,0]
    T[:,:,1]=np.linalg.multi_dot((A[:,:,0],A[:,:,1]))
    T[:,:,2]=np.linalg.multi_dot((A[:,:,0],A[:,:,1],A[:,:,2]))
    T[:,:,3]=np.linalg.multi_dot((A[:,:,0],A[:,:,1],A[:,:,2],A[:,:,3]))
    T[:,:,4]=np.linalg.multi_dot((A[:,:,0],A[:,:,1],A[:,:,2],A[:,:,3],A[:,:,4]))
    T[:,:,5]=np.linalg.multi_dot((A[:,:,0],A[:,:,1],A[:,:,2],A[:,:,3],A[:,:,4],A[:,:,5])) # 4x4 Trafo

    #Initialisierung der Positionsmatrix und Orientierungsmatrix
    Position=np.zeros(shape=(3,6))
    Orientierung=np.zeros(shape=(3,6))


    #Berechnung von Position und Orientierung der einzelnen Koordinatensysteme
    for u in range(6):
        Position[:,u]=T[:3,3,u]

        #Berechnung der Orientierung (in Kardanwinkeln)
        alpha=np.arctan2(-T[1,2,u],T[2,2,u])
        gamma=np.arctan2(-T[0,1,u],T[0,0,u])
        beta=np.arctan2(T[0,2,u],T[0,0,u]*np.cos(gamma)-T[0,1,u]*np.sin(gamma))
        Orientierung[:,u]=[alpha,beta,gamma]


    #Rückgabe der Orientierung in 6-FHG
    # TODO: Rueckgabe trennen in Pos+Orientierung?
    return np.vstack((Position,Orientierung)), T

def detect_invalid_angpos(q, workspace, dim):
    """
    Die Funktion ermittelt welche Achsen den Arbeitsraum (kartesisch oder zylindrisch) verletzen.

    :param q: 6 Gelenkwinkel des Yu in rad (ein Gelenkwinkel pro Zeile bei mehreren Positionen)
    :type q:  numpy.array, list
    :param workspace: Art des Arbeitsraumes
    :type workspace:  'cartesian', 'cylindric'
    :param dim: Ausdehnung des Arbeitsraumes (x,y,z für 'cartesian' und r,h für 'cylindric'). Die y- und y-Werte für den
                kartesischen Arbeitsraum werden positiv und negativ berücksichtigt, der z-Wert rein positiv.
    :type dim: numpy.array, list

    :return: Indizes der Achsen, welche den Arbeitsraum verletzen
    :rtype: list
    """
    X_cart = direct_kinematics(q)[0][:3, :]
    if workspace.lower()=='cartesian':
        ind = np.where((X_cart[0,1:]>dim[0]) | (X_cart[0,1:]<-dim[0]) |
                       (X_cart[1,1:]>dim[1]) | (X_cart[1,1:]<-dim[1]) |
                       (X_cart[2,1:]>dim[2]) | (X_cart[2,1:]<0.2))
    elif workspace.lower()=='cylindric':
        ind = np.where(((X_cart[0, 1:] ** 2 + X_cart[1, 1:] ** 2) > dim[0] ** 2) |
                       (X_cart[2, 1:] > dim[1]) | (X_cart[2, 1:] < 0.2))[0]
    ind = [x + 1 for x in ind]
    return ind


def generatePoses(numPos, validPos, workspace, dim, gap):
    """
    Die Funktion generiert eine gewählte Anzahl von Positionen innerhalb sowie außerhalb des Arbeitsraumes

    :param numPos: Anzahl an Positionen
    :type numPos:  integer
    :param validPos: Anzahl an Positionen innerhalb des Arbeitsraumens
    :type validPos:  integer
    :param workspace: Art des Arbeitsraumes
    :type workspace:  'cartesian', 'cylindric'
    :param dim: Ausdehnung des Arbeitsraumes (x,y,z für 'cartesian' und r,h für 'cylindric'). Die y- und y-Werte für den
                kartesischen Arbeitsraum werden positiv und negativ berücksichtigt, der z-Wert rein positiv.
    :type dim: numpy.array, list
    :param gap: Toleranzband um die Arbeitsraumgrenze
    :type gap: flat

    :return: Positionen in Grad
    :rtype: list
    :return: Position valide oder invalide
    :rtype: list of boolean
    """
    samples=100
    q_lim = np.array([[-180, 180],
                      [-180, 10],
                      [-130, 130],
                      [-180, 130],
                      [-92, 0],
                      [0, 0]])
    pose = []
    valid= []
    validC = 0
    invalidC = 0
    while validC+invalidC<numPos:
        qs = deg2rad(np.random.uniform(q_lim[:, 0], q_lim[:, 1], (samples, 6)))
        for q in qs:
            ind = detect_invalid_angpos(q, workspace, [x-gap for x in dim])
            if len(ind)==0 and validC<validPos:
                pose.append(rad2deg(q))
                valid.append(1)
                validC+=1
            else:
                ind = detect_invalid_angpos(q, workspace, [x+gap for x in dim])
                if len(ind)>0 and invalidC<numPos-validPos:
                    pose.append(rad2deg(q))
                    valid.append(0)
                    invalidC+=1
                elif validC+invalidC == numPos:
                    break
    pose = np.vstack(pose)
    valid = np.array(valid)
    inds = np.arange(numPos)
    np.random.shuffle(inds)
    return pose[inds], valid[inds].tolist()
def geoJacobi(q):
    """
    Die Funktion generiert Dir Geometrische Jacobi einer gegeben Gelenkskoordinaten

    :param q: gelenkskoordinaten in grad
    :type q:  list
    
    :return: geometrische Jacobi [6x6] 
    :rtype: list
    """
    xE, T = direct_kinematics(q)
    e_z_0 = [0, 0, 1]
    e_z_1 = np.dot(T[0:3, 0: 3, 1], e_z_0)
    e_z_2 = np.dot(T[0:3, 0: 3, 2], e_z_0)
    e_z_3 = np.dot(T[0:3, 0: 3, 3], e_z_0)
    e_z_4 = np.dot(T[0:3, 0: 3, 4], e_z_0)
    e_z_5 = np.dot(T[0:3, 0: 3, 5], e_z_0)
    r_0_0 = [0, 0, 0, 1]
    r_0_E = np.dot(T[:, :, 5], r_0_0) - r_0_0
    r_1_E = np.dot(T[:, :, 5], r_0_0) - np.dot(T[:, :, 0],r_0_0)
    r_2_E = np.dot(T[:, :, 5], r_0_0) - np.dot(T[:, :, 1],r_0_0)
    r_3_E = np.dot(T[:, :, 5], r_0_0) - np.dot(T[:, :, 2],r_0_0)
    r_4_E = np.dot(T[:, :, 5], r_0_0) - np.dot(T[:, :, 3],r_0_0)
    r_5_E = np.dot(T[:, :, 5], r_0_0) - np.dot(T[:, :, 4],r_0_0)
    j_0 = [np.cross(e_z_0, r_0_E[0:3]), e_z_0]
    j_1 = [np.cross(e_z_1, r_1_E[0:3]), e_z_1]
    j_2 = [np.cross(e_z_2, r_2_E[0:3]), e_z_2]
    j_3 = [np.cross(e_z_3, r_3_E[0:3]), e_z_3]
    j_4 = [np.cross(e_z_4, r_4_E[0:3]), e_z_4]
    j_5 = [np.cross(e_z_5, r_5_E[0:3]), e_z_5]
    j_0 = np.reshape(j_0,[6,1])
    j_1 = np.reshape(j_1,[6,1])
    j_2 = np.reshape(j_2,[6,1])
    j_3 = np.reshape(j_3,[6,1])
    j_4 = np.reshape(j_4,[6,1])
    j_5 = np.reshape(j_5,[6,1])
    J_geo = [j_0, j_1, j_2, j_3, j_4, j_5]
    J_geo = np.reshape(J_geo, [6, 6])
    J_geo = np.transpose(J_geo)
    J_geo = J_geo.round(7)
    return J_geo


def iterative_inv_kin(pose, q_start, itr):
    """
    Die Funktion generiert eine Inkrementelle Lösung der Inverse Kinematik bzw der richtigen Position [x, y, z]

    :param pose: Position [x, y, z] in m
    :type pose:  list

    :param q_start: Startgelenkskoordinaten in rad
    :type q_start:  list

    :param itr: Iterationszahl
    :type itr:  int

    :return: Gelenkskoordinaten  [6x1]
    :rtype: list
    """
    i = 1
    while i < itr:
        pose_ink = direct_kinematics(q_start)[0][:, 5]
        d_x = pose - pose_ink[0:3]
        j = geoJacobi(q_start)
        jj = j[0:3, 0:3]
        d_q = np.dot(np.linalg.inv(jj), d_x)
        q_start[0:3] = q_start[0:3] + d_q
        i = i+1
    return q_start

