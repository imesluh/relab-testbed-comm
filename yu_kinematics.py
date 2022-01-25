import numpy as np



def rad2deg(qs):
    """
    Die Funktion rechnet die Gelenkwinkel in rad in ° um.

    :param qs: Gelenkwinkel in rad
    :type qs: numpy.array, list

    :rtype: numpy.array, list
    """
    i=0
    for q in qs:
        qs[i] = q*180/np.pi
        i+=1
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


def direct_kinematics(q):
    """
    Die Funktion bestimmt aus den gegeben Gelenkwinkeln des Yus in rad die Positionen der Armenden in x,y,z und die Pose
    in Kardanwinkel-Notation.

    :param q: 6 Gelenkwinkel des Yu in rad
    :type q:  numpy.array, list

    :return: kartesische Positionen und Orientierungen (Kardanwinkel-Notation), Spalten stehen für die verschiedenen
             Arme. Endeffektor ist die letzte Spalte.
    :rtype: numpy.array
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
    T[:,:,5]=np.linalg.multi_dot((A[:,:,0],A[:,:,1],A[:,:,2],A[:,:,3],A[:,:,4],A[:,:,5]))

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
    return np.vstack((Position,Orientierung))

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
    X_cart = direct_kinematics(q)[:3, :]
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