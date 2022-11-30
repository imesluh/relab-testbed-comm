from opcua import Client, ua
import time
import datetime
import numpy as np
import gevent
import os
import csv
import traceback
import pdb

def connect(address):
    #global conn, nodes
    conn = Client(address)
    conn.connect()
    nodes = conn.get_objects_node()
    # try:
    #     conn.export_xml([nodes], "/var/www/robotiki/opcua_nodes.xml")
    # except Exception as e:
    #     pdb.set_trace()
    #     print(e)
    #     print(traceback.format_exc())
    return conn, nodes

def disconnect(conn):
    conn.disconnect()
    print("Disconnected OPCUA communication.")

def communicate_backup(nodes, intType, intData, timeout):
    """
    Die Funktion sendet einen Wert an den Versuchsstand (wenn intType = 0, 2, 3 oder 4) bzw. liest den Wert (wenn
    intType = 1

    :param nodes: Nodes der OPCUA-Schnittstelle
    :type nodes: OPCUA-Nodes
    :param intType: Zähler bzw. Kanal, welcher Wert geschickt bzw. gelesen wird:
                    0: Labor (write) (1-4)
                    1: Status (read), WERTE: 0: Ready; 1: Busy; 2: Wert wurde an den Yu gesendet
                    2: Start (write)
                    3: Achszähler (1-6) (write)
                    4: Winkel in Grad (write)
    :type intType: integer oder list of integer
    :param intData: Zahlenwert, nur beim Schreiben von Interesse
    :type intData: integer oder list of integer
    :param timeout: max. Zeit in Sekunden für das Senden
    :type timeout: float

    :return: Erfolg (beim Senden) oder Wert (beim Lesen)
    :rtpype: boolean bzw. integer
    """
    try:
        # t = time.localtime()
        # current_time = time.strftime("%H:%M:%S", t)
        # if intType != 1:
        #     print("$$$$ [" + current_time + "]: tbc.communicate(nodes, intType=" + str(intType) + ", intData = " + str(intData) + ")") #debug print
        # print(nodes.get_children()[7].get_children()[1].get_children())
        # print(nodes.get_children()[7].get_children()[1].get_children()[1].get_display_name())
        # print(nodes.get_children()[7].get_children()[1].get_children()[1].get_variables())
        if intType==1:
            busy = nodes.get_children()[7].get_children()[1].get_children()[intType].get_value()
            return busy
        else:
            if not isinstance(intType, list):
                intType = [intType]
                intData = [intData]
            if int(3) in intType:
                print("Achswerte an YU:")
                print(intType)
                print(intData)
            elif int(0) in intType:
                print("Labornummer an Yu:" + str(intData))
            elif int(2) in intType:
                print("Start an Yu: " + str(intData))
            for intT, intD in zip(intType, intData):
                if intT in [0,2,3,4]:
                    nodes.get_children()[7].get_children()[1].get_children()[intT].set_value(int(intD),
                                                                                             ua.VariantType.Int32)
                else:
                    raise Exception('intType muss in der Range 0...4 liegen.')
            nodes.get_children()[7].get_children()[1].get_children()[1].set_value(2, ua.VariantType.Int32)
            # print(nodes.get_children())
            # print(nodes.get_children()[7].get_children())
            # print(nodes.get_children()[7].get_children()[1].get_children())
            # print(nodes.get_children()[7].get_children()[1].get_children()[1].get_display_name())
            #print(nodes.get_children()[7].get_children()[1].get_children()[1].get_methods())
            #print(nodes.get_children()[7].get_children()[1].get_children()[1].get_variables())
            # print(nodes.get_children_descriptions)
            tStart = datetime.datetime.now()
            while True:
                if nodes.get_children()[7].get_children()[1].get_children()[1].get_value() < 2:
                    return True
                if (datetime.datetime.now()-tStart).total_seconds()>timeout:
                    print(nodes.get_children()[7].get_children()[1].get_children()[1].get_value())
                    raise Exception('Das Packet ist nicht innerhalb des Timeout angekommen.')
                time.sleep(0.001)
    except Exception as e:
        raise e

def communicate(nodes, intType, intData, timeout):
    """
    Die Funktion sendet einen Wert an den Versuchsstand (wenn intType = 0, 2, 3 oder 4) bzw. liest den Wert (wenn
    intType = 1

    :param nodes: Nodes der OPCUA-Schnittstelle
    :type nodes: OPCUA-Nodes
    :param intType: Zähler bzw. Kanal, welcher Wert geschickt bzw. gelesen wird:
                    0: Labornummer (write) (States fuer die State Machine auf dem Yu)
                    1: Status (read), WERTE: 0: Ready; 1: Busy; 2: Wert wurde an den Yu gesendet
                    2: Start (write)
                    3: Achszähler (1-6) (write)
                    4: Winkel in Grad (write)
    :type intType: integer oder list of integer
    :param intData: Zahlenwert, nur beim Schreiben von Interesse
    :type intData: integer oder list of integer
    :param timeout: max. Zeit in Sekunden für das Senden
    :type timeout: float

    :return: Erfolg (beim Senden) oder Wert (beim Lesen)
    :rtpype: boolean bzw. integer
    """
    try:
        if intType==1:
            # read value from opcua node
            busy = nodes.get_children()[7].get_children()[1].get_children()[intType].get_value()
            return busy
        else:
            # write data to opcua node
            if np.isscalar(intType):       #not isinstance(intType, list):
                intType = [intType]
                intData = [intData]
            if int(3) in intType:
                print("Achswerte an YU: " + str(intData[0]) + " - " + str(intData[1]/100) + " degree")
                # print(intType)
                # print(intData)
            elif int(0) in intType:
                print("Labornummer an Yu:" + str(intData))
            elif int(2) in intType:
                print("Start an Yu: " + str(intData))
            for intT, intD in zip(intType, intData):
                if not (intT in [0,2,3,4]):
                    raise Exception('tb.communicate(): intType muss in der Range 0...4 liegen.')
                nodes.get_children()[7].get_children()[1].get_children()[intT].set_value(int(intD), ua.VariantType.Int32)   # write value to opcua node
                time.sleep(0.001)

            nodes.get_children()[7].get_children()[1].get_children()[1].set_value(2, ua.VariantType.Int32)  # tell the server we wrote new values
            # check if server read the new value (if server set "status" to busy or ready)
            tStart = datetime.datetime.now()
            while True:
                if nodes.get_children()[7].get_children()[1].get_children()[1].get_value() < 2:
                    break
                if (datetime.datetime.now() - tStart).total_seconds() > timeout:
                    print(' >>>>>>>>>>>>> Das Paket ist nicht innerhalb des Timeout angekommen.')
                    return False
                time.sleep(0.001)
            # all new values were read by the server
            return True
    except Exception as e:
        #print(traceback.print_exc())
        raise e

def sendLabNumber(nodes, labnumber):
    """
    Send lab number (State) to the Yu
    """
    return communicate(nodes, int(0), int(labnumber), timeout=5)

def startAction(nodes):
    """
    Setze "Start" auf True: Nur dann kann der Yu in einen State wechseln und z.B. Bewegungen ausfuehren
    """
    communicate(nodes, int(2), 1, timeout=5)  # Starten der Bewegung
    time.sleep(0.15) # warten bis gevent-thread mind. einmal Status vom Versuchsstand abgefragt hat (der nicht mehr ready ist)

def isReady(nodes):
    """
    return if Yu is ready
    """
    if (getStatus(nodes) == 0):
        return True
    else:
        return False

def getStatus(nodes):
    """
    return Status of Yu
    0: ready
    1: Busy
    2: value sent to the Server, but server did not yet read
    """
    status = communicate(nodes, int(1), 1, timeout=5)
    return status

def sendAxValues_deg(nodes, q_deg):
    """
    Die Funkion sendet Achswerte (in Grad) an den Yu.

    :param nodes: Nodes der OPCUA-Schnittstelle
    :type nodes: OPCUA-Nodes
    :param q_deg: Liste oder numpy array der Achswerte in Grad
    :type q_deg: list or numpy array

    :return: Success
    :rtpype: boolean
    """
    if np.isscalar(q_deg):
        q_deg = [q_deg]
    length = 0
    try:
        length = len(q_deg)
    except:
        length = q_deg.size
    if (length != 6):
        print("sendAxValues_deg(): Ungültiges Argument übergeben.")
        return False

    bSuccess = True
    for winkel, achse in zip(q_deg, range(len(q_deg))):  # Achswinkel senden
        bSent = communicate(nodes, [3, 4], [achse + 1, winkel * 100], timeout=5)
        if not bSent:
            print("Winkel fuer Achse mit Index " + str(achse) + " konnte nicht uebermittelt werden.")
            bSuccess = False
            raise Exception('Uebermittlung der Achswinkel an den Yu fehlgeschlagen.')
        time.sleep(0.001)
    return bSuccess

def turnOnLight(nodes):
    if sendLabNumber(nodes, 91):
        startAction(nodes)

def turnOffLight(nodes):
    if sendLabNumber(nodes, 90):
        startAction(nodes)

def readAxValues(nodes):
    """
    Die Funktion liest die Achswerte aus. Im Gegensatz zum lesen der Werte in communicate() sind diese Werte nicht von
    uns angelegt, sondern existieren so auf der SPS. Daher können die Werte unbegrenzt gelesen werden.

    :param nodes: Nodes der OPCUA-Schnittstelle
    :type nodes: OPCUA-Nodes

    :return: Zeitstempel (Timestamp als integer)
    :rtpype: integer
    :return: Achswinkel
    :rtpype: numpy array (6)
    :return: Achsgeschwindigkeiten
    :rtpype: numpy array (6)
    :return: Karteisische Position + Orientierung
    :rtpype: numpy array (6)
    """
    q = [x*180/np.pi for x in nodes.get_children()[2].get_children()[0].get_value()]
    dq = [x*180/np.pi for x in nodes.get_children()[2].get_children()[1].get_value()]
    t = nodes.get_children()[2].get_children()[0].get_data_value().SourceTimestamp.timestamp()
    pos = [x for x in nodes.get_children()[3].get_children()[0].get_value()]
    pos[:3] = pos[:3]*1000
    return t, q, dq, pos


def write_target_data(filename, basedir, indices, clmnNames, yu_nodes, stop, *args):
    """
        Die Funktion schreibt die Daten mit der gewünschten Samplezeit (vorgegeben am Anfang der Funktoin) i eine csv-Datei.

        :param filename: Name der csv-Datei, wenn leer, wird das Array im Arbeitsspeicher zurückgegeben
        :type filename: basestring
        :param basedir: Pfad der Datei (ohne subpfad exchange, der in der Funktion gesetzt wird)
        :type basedir: basestring
        :param indices: Indizes aus dem Messvektor, die gespeichert werden sollen (ohne die Zeit t, Indizes daher um 1 verschoben)
        :type indices : list of int
        :param clmnNames: Lister der Spaltennamen für den Header der csv (mit der Zeit t)
        :type clmnNames : list of basestring
        :param yu_nodes: Nodes der OPC-UA-Verbindung zum Yu
        :tyoe yu_nodes: opcua nodes
        :param stop: Art des Beenden der Aufzeichnung
        :type stop: {'extern', 'intern'}
        :param args: wenn der stop 'intern' ist, muss die Dauer der Aufzeichnung vorgegeben werden
        :type args: float
        """
    # Initialisierung
    sample_time = 0.05
    data_now = []
    data = []
    for index in indices:
        data_now.append(0)
    first = True
    lastT = -1
    # Wenn der Stop der Aufzeichnung extern getriggert wird (durch eine kill des aufrufenden Greenlets glet.kill())
    if stop.lower() == 'extern':
        try:
            os.remove(basedir + '/exchange/' + filename)
        except:
            pass
        while True:
            with open((basedir + '/exchange/' + filename), 'a') as csv_clear:
                spamwriter = csv.writer(csv_clear, delimiter=';', lineterminator='\n')
                if first:
                    t, q, dq, pos = readAxValues(yu_nodes)
                    startT = t
                    spamwriter.writerow(clmnNames)
                    first = False
                t, q, dq, pos = readAxValues(yu_nodes)
                if (t - lastT) >= 0.97 * sample_time:
                    lastT = t
                    signal = np.concatenate((q, dq, pos))
                    data_now = [signal[x] for x in indices]
                    data_now.insert(0, t-startT)
                    spamwriter.writerow(data_now)
            gevent.sleep(0.001)
    else:
        # Aufzeichnen für die Dauer, die über args übermittelt wird.
        t_end = datetime.datetime.now() + datetime.timedelta(seconds=args[0]) + datetime.timedelta(seconds=0.1)
        t_start = datetime.datetime.now()
        t_now = t_start
        while t_now < t_end:
            t, q, dq, pos = readAxValues(yu_nodes)
            t_now = datetime.datetime.now()
            if (t - lastT) >= 0.97 * sample_time:
                lastT = t
                signal = np.concatenate((t, q, dq, pos))
                data.append([signal[x] for x in indices])
            gevent.sleep(0.001)
        time = data[0][0]
        if isinstance(filename[0], str):
            with open((basedir + '/exchange/' + filename), 'a') as csv_clear:
                spamwriter = csv.writer(csv_clear, delimiter=';', lineterminator='\n')
                spamwriter.writerow(clmnNames)
                for row in data:
                    row[0] = row[0] - time
                    spamwriter.writerow(row)
        else:
            return data

def send_ITP(nodes, to):
    """
            Initial target post: Yu wird in Home-Position gefahren.

            :param nodes: Nodes der OPCUA-Schnittstelle
            :type nodes: OPCUA-Nodes
            :param to: timeout,max. Zeit in Sekunden für das Senden
            :type to: float
            """
    print("%%% ITP")
    while not isReady(nodes):
        time.sleep(0.5)
        print("%%% ITP: Warte bis Yu bereit ist, um ITP zu senden.")

    # "Labornummer" (State) fuer den Yu senden
    #communicate(nodes, int(0), int(99), timeout=5)
    sendLabNumber(nodes, 99)
    # Starten der Bewegung
    #communicate(nodes, int(2), 1, timeout=to)  # Starten der Bewegung
    startAction(nodes)   # Starten der Bewegung
    busy = True
    # warten bis Home-Bewegung durchgeführt
    start = datetime.datetime.now()
    busy = communicate(nodes, int(1), 0, timeout=to)
    while busy:
        if (datetime.datetime.now() - start).total_seconds() > to:
            if busy == 2:
                print(' >>>>>>>>>>>>> ITP: Das Paket ist nicht innerhalb des Timeout angekommen.')
                break
        busy = communicate(nodes, int(1), 0, timeout=to)
        print("%%% ITP: moving towards home position..")
        time.sleep(0.5)
