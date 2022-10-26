from opcua import Client, ua
import time
import datetime
import numpy as np
import gevent
import os
import csv

def connect(address):
    #global conn, nodes
    conn = Client(address)
    conn.connect()
    nodes = conn.get_objects_node()
    return conn, nodes

def disconnect(conn):
    conn.disconnect()


def communicate(nodes, intType, intData, timeout):
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

def getStatus(nodes):
    # nodes.get_children()[7].get_children()[1].get_children()[1].get_value()
    pass
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
    # Roboter in Home-Position fahren
    pos_home = [0,-90,90,-90,-90,0]     #  Achswinkel der home position

    communicate(nodes, int(0), int(0), timeout=to)    # Labornummer "0" für Bewegungsvorgabe
    for angle, axes in zip(pos_home, range(len(pos_home))):  # Achswinkel senden
        communicate(nodes, [3, 4], [axes + 1, angle * 100], timeout=to)
        time.sleep(0.01)

    # Starten der Bewegung
    communicate(nodes, int(2), 1, timeout=to)  # Starten der Bewegung
    state = False
    # warten bis Home-Bewegung durchgeführt
    start = datetime.datetime.now()
    while not state:
        if (datetime.datetime.now() - start).total_seconds() > to:
            break
        state = communicate(nodes, int(1), 0, timeout=to)
        print("%%% moving towards home position..")
        time.sleep(0.1)