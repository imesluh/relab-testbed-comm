from opcua import Client, ua
import time
import datetime
import numpy as np


def connect(address):
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
                    0: Labor (write)
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
            tStart = datetime.datetime.now()
            while True:
                if nodes.get_children()[7].get_children()[1].get_children()[1].get_value()<2:
                    return True
                if (datetime.datetime.now()-tStart).total_seconds()>timeout:

                    raise Exception('Das Packet ist nicht innerhalb des Timeout angekommen.')
                time.sleep(0.001)
    except Exception as e:
        raise e

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

