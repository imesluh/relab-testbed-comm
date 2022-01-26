from datetime import datetime
from datetime import timedelta
import csv
import struct
import gevent
import socket
import time
import os


def init_communication(config):
    """
    Die Funktion initialisiert die Kommunikation mit dem Versuchsstand. Es werden vier Sockets für den Empfang der
    Daten erzeut sowie ein Socket für das Senden der Daten.

    :param config: Teil control_config["Addresses"]["Desktop"] aus control_config.json
    :type config: dict

    :return: Dictionary an receive-Sockets (getrennt nach float, bit und proof (z.B. für Berechnung des Scores)). Es
             gibt zwei Sockets für float, um ohne Konkurrenz den Datenstream zur Website (Index 0) und für das Erstellen
             von csv-Dateien zum Download zu ermöglichen.
    :return: Socket zum Senden von Packeten zum Versuchsstand.
    """
    sock_recv = {
        'float': [socket.socket(socket.AF_INET, socket.SOCK_DGRAM), socket.socket(socket.AF_INET, socket.SOCK_DGRAM)],
        'bit': socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
        'proof': socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    }
    sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock_recv['float'][0].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_recv['float'][0].setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1)
    sock_recv['float'][1].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_recv['float'][1].setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1)
    sock_recv['bit'].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_recv['bit'].setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1)
    sock_recv['proof'].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_recv['proof'].setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1)

    sock_recv['float'][0].bind((config["IP"], config["Float1"]["Port"]))
    sock_recv['float'][1].bind((config["IP"], config["Float2"]["Port"]))
    sock_recv['bit'].bind((config["IP"], config["Bit"]["Port"]))
    sock_recv['proof'].bind((config["IP"], config["Proof"]["Port"]))
    return sock_recv, sock_send


def send_ITP(sock, configData, configDest):
    """
    Die Funktion sendet den default-Vektor (initial targest post, itp) an den Versuchsstand. configData ist die
    Information unter ITP{1,2,3,4} in control_config.json; configDest ist die Information unter "Target".

    :param sock: socket.socket
    :param configData: Teil control_config["ITP1"] aus control_config.json
    :type config: dict
    :param configDest: Teil control_config["Addresses"]["Desktop"] aus control_config.json
    :type config: dict
    """
    sock.sendto(struct.pack('%s?' % len(configData["Bit"]), *configData["Bit"]), (
        configDest["IP"], configDest["Bit"]["Port"]))
    time.sleep(0.01)
    sock.sendto(struct.pack('%sd' % len(configData["Stream"]), *configData["Stream"]),
                (configDest["IP"], configDest["Stream"]["Port"]))
    time.sleep(0.01)
    sock.sendto(struct.pack('%sd' % len(configData["Float"]), *configData["Float"]),
                (configDest["IP"], configDest["Float"]["Port"]))
    time.sleep(0.01)
    sock.sendto(struct.pack('%sB' % len(configData["Int"]), *configData["Int"]),
                (configDest["IP"], configDest["Int"]["Port"]))

def send_Edge(sock, configData, configDest, ind, direction):
    """
    Schickt eine steigende oder fallende Flanke (Impuls) auf den Bit-Eingang des Versuchsstandes.

    :param sock: socket.socket
    :param configData: Teil control_config["ITP1"] aus control_config.json
    :type config: dict
    :param configDest: Teil control_config["Addresses"]["Desktop"] aus control_config.json
    :type config: dict
    :param ind: Index, mit Flanke
    :type ind: int
    :param direction: Richtung der Flanke
    :type direction: {rising, falling}
    """
    vals = configData["Bit"].copy()
    if direction.lower()=='rising':
        vals[ind]=True
    else:
        vals[ind] = False
    send_to_target(sock, configDest, vals, '?', 'Bit')
    time.sleep(0.1)
    send_to_target(sock, configDest, configData["Bit"], '?', 'Bit')


def send_Parametrization(sock, configDest, floats, ints):
    """
    Die Funktion schickt die Parameterierung an den Versuchsstand.

    :param sock: socket.socket
    :param configDest: Teil control_config["Addresses"]["Desktop"] aus control_config.json
    :type config: dict
    :param floats: Float-Variablen für den Versuchsstand
    :type floats: list of float
    :param ints: Int-Variablen für den Versuchsstand
    :type ints: list of int
    """
    sock.sendto(struct.pack('%sd' % len(floats), *floats),
                (configDest["IP"], configDest["Float"]["Port"]))
    time.sleep(0.01)
    sock.sendto(struct.pack('%sB' % len(ints), *ints),
                (configDest["IP"], configDest["Int"]["Port"]))


def write_target_data(filename, basedir, indices, clmnNames, length, rec_sock, stop, *args):
    """
    Die Funktion schreibt die Daten mit der gewünschten Samplezeit (vorgegeben am Anfang der Funktoin) i eine csv-Datei.

    :param filename: Name der csv-Datei, wenn leer, wird das Array im Arbeitsspeicher zurückgegeben
    :type filename: basestring
    :param basedir: Pfad der Datei (ohne subpfad exchange, der in der Funktion gesetzt wird)
    :type basedir: basestring
    :param indices: Indizes aus dem Messvektor, die gespeichert werden sollen
    :type indices : list of float
    :param clmnNames: Lister der Spaltennamen für den Header der csv
    :type clmnNames : list of basestring
    :param length: Länge des Messvektors des Versuchsstandes
    :type length : int
    :param rec_sock: Socket zum Empfang der Daten
    :tyoe rec_sock: socket.socket()
    :param stop: Art des Beenden der Aufzeichnung
    :type stop: {'extern', 'intern'}
    :param args: wenn der stop 'intern' ist, muss die Dauer der Aufzeichnung vorgegeben werden
    :type args: float
    """
    # Initialisierung
    try:
        os.remove(basedir + '/exchange/' + filename)
    except:
        pass
    sample_time = 0.01
    data_now = []
    data = []
    for index in indices:
        data_now.append(0)
    first = True
    lastT=-1
    # Wenn der Stop der Aufzeichnung extern getriggert wird (durch eine kill des aufrufenden Greenlets glet.kill())
    if stop.lower()=='extern':
        while True:
            with open((basedir + '/exchange/' + filename), 'a') as csv_clear:
                spamwriter = csv.writer(csv_clear, delimiter=';', lineterminator = '\n')
                if first:
                    data_tmp = receive_value_from_target(rec_sock, length, 'd')
                    startT = data_tmp[0]
                    spamwriter.writerow(clmnNames)
                    first = False
                data_tmp = receive_stream_from_target(rec_sock, length, 'd')
                if (data_tmp[0]-lastT)>=0.97*sample_time:
                    lastT=data_tmp[0]
                    data_now = [data_tmp[x] for x in indices]
                    data_now[0] = data_now[0]-startT
                    spamwriter.writerow(data_now)
            gevent.sleep(0.001)
    else:
        # Aufzeichnen für die Dauer, die über args übermittelt wird.
        t_end = datetime.now() + timedelta(seconds=args[0]) + timedelta(seconds=0.1)
        t_start = datetime.now()
        t_now = t_start
        data_tmp = receive_value_from_target(rec_sock, length, 'd')
        while t_now < t_end:
            data_tmp = receive_stream_from_target(rec_sock, length, 'd')
            t_now = datetime.now()
            if (data_tmp[0] - lastT) >= 0.97*sample_time:
                lastT = data_tmp[0]
                data.append([data_tmp[x] for x in indices])
            gevent.sleep(0.001)
        data.pop(0)
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

def receive_stream_from_target(rec_sock, length, dtype):
    """
    Muss zyklisch aufgerufen werden, um den Datenstream auszulesen.
    :param rec_sock: Receive-Socket
    :type re_sock: socket.socket
    :param length: Anzahl an Daten
    :type length: int
    :param dtype: Datatype
    :type dtype: char

    :return: Array der Werte
    """
    data = list(struct.unpack(str(length) + dtype, rec_sock.recv(length*8)))
    return data


def receive_value_from_target(rec_sock, length, dtype):
    """
    Liest einen konstant anliegenden Wert des Versuchsstandes aus.
    :param rec_sock: Receive-Socket
    :type re_sock: socket.socket
    :param length: Anzahl an Daten
    :type length: int
    :param dtype: Datatype
    :type dtype: char
    
    :return: Array der Werte
    """
    i = 0
    while i < 6:
        data = list(struct.unpack(str(length) + dtype, rec_sock.recv(length*8)))
        i += 1
    return data


def send_to_target(sock, configDest, data, dtype, sink):
    """
    Sendet einen konstant anliegenden Wert an den Versuchsstand.
    :param sock: Socket zum Senden
    :type sock: socket.socket
    :param configDest: Teil control_config["Addresses"]["Target"] aus control_config.json
    :type configDest: dict
    :param data: zu sendende Daten
    :param dtype: Datatapye (B,d,?)
    :tpye dtype: char
    :type data: numeric
    :param sink: Port des Versuchsstandes (Float, Int, Bit oder Stream)
    :type sink: string

    :return: Array der Werte
    """
    i = 0
    while i < 6:
        sock.sendto(struct.pack(('%s'+dtype) % len(data), *data), (
            configDest["IP"], configDest[sink]["Port"]))
        i += 1
