# pylint: disable=invalid-name
'''
Created on Jun 13, 2017

@author: broihier
'''
import math
import re
import struct
import time
import sys
import serial
import DictDatabase as dd

class SerialLink():
    '''
    Create a serial link to the GPS.
    '''

    def __init__(self, device):
        '''
        Constructor
        '''
        self.DLE = bytes([0x10]) # pylint: disable=invalid-name
        self.ETX = bytes([0x03]) # pylint: disable=invalid-name
        self.link = serial.Serial(device, timeout=5, baudrate=9600)
        self.link_protocol = "undefined"
        self.application_protocols = {}

    def send_packet(self, packet_type, packet, debug=None):
        '''
        Send packet to eTrex
        '''
        assert isinstance(bytearray(), type(packet))
        serial_layer_packet = self.wrap(packet_type, packet)
        if debug is not None:
            print("Sending:", end=" ")
            for i in serial_layer_packet:
                print("{:02x}".format(i), end=" ")
            print()
        self.link.write(serial_layer_packet)

    def read_packet(self, debug=None):
        '''
        Read packet from eTrex
        '''
        self.link.setTimeout(1)
        #read a byte - it should be a DLE
        single_byte = self.link.read()
        while single_byte != self.DLE:
            #read until DLE ETX (end of message) mark
            print("resync is necessary - discarding misaligned packet")
            while single_byte != self.DLE:
                single_byte = self.link.read()
                print('. {}'.format(single_byte))
                if single_byte == bytes():
                    return 0, bytearray()
            single_byte = self.link.read()
            if single_byte == self.ETX:
                single_byte = self.link.read() # see if next byte is DLE
            # the else of this is that the byte wasn't an ETX so not the end of a frame
        #At this point, we've read the DLE - we're going to read to the end of message
        #removing the escapes as they are observed
        message_bytes = bytearray()
        done = False
        while not done:
            single_byte = self.link.read()
            if single_byte == self.DLE:
                single_byte = self.link.read()
                if single_byte == self.ETX:
                    done = True
                #else, this should be another DLE, store it
            if not done:
                message_bytes += single_byte
            # else don't store trailing DLE and ETX

        #message_bytes should have packet_type, length of packet, packet, and then checksum
        check_sum = message_bytes.pop(-1)
        calculated_check_sum = ord(self.checksum(message_bytes))
        if check_sum != calculated_check_sum:
            print("Checksums don't match - received: {}, calculated: {}".
                  format(check_sum, calculated_check_sum))
        packet_type = message_bytes.pop(0)
        #discard length byte
        message_bytes.pop(0)
        if debug is not None:
            print("Bytes received:", end=" ")
            for i in message_bytes:
                print("{:02x}".format(i), end=" ")
            print()
        return packet_type, message_bytes


    def wrap(self, packet_type, packet):
        '''
        Wrap packet to link frame with leading DLE escaped packet bytes and then trailing DLE ETX
        '''
        length = bytes([len(packet)])
        check_sum = self.checksum(packet_type + length + packet)
        bytes_to_escape = packet_type + length + packet + check_sum
        escaped_bytes = bytearray()
        for single_byte in bytes_to_escape:
            escaped_bytes += bytes([single_byte])
            if single_byte == self.DLE:
                escaped_bytes += bytes([single_byte])
        return self.DLE + escaped_bytes + self.DLE + self.ETX

    @staticmethod
    def checksum(packet):
        '''
        Checksum a packet of bytes
        '''
        accumulator = 0
        for i in packet:
            accumulator += i
        accumulator &= 0xff
        accumulator = ((~ accumulator) + 1) & 0xff
        return bytes([accumulator])


    def set_protocol(self, protocol):
        '''
        Set the link and application protocols
        '''
        if protocol == "L001":
            self.link_protocol = L001()
        elif protocol == "A010":
            self.application_protocols["A010"] = A010()

class Capabilities():
    '''
    Capabilites of device class
    '''

    def __init__(self, link, packet):
        '''
        Constructor
        '''
        assert isinstance(bytearray(), type(packet))
        self.capability_list = []
        for i in range(0, len(packet), 3):
            name = ""
            name += chr(packet[i])
            value = packet[i+2]
            value = value << 8
            value += packet[i+1]
            name += "%3.3d" % value
            self.capability_list.append(name)

        link_protocols = []
        for capability in self.capability_list:
            if capability.startswith("L"):
                link_protocols.append(capability)
        if len(link_protocols) == 1:
            link.set_protocol(link_protocols[0])
        else:
            print("Error - more than one link protocol detected")

    def __str__(self):
        '''
        To string for class
        '''
        for i in self.capability_list:
            print(i)
        return ""

class L001():
    '''
    L001 link protocol
    '''

    Pid_Command_Data = bytes([0x0a]) #10
    Pid_Xfer_Cmplt = bytes([0x0c]) # 12
    Pid_Position_Data = bytes([0x0e]) # 14
    Pid_Date_Time_Data = bytes([0x11]) #17
    Pid_Prx_Wpt_Data = bytes([0x13]) #19
    Pid_Records = bytes([0x1b]) # 27
    Pid_Rte_Hdr = bytes([0x1d]) # 29
    Pid_Rte_Wpt_Data = bytes([0x1e]) # 30
    Pid_Almanac_Data = bytes([0x1f]) # 31
    Pid_Trk_Data = bytes([0x22]) #34
    Pid_Wpt_Data = bytes([0x23]) #35
    Pid_Pvt_Data = bytes([0x33]) #51
    Pid_Rte_Link_Data = bytes([0x62]) # 98
    Pid_Trk_Hdr = bytes([0x63]) # 99
    # remained not supported by eTrex


class A010():
    '''
    A010 application protocol
    '''

    Cmnd_Abort_Transfer = bytearray([0x00, 0x00]) # 0
    Cmnd_Transfer_Alm = bytearray([0x01, 0x00]) # 1
    Cmnd_Transfer_Posn = bytearray([0x02, 0x00]) # 2
    Cmnd_Transfer_Prx = bytearray([0x03, 0x00]) # 3
    Cmnd_Transfer_Rte = bytearray([0x04, 0x00]) # 4
    Cmnd_Transfer_Time = bytearray([0x05, 0x00]) # 5
    Cmnd_Transfer_Trk = bytearray([0x06, 0x00]) # 6
    Cmnd_Transfer_Wpt = bytearray([0x07, 0x00]) # 7
    Cmnd_Turn_Off_Pwr = bytearray([0x08, 0x00]) # 8
    Cmnd_Start_Pvt_Data = bytearray([0x31, 0x00]) # 49
    Cmnd_Stop_Pvt_Data = bytearray([0x32, 0x00]) # 50

    @staticmethod
    def get_string(packet):
        '''
        Get string from packet
        '''
        local_copy = packet.copy()
        local_copy.pop(0)
        local_copy.pop(0)
        string = local_copy.decode(encoding="utf_8")
        return string

    @staticmethod
    def get_count(packet):
        '''
        Get number of packets to follow
        '''
        count = 0
        count |= (packet[1] << 8) + packet[0]
        return count

class Position():
    '''
    Earth relative position
    '''
    def __init__(self, position_bytes=None, latitude=None, longitude=None):
        '''
        Constructor
        '''
        self.latitude = 0.0
        self.longitude = 0.0
        if (position_bytes is not None) and ((latitude is not None) or (longitude is not None)):
            print("Error in Position constructor - operations are mutually exclusive")
            raise ValueError
        if latitude is not None:
            self.latitude = latitude
        if longitude is not None:
            self.longitude = longitude
        if position_bytes is not None:
            self.bytes_to_position(position_bytes)
        else:
            self.position_to_bytes()


    def bytes_to_position(self, position_bytes):
        '''
        Parse bytes into earth relative position.
        '''
        latitude = 0
        if (position_bytes[3] & 0x80) != 0:
            latitude = (-1) ^ 0x7fffffff
        latitude |= ((position_bytes[3] << 24) | (position_bytes[2] << 16)
                     | (position_bytes[1] << 8)
                     | position_bytes[0])
        self.latitude = latitude * 180.0 / (math.pow(2.0, 31))
        longitude = 0
        if (position_bytes[7] & 0x80) != 0:
            longitude = (-1) ^ 0x7fffffff
        longitude |= ((position_bytes[7] << 24) | (position_bytes[6] << 16)
                      | (position_bytes[5] << 8)
                      | position_bytes[4])
        self.longitude = longitude * 180.0 / (math.pow(2.0, 31))

    def position_to_bytes(self):
        '''
        Parse floating point position to bytes.
        '''
        latitude = int(self.latitude * math.pow(2.0, 31) / 180.0) & 0xffffffff
        longitude = int(self.longitude * math.pow(2.0, 31) / 180.0) & 0xffffffff

        self.position_bytes = bytearray()
        for i in range(4):
            self.position_bytes += bytes([(latitude >> (8 * i)) & 0xff])
        for i in range(4):
            self.position_bytes += bytes([(longitude >> (8 * i)) & 0xff])

    def __str__(self):
        '''
        Class description.
        '''
        return "latitude {} {}, longitude {} {}".format("S" if self.latitude < 0.0 else "N",
                                                        abs(self.latitude),
                                                        "W" if self.longitude < 0.0 else "E",
                                                        abs(self.longitude))


class Track():
    '''
    Track class - name and sequence of positions.
    '''

    offset = time.mktime(time.strptime("Dec 31 89", "%b %d %y"))

    def __init__(self):
        '''
        Constructor
        '''
        self.name = ""
        self.positions = []
        self.position_time = []
        self.altitude = []
        self.raw = []

    def add_position(self, position):
        '''
        Add a poition to the track.
        '''
        assert isinstance(Position(), type(position))
        self.positions.append(position)

    def set_name(self, name):
        '''
        Format and set the name of the track.
        '''
        self.name = "T" + name.strip('\0x00').replace("-", "_").replace(" ", "_") # normalize

    def __str__(self):
        '''
        Class description
        '''
        print(self.name)
        for index, position in enumerate(self.positions):
            if (self.position_time[index] ^ 0xffffffff) == 0:
                print("{}, time = invalid, altitude = {} meters".format(position,
                                                                        self.altitude[index]))
            else:
                print("{}, time = {}, altitude = {} meters".
                      format(position, time.ctime(self.position_time[index] + self.offset),
                             self.altitude[index]))
        return ""


    def decode(self, track_bytes):
        '''
        Decode a A010, track from eTrex into a position.
        '''
        assert isinstance(bytearray(), type(track_bytes))
        self.raw.append(track_bytes.copy())
        self.add_position(Position(position_bytes=track_bytes[0:8]))
        i = 0
        for index, single_byte in enumerate(track_bytes[8:12]):
            i |= single_byte << (8 * index)
        self.position_time.append(i)
        self.altitude.append(struct.unpack('f', track_bytes[12:16])[0])

class Waypoint(object):
    '''
    Waypoint class - name and position
    '''

    offset = time.mktime(time.strptime("Dec 31 89", "%b %d %y"))

    def __init__(self, name=None, latitude=None, longitude=None):
        '''
        Constructor
        '''
        if name is None:
            self.name = ""
            self.position = Position()
            self.altitude = 0.0
            self.raw = []
        else:
            self.name = name
            self.position = Position(latitude=latitude, longitude=longitude)
            self.altitude = 0.0
            self.encode()

    def __str__(self):
        '''
        Class description
        '''
        print(self.name)
        print("{}, time = invalid, altitude = {} meters".format(self.position, self.altitude))
        return ""

    def decode(self, waypoint_bytes):
        '''
        Decode a A100, D108 waypoint from eTrex into a Waypoint object.
        '''
        assert isinstance(bytearray(), type(waypoint_bytes))
        self.raw.append(waypoint_bytes.copy())
        self.position = Position(position_bytes=waypoint_bytes[24:32])
        self.altitude = struct.unpack('f', waypoint_bytes[32:36])
        self.name = waypoint_bytes[48:].decode(encoding="utf_8").strip('\0x00')

    def encode(self):
        '''
        Encode a D108 waypoint from instance variables.
        '''
        self.raw = bytearray()
        self.raw += bytes([0x00]) # class - user
        self.raw += bytes([0xff]) # color - default
        self.raw += bytes([0x00]) # display - with name
        self.raw += bytes([0x60]) # attributes (always 0x60
        self.raw += bytearray([0xb2, 0x00]) # flag
        self.raw += bytearray([0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00,
                               0xff, 0xff, 0xff, 0xff,
                               0xff, 0xff, 0xff, 0xff,
                               0xff, 0xff, 0xff, 0xff]
                             ) # subclass - not a map waypoint
        self.raw += self.position.position_bytes
        self.raw += struct.pack("<f", self.altitude) # altitude
        self.raw += struct.pack("<f", 0.0) # depth
        self.raw += struct.pack("<f", 0.0) # proximity
        self.raw += bytearray([0x20, 0x20]) # state
        self.raw += bytearray([0x20, 0x20]) # country code
        self.raw += self.name.encode(encoding="utf_8") # waypoint identity
        self.raw += bytes([0x00]) # terminate waypoint identity
        self.raw += bytes([0x00]) # terminate comment
        self.raw += bytes([0x00]) # terminate facility name
        self.raw += bytes([0x00]) # terminate city name
        self.raw += bytes([0x00]) # terminate address
        self.raw += bytes([0x00]) # terminate intersecting road label


class eTrex(object):
    '''
    eTrex GPS device.
    '''

    @staticmethod
    def get_tracks(link, tracks):
        '''
        Get tracks from eTrex
        '''
        track_file = -1
        number_of_records = 0
        observed_records = 0
        link.send_packet(L001.Pid_Command_Data, A010.Cmnd_Transfer_Trk)
        ptype, data = link.read_packet() # read Ack
        print("Downloading Tracks")
        while data: # length of data is not zero
            ptype, data = link.read_packet() # get next packet
            link.send_packet(bytes([0x06]), bytearray([ptype, 0x00])) #send Ack
            if ptype == ord(L001.Pid_Trk_Hdr):
                tracks.append(Track())
                track_file = track_file + 1
                tracks[track_file].set_name(A010.get_string(data))
                observed_records = observed_records + 1
            elif ptype == ord(L001.Pid_Trk_Data):
                tracks[track_file].decode(data)
                observed_records = observed_records + 1
            elif ptype == ord(L001.Pid_Records):
                number_of_records = A010.get_count(data)
                observed_records = 0
            elif ptype == ord(L001.Pid_Xfer_Cmplt):
                if number_of_records != observed_records:
                    print("Unexpected number of record - expected: {}, saw: {}".
                          format(number_of_records, observed_records))
                else:
                    break
            else:
                print("Undexpected ptype: {}".format(ptype))
        print("{} total track logs collected".format(len(tracks)))

    @staticmethod
    def get_waypoints(link, waypoints):
        '''
        Get waypoints from eTrex
        '''
        waypoint_file = -1
        number_of_records = 0
        observed_records = 0
        link.send_packet(L001.Pid_Command_Data, A010.Cmnd_Transfer_Wpt)
        ptype, data = link.read_packet() # read Ack
        print("Downloading Waypoints")
        while data: # length of data is not zero
            ptype, data = link.read_packet() # get next packet
            link.send_packet(bytes([0x06]), bytearray([ptype, 0x00])) #send Ack
            if ptype == ord(L001.Pid_Wpt_Data):
                waypoints.append(Waypoint())
                waypoint_file = waypoint_file + 1
                waypoints[waypoint_file].decode(data)
                observed_records = observed_records + 1
            elif ptype == ord(L001.Pid_Records):
                number_of_records = A010.get_count(data)
                observed_records = 0
            elif ptype == ord(L001.Pid_Xfer_Cmplt):
                if number_of_records != observed_records:
                    print("Unexpected number of record - expected: {}, saw: {}".
                          format(number_of_records, observed_records))
                else:
                    break
            else:
                print("Undexpected ptype: {}".format(ptype))
        print("{} total waypoints collected".format(len(waypoints)))

    @staticmethod
    def write_tracks_to_database(tracks, database):
        '''
        Write tracks to the sqlite3 database.
        '''
        print("Track files that will be created or overwritten:")
        for i in tracks:
            print("{}".format(i.name))
        print("writing tracks to sqlite3 database")
        #device_database.drop("track_names")
        database.create(dict(name="track_names", fields="name text"))
        for i in tracks:
            # create track table
            database.drop(i.name) # purge old copy
            database.create(dict(name=i.name, fields="latitude float, longitude float"))
            for j in i.positions:
                database.insert(dict(name=i.name, fieldNames="latitude,longitude",
                                     values="{},{}".format(j.latitude, j.longitude)),
                                buffer=True)
            #Note: these SQL commands will commit the buffered inserts from above
            database.delete(dict(name="track_names", field="name",
                                 value="\"" + i.name + "\""))
            database.insert(dict(name="track_names", fieldNames="name",
                                 values="\"" + i.name + "\""))

    @staticmethod
    def write_waypoints_to_database(waypoints, database):
        '''
        Write waypoints to the sqlite3 database.
        '''
        print("Waypoints that will be created or overwritten:")
        for i in waypoints:
            print(i)
        print("writing waypoints to sqlite3 database")

        database.create(dict(name="waypoints",
                             fields="name text, latitude float, longitude float"))
        for i in waypoints:
            # update waypoint, insert if the update fails
            if database.update(dict(name="waypoints", fieldName="latitude",
                                    value="{}".format(i.position.latitude),
                                    selector="name", selectorValue="\"" + i.name + "\""),
                              ):
                database.insert(dict(name="waypoints", fieldNames="name,latitude,longitude",
                                     values="{},{},{}".
                                     format("\"" + i.name + "\"", i.position.latitude,
                                            i.position.longitude)),
                               )
            else:
                database.update(dict(name="waypoints", fieldName="longitude",
                                     value="{}".format(i.position.longitude),
                                     selector="name", selectorValue="\"" + i.name + "\""),
                               )

    @staticmethod
    def dump():
        '''
        Read the eTrex device and dump it to the database.
        '''
        device_database = dd.DictDatabase("eTrex.db")
        link = SerialLink("/dev/ttyUSB0")
        link.send_packet(bytes([0xfe]), bytearray())
        ptype, data = link.read_packet() #get ack

        ptype, data = link.read_packet() # get response

        link.send_packet(bytes([0x06]), bytearray([ptype, 0x00])) # send ack

        ptype, data = link.read_packet() # get response
        link.send_packet(bytes([0x06]), bytearray([ptype, 0x00]))

        capabilities = Capabilities(link, data)

        print("Connection made to eTrex device and capabilities received:")
        print(capabilities)

        tracks = []
        waypoints = []
        eTrex.get_tracks(link, tracks)
        eTrex.get_waypoints(link, waypoints)
        eTrex.write_tracks_to_database(tracks, device_database)
        eTrex.write_waypoints_to_database(waypoints, device_database)

        device_database.close()

    @staticmethod
    def load_waypoint(line):
        '''
        Load a waypoint into the eTrex
        '''
        (name, latitude, longitude) = re.split(", +", line)[1:]

        waypoint_to_insert = Waypoint(name=name,
                                      latitude=float(latitude),
                                      longitude=float(longitude))

        # connect to eTrex
        link = SerialLink("/dev/ttyUSB0")
        link.send_packet(bytes([0xfe]), bytearray())
        link.read_packet() #get ack

        ptype = link.read_packet()[0] # get response

        link.send_packet(bytes([0x06]), bytearray([ptype, 0x00])) # send ack

        ptype, data = link.read_packet() # get response
        link.send_packet(bytes([0x06]), bytearray([ptype, 0x00]))

        capabilities = Capabilities(link, data)
        print("Connect to device made, capabilities of device are:")
        print(capabilities)

        link.send_packet(L001.Pid_Records, bytearray([0x01, 0x00])) # transferring records (1)
        ptype, data = link.read_packet() # read Ack
        ack_message = "Pid_Records sent, " +\
            "ack received for {}" if ptype == 0x06 else "nak received for {}"
        print(ack_message.format(data[0]))

        link.send_packet(L001.Pid_Wpt_Data, waypoint_to_insert.raw)
        ptype, data = link.read_packet() # read Ack
        ack_message = "Waypoint sent, " +\
            "ack received for {}" if ptype == 0x06 else "nak received for {}"
        print(ack_message.format(data[0]))

        link.send_packet(L001.Pid_Xfer_Cmplt, A010.Cmnd_Transfer_Wpt) # transfer waypoint complete
        ptype, data = link.read_packet() # read Ack
        ack_message = "Transfer complete sent, " +\
            "ack received for {}" if ptype == 0x06 else "nak received for {}"
        print(ack_message.format(data[0]))

        waypoints = []
        eTrex.get_waypoints(link, waypoints)
        print("New List of Waypoints:")
        for i in waypoints:
            print(i.name)
            print(i.position)

    @staticmethod
    def load_track(name, file):
        '''
        Load a track into the eTrex
        '''
        pass

    @staticmethod
    def load(file_name):
        '''
        Load some sort of information into the eTrex.
        '''
        try:
            file = open(str(file_name))
            line = file.readline()
            if line == "":
                print("End of file")
                print("Terminating with no action taken")
                sys.exit(-1)
        except FileNotFoundError as error:
            print(error)
            sys.exit(-1)
        (parameter_type, name) = line.strip().split(",")[0:2]
        if parameter_type == "waypoint":
            eTrex.load_waypoint(line)
        elif parameter_type == "track":
            eTrex.load_track(name, file)
        else:
            print("Error in type of load - terminating with no action taken - type was: {}".
                  format(parameter_type))

def main():
    '''
    main program
    '''
    if len(sys.argv) == 1:
        eTrex.dump()
    else:
        eTrex.load(sys.argv[1])

if __name__ == "__main__":
    main()
