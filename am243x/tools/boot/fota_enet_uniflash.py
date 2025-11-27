
import os
import sys
import time
import math
import socket
import argparse
import struct
import ipaddress
import shlex

try:
    from tqdm import tqdm
except ImportError:
    print('[ERROR] Dependent modules not installed, use below pip command to install them. Ensure proxy for pip is setup if needed.')
    print()
    print('python -m pip install tqdm --proxy={http://your proxy server:port or leave blank if no proxy}')
    sys.exit(2)

BOOTLOADER_UNIFLASH_PACKET_SIZE                      = 1464
BOOTLOADER_UNIFLASH_SEQNUM_SIZE                      = 4
BOOTLOADER_UNIFLASH_MGCNUM_SIZE                      = 4
BOOTLOADER_UNIFLASH_ERASESIZE_SIZE                   = 4
BOOTLOADER_UNIFLASH_PAYLOAD_SIZE                     = BOOTLOADER_UNIFLASH_PACKET_SIZE + BOOTLOADER_UNIFLASH_SEQNUM_SIZE + BOOTLOADER_UNIFLASH_MGCNUM_SIZE
BOOTLOADER_UNIFLASH_BUF_SIZE                         = 1024*1024 # 1 MB This has to be a 256 KB aligned value, because flash writes will be block oriented
BOOTLOADER_UNIFLASH_HEADER_SIZE                      = 32 # 32 B
BOOTLOADER_UNIFLASH_HEADER_PKT_SIZE                  = BOOTLOADER_UNIFLASH_HEADER_SIZE + BOOTLOADER_UNIFLASH_SEQNUM_SIZE + BOOTLOADER_UNIFLASH_MGCNUM_SIZE

BOOTLOADER_UNIFLASH_MAX_FILE_PKT_NUM                 = math.floor((BOOTLOADER_UNIFLASH_BUF_SIZE - BOOTLOADER_UNIFLASH_HEADER_PKT_SIZE)/BOOTLOADER_UNIFLASH_PAYLOAD_SIZE)

BOOTLOADER_UNIFLASH_PKT_MAGIC_NUMBER                 = 0x05B1C00D # SBL GOOD
BOOTLOADER_UNIFLASH_PKT_ACK                          = 0x05B10ACD # SBL AC(K)D
BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER         = bytearray([0x42, 0x4C ,0x55, 0x46]) # BLUF
BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER         = 0x52554C42 # BLUR

ENET_START_FRAME                                      = 0xAACBEDFA

DEADBABE = bytearray([0xBE, 0xBA, 0xAD, 0xDE])

bar_cursize = 0
bar_filesize = 0

class LineCfg():
    def __init__(self, line=None, filename=None):
        self.line = line
        self.filename = filename
        self.exit_now = False

    # Takes a string of comma separated key=value pairs and parses into a dictionary
    def parse_to_dict(self, config_string):
        config_dict = dict()
        splitter = shlex.shlex(config_string, posix=True)
        splitter.commenters="#"
        splitter.whitespace = ' '
        splitter.whitespace_split = True

        for key_value_pair in splitter:
            kv = key_value_pair.strip()
            if not kv:
                continue
            kv_t = kv.split('=', 1)
            if(len(kv_t)==1):
                #error, no value
                pass
            else:
                config_dict[kv_t[0]] = kv_t[1]

        return config_dict

    # Validate the configuration
    def validate(self):
        status = 0
        if(self.line!=None):
            config_dict = self.parse_to_dict(self.line)
            if not config_dict:
                status = "invalid_line"
            else:
                if "--file" not in config_dict.keys():
                    status = "[ERROR] no filename provided !!!"
                    return status
                else:
                    self.filename = config_dict["--file"]

                try:
                    f = open(self.filename)
                except FileNotFoundError:
                    status = "[ERROR] File not found !!!"
                    return status
                    
        else:
            # Should not hit this
            status = -1
        return status

class FileCfg():
    def __init__(self, filename=None):
        self.filename = filename
        self.lines = list()
        self.cfgs = list()
        self.errors = list()

    def parse(self):
        f = open(self.filename, "r")
        lines = f.readlines()
        f.close()
        parse_status = 0
        linecount = 0
        valid_cfg_count = 0

        for line in lines:
            linecfg = LineCfg(line=line)
            status = linecfg.validate()

            if(status != 0 and status != "invalid_line"):
                self.errors.append("[ERROR] Parsing error found on line {} of {}\n{}\n".format(linecount+1, self.filename, status))
            else:
                if(status != "invalid_line"):
                    self.cfgs.append(linecfg)
                    self.lines.append(line.rstrip('\n'))
                    valid_cfg_count += 1
            linecount += 1

        if(len(self.errors) > 0):
            # Some commands have errors
            parse_status = "Parsing config file ... ERROR. {} error(s).\n\n".format(len(self.errors)) + "\n".join(self.errors)

        return parse_status

# Parse and validate input arguments, config file
def parse():
    parser = argparse.ArgumentParser(description="Helper script to upload appimage files via ethernet to AWR2944EVM",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--hostIP",
                        metavar='ip',
                        type=str,
                        help="Host PC IP address",
                        default="192.168.0.193")
    parser.add_argument("--hostPort",
                        metavar='n',
                        type=int,
                        help="Host PC UDP Port number",
                        default=5001)
    parser.add_argument("--boardIP",
                        metavar='ip',
                        type=str,
                        help="EVM IP address",
                        default="192.168.0.195")
    parser.add_argument("--boardPort",
                        metavar='n',
                        type=int,
                        help="EVM UDP Port number",
                        default=5001)
    parser.add_argument("--debug", "-d",
                        action="store_true",
                        help="Print debug statements")
    parser.add_argument("--cfg",
                        metavar="myconfig.cfg",
                        type=str,
                        help="Path to config file",
                        default="default_sbl_eth.cfg")
    args = parser.parse_args()

    try:
        hostIP = ipaddress.ip_address(args.hostIP)
    except ValueError:
        print("[ERROR] Bad Host IP Address entered")
        sys.exit(1)
    try:
        boardIP = ipaddress.ip_address(args.boardIP)
    except ValueError:
        print("[ERROR] Bad EVM IP Address entered")
        sys.exit(1)

    if not os.path.exists(args.cfg):
        print("[ERROR] Config file not found")
        sys.exit(3)
    else:
        print("[LOG] Parsing config file ...")
        filecfg = FileCfg(args.cfg)
        parse_status = filecfg.parse()
        if(parse_status != 0):
            print(parse_status)
            sys.exit()
        else:

            # No errors, can proceed to flash
            print("[LOG] Found {} command(s) !!!".format(len(filecfg.cfgs)))
    return args, filecfg

# Make custom payloads from given file by splitting into multiple chunks
def packetize(linecfg: LineCfg, debug: bool):
    total_packet_list = []
    total_size = 0
    filesize = 0

    filesize = os.path.getsize(linecfg.filename)
    total_num_packets = math.ceil(filesize/BOOTLOADER_UNIFLASH_PACKET_SIZE)
    num_files = math.ceil(total_num_packets/BOOTLOADER_UNIFLASH_MAX_FILE_PKT_NUM)
    with open(linecfg.filename, 'rb') as f:
        file_bytes = bytes(f.read())
    
    last_file_size = total_num_packets - ((num_files-1)*BOOTLOADER_UNIFLASH_MAX_FILE_PKT_NUM)
    file_sizes = [BOOTLOADER_UNIFLASH_MAX_FILE_PKT_NUM]*(num_files-1)
    file_sizes.append(last_file_size)

    for i in range(num_files):
        file_packet_list = []

        # Add magic number to differentiate from other packets
        # Add SEQ number
        # Add Uniflash header
        # Fill total number of packets in rsv1 of header
        data_bytes = bytearray(BOOTLOADER_UNIFLASH_PKT_MAGIC_NUMBER.to_bytes(length=BOOTLOADER_UNIFLASH_MGCNUM_SIZE, byteorder='little', signed=False))
        data_bytes += bytearray(BOOTLOADER_UNIFLASH_SEQNUM_SIZE)
        data_bytes += BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER
        data_bytes += DEADBABE
        data_bytes += DEADBABE
        data_bytes += DEADBABE
        total_size = filesize + (total_num_packets*8) + 32
        data_bytes += bytearray(total_size.to_bytes(length=BOOTLOADER_UNIFLASH_ERASESIZE_SIZE, byteorder='little', signed=False))
        # reserved bytes in uniflash header
        # fill partial file packet count in rsv1 of uniflash header
        data_bytes += bytearray((file_sizes[i]+1).to_bytes(length=BOOTLOADER_UNIFLASH_SEQNUM_SIZE, byteorder='little', signed=False))
        data_bytes += DEADBABE
        data_bytes += DEADBABE
        if debug:
            print("[DEBUG] Part {} Packet {}: ".format(i, 0))
            print(data_bytes.hex())
        file_packet_list.extend([data_bytes])

        for j in range(file_sizes[i]):
            # Add magic number to differentiate from other packets
            # Add SEQ number
            # Add actual data from file
            data_bytes = bytearray(BOOTLOADER_UNIFLASH_PKT_MAGIC_NUMBER.to_bytes(length=BOOTLOADER_UNIFLASH_MGCNUM_SIZE, byteorder='little', signed=False))
            data_bytes += bytearray((j+1).to_bytes(length=BOOTLOADER_UNIFLASH_SEQNUM_SIZE, byteorder='little', signed=False))
            start = (i*BOOTLOADER_UNIFLASH_BUF_SIZE) + (j*BOOTLOADER_UNIFLASH_PACKET_SIZE)
            end = start + BOOTLOADER_UNIFLASH_PACKET_SIZE
            if end >= len(file_bytes):
                end = None
            data_bytes += file_bytes[start:end]
            if debug:
                print("[DEBUG] Part {} Packet {}: ".format(i, j+1))
                print(data_bytes.hex())
            file_packet_list.extend([data_bytes])
        total_packet_list.append(file_packet_list)
    total_size = filesize + (total_num_packets*8) + 32
    return total_size, total_num_packets, total_packet_list

# Setup UDP socket
def setup(bind_ip: str, bind_port: int):
    print("[LOG] Creating socket")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.settimeout(5)
    sock.bind((bind_ip, bind_port))
    return sock

# Perform cleanup functions - close socket
def cleanup(sock: socket.SocketType):
    print("[LOG] Closing socket")
    sock.close()

# Transfer with Stop-and-Wait ARQ protocol
def transceive(sock: socket.SocketType, payloads: list, num_packets: int, transmit_ip: str, transmit_port: int, filename: str, total_size: int, debug: bool=False):
    flashed = False
    bar = tqdm(total=total_size, unit="packets", leave=False, desc='Sending {}'.format(filename))
    tstart = time.time()
    for i in range(num_packets):
        sock.sendto(payloads[i], (transmit_ip, transmit_port))
        ackd = False
        pkt_timeout_count = 0
        while not ackd:
            try:
                data, addr = sock.recvfrom(8)
                code = struct.unpack('<II', data)
                if addr[0] == transmit_ip:
                    if (code[0] == i) and (code[1] == BOOTLOADER_UNIFLASH_PKT_ACK):
                        ackd = True
                        if debug:
                            print("[TRANSFER] (SUCCESS) ACK received for packet {}".format(i))
                        bar.update(len(payloads[i]))
                        bar.refresh()
            except socket.timeout:
                pkt_timeout_count += 1
                if debug:
                    print("[TRANSFER] (TIMEOUT) ACK timed out for packet {}".format(i))
                if pkt_timeout_count == 5:
                    print("[ERROR] Connection timed out too many times for same packet. Check connection to EVM.")
                    print("        Power cycle EVM and run this script again !!!")
                    bar.close()
                    sock.close()
                    sys.exit(6)
                sock.sendto(payloads[i], (transmit_ip, transmit_port))
    tstop = time.time()
    timetaken = round(tstop-tstart, 2)

    bar.close()
    return timetaken

addr="test"
if __name__ == "__main__":
    args, filecfg = parse()
    sock = setup(args.hostIP, args.hostPort)
    try:
        # Wait for EVM linkup
        linkup = False
        linkup_timeout_count = 0
        while linkup is False:
            try:
                print("Starting Linkup ...")
                data, addr = sock.recvfrom(8)

                print("Received.")
                print(addr)
                code = struct.unpack('<II', data)
                if addr[0] == args.boardIP:
                    if code[0] == BOOTLOADER_UNIFLASH_PKT_MAGIC_NUMBER:
                        linkup = True
                        if code[1] == BOOTLOADER_UNIFLASH_PKT_ACK:
                            print("")
                            print("[LINKUP] (SUCCESS) EVM Linked up. Starting transfer ...")
                            print("")
            # linkup might timeout 1 or 2 times due to the small socket timeout, ignore this.
            # if more than 5 times, then something seriously wrong
            except socket.timeout:
                linkup_timeout_count += 1
                if linkup_timeout_count == 5:
                    print("[ERROR] Connection timed out too many times for link-up. Check connection to EVM.")
                    print("        Power cycle EVM and run this script again !!!")
                    sock.close()
                    sys.exit(3)

        # Send each file
        for index in range(len(filecfg.cfgs)):
            total_size, num_packets, total_packet_list = packetize(filecfg.cfgs[index], args.debug)
            total_time = 0
            for partial_file in total_packet_list:
                part_time = transceive(sock, partial_file, len(partial_file), args.boardIP, args.boardPort, filecfg.cfgs[index].filename, total_size)
                total_time += part_time
        print("[LOG] All commands from config file are executed !!!")

    except KeyboardInterrupt:
        tqdm.close()
        print("[ERROR] Operation cancelled by user")
        print("        Power cycle EVM and run this script again !!!")
        sys.exit(8)