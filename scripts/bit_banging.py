import struct
import math
import numpy as np
from matplotlib import pyplot as plt

scan_print = """96 5 0 0 8 0 -81 -2 4 21 2 -56 3 0 0 0 -92 122 4 0 4 9 -118 0 -20 8 -119 0 -41 8 -120 0 -70 8 -123 0 -82 8 -118 0 -107 8 -118 0 116 8 -120 0 -117 8 120 0 88 8 112 0 73 8 119 0 47 8 116 0 24 8 121 0 7 8 119 0 -1 7 119 0 -9 7 118 0 -38 7 119 0 -38 7 119 0 -54 7 120 0 -63 7 120 0 -87 7 122 0 -80 7 116 0 -105 7 116 0 -112 7 110 0 -23 3 101 0 -11 3 101 0 110 7 109 0 33 5 106 0 24 5 54 0 -128 7 110 0 -53 7 108 0 63 8 112 0 -72 8 110 0 72 9 110 0 26 18 121 0 5 18 125 0 -124 12 111 0 0 6 90 0 74 6 -100 0 26 6 -86 0 -8 6 126 0 -4 6 120 0 -8 6 118 0 -12 6 124 0 -7 6 -127 0 119 6 115 0 119 6 114 0 111 6 113 0 -107 0 73 0 -105 0 96 0 -94 0 98 0 -95 0 92 0 -107 0 68 0 -121 0 35 0 58 6 114 0 45 6 115 0 42 6 118 0 37 6 114 0 37 6 115 0 27 6 111 0 44 6 108 0 -10 6 101 0 36 7 115 0 32 7 114 0 -80 0 20 0 -43 0 29 0 -105 0 21 0 63 7 91 0 57 7 90 0 -74 0 60 0 -100 0 80 0 116 0 76 0 -105 0 93 0 -118 0 74 0 -125 0 80 0 -104 0 105 0 -111 0 112 0 -124 0 -126 0 -114 0 -102 0 -122 0 -100 0 -111 0 -116 0 -111 0 112 0 -117 0 102 0 -97 0 85 0 -112 0 70 0 -4 7 67 0 8 8 83 0 30 8 84 0 73 8 119 0 85 8 123 0 110 8 121 0 38 7 111 0 -105 8 121 0 -73 8 116 0 -55 8 120 0 -1 -1 0 0 -1 -1 0 0 -66 2 82 0 -75 2 79 0 -87 2 90 0 -84 2 98 0 -82 2 83 0 -76 2 84 0 -54 6 66 0 -1 -1 0 0 -121 2 55 0 -95 2 69 0 -127 2 70 0 -105 2 73 0 -114 2 65 0 -1 -1 0 0 77 32 34 0 -118 23 101 0 -1 -1 0 0 93 15 60 0 60 3 103 0 71 3 94 0 115 3 31 0 -84 14 98 0 -113 14 101 0 109 14 94 0 125 14 94 0 -41 14 96 0 84 15 -119 0 60 15 -118 0 23 15 -113 0 -124 15 120 0 -17 15 119 0 90 16 119 0 -51 16 118 0 56 17 107 0 64 29 105 0 45 52 91 0 57 28 102 0 67 28 68 0 96 26 56 0 89 26 92 0 62 26 85 0 35 26 81 0 -44 26 86 0 -1 -1 0 0 -104 25 68 0 -119 28 103 0 1 32 112 0 -23 31 114 0 -53 25 114 0 -25 26 30 5 -106 47 106 0 112 25 114 0 -118 49 94 0 -6 48 36 0 -26 48 108 0 -1 -1 0 0 -104 43 26 0 52 49 102 0 110 44 83 0 119 49 103 0 71 48 74 0 88 58 102 0 -25 68 101 0 -25 68 103 0 -29 68 98 0 -68 33 54 0 -6 32 110 0 -19 16 116 0 19 16 116 0 82 15 125 0 2 15 -121 0 2 15 -116 0 23 15 -121 0 63 15 -124 0 60 15 -120 0 87 15 -124 0 109 15 -123 0 124 15 -127 0 -41 15 -125 0 -8 15 -127 0 21 16 -124 0 33 16 -126 0 58 16 -125 0 86 16 -125 0 119 16 -128 0 -112 16 -128 0 -83 16 -126 0 -50 16 -128 0 -22 16 -125 0 104 17 114 0 0 17 -123 0 56 16 114 0 -84 15 114 0 -94 15 -112 0 -73 15 -112 0 -48 15 -111 0 -28 15 -114 0 1 16 -122 0 -43 15 109 0 78 15 107 0 114 15 74 0 -20 1 96 0 -30 1 108 0 -50 1 112 0 -52 1 110 0 -56 1 110 0 -70 1 114 0 -70 1 115 0 -81 1 117 0 -81 1 119 0 -70 1 116 0 -69 1 117 0 -85 1 119 0 -61 1 116 0 -66 1 116 0 -65 1 121 0 -52 1 118 0 -64 1 110 0 -71 1 112 0 -76 1 108 0 -88 1 111 0 -81 1 110 0 -88 1 111 0 -91 1 112 0 -85 1 110 0 -103 1 113 0 -103 1 114 0 -91 1 112 0 -86 1 109 0 -103 1 115 0 -102 1 118 0 -88 1 111 0 -94 1 114 0 -86 1 114 0 -79 1 112 0 -84 1 111 0 -68 1 111 0 -64 1 110 0 -55 1 111 0 -32 1 110 0 -5 1 106 0 -11 1 75 0 85 4 115 0 96 4 106 0 90 3 -103 0 99 3 110 0 39 3 100 0 40 3 -99 0 -83 3 127 0 109 4 116 0 54 4 111 0 -98 4 112 0 -98 4 106 0 78 4 105 0 93 4 109 0 32 4 120 0 -71 4 108 0 -46 4 111 0 -11 4 104 0 7 5 111 0 -54 4 108 0 -28 4 115 0 -101 4 104 0 88 2 24 0 -53 4 41 0 49 2 90 0 45 2 114 0 44 2 -121 0 -71 1 60 0 -118 1 80 0 -118 1 80 0 120 1 84 0 123 1 86 0 108 1 98 0 105 1 105 0 110 1 105 0 109 1 104 0 94 1 108 0 105 1 105 0 116 1 106 0 109 1 104 0 110 1 105 0 116 1 103 0 120 1 101 0 -116 1 95 0 -116 1 89 0 -114 1 72 0 -106 1 69 0 -63 6 115 0 -65 6 -128 0 -52 6 -125 0 -44 6 -125 0 -39 6 -122 0 -32 6 -124 0 -35 6 -123 0 -27 6 -122 0 -8 6 126 0 16 7 113 0 76 2 97 0 9 2 125 0 -60 1 105 0 -46 1 101 0 37 2 103 0 42 2 104 0 91 7 123 0 82 7 124 0 114 7 114 0 125 7 108 0 -111 7 108 0 -77 1 99 0 -66 1 103 0 -32 1 99 0 22 2 102 0 18 2 102 0 10 2 105 0 1 2 107 0 -3 1 105 0 -5 1 100 0 -11 1 105 0 -17 1 106 0 -19 1 105 0 26 1 -97 0 -7 0 -103 0 -32 0 116 0 -23 0 113 0 -11 0 116 0 15 1 118 0 33 1 -116 0 0 1 -110 0 25 1 -116 0 -114 1 110 0 -102 1 124 0 -33 1 113 0 9 2 104 0 21 2 96 0 17 2 96 0 14 2 91 0 14 2 94 0 59 2 69 0 67 1 74 0 74 1 81 0 62 1 60 0 109 1 24 0"""
info_print = """48 0 0 0 8 0 -81 -2 4 21 2 -56 1 0 0 0 -18 62 5 0 1 1 0 -128 0 0 16 -128 -18 62 5 0 -64 17 0 -16 0 0 0 0 0 0 -117 10 8 0 0 0"""

def parse_string(string):
    ints = [int(x) for x in string.split(" ") if len(x) > 0]
    for index in range(len(ints)):
        if ints[index] < 0:
            ints[index] += 0x100
    byte_data = bytes(ints)
    return byte_data

def display_frame(data_bytes):
    frame_data = struct.unpack('<IIIHHI', data_bytes[0:20])
    length = frame_data[0]
    frame_id = frame_data[3]
    block = frame_data[4]
    scan_num = frame_data[5]
    print("Data length:", length)
    print("Array length:", len(info_bytes))
    print("ID:", frame_id)
    print("Block:", block)
    print("Scan number:", scan_num)
    return length, frame_id, block, scan_num

def display_contour(data_bytes):
    contour_data = struct.unpack('<HHHH', data_bytes[40:48])
    start_index, stop_index, interval = contour_data[0:3]
    num_beams = 1 + int(math.ceil(stop_index - start_index) / interval)
    print("Start index:", start_index)
    print("Stop index:", stop_index)
    print("Index interval:", interval)
    print("Num beams:", num_beams)
    return start_index, stop_index, interval, num_beams


print("Info frame")
info_bytes = parse_string(info_print)
length, frame_id, block, scan_num = display_frame(info_bytes)
start_index, stop_index, interval, num_beams = display_contour(info_bytes)
print()

# status_data = struct.unpack('<' + (20 * 'B'), info_bytes[20:40])

print("Scan frame")
scan_bytes = parse_string(scan_print)
length, frame_id, block, scan_num = display_frame(scan_bytes)
print(length)
print(-len(scan_bytes) + (length - 20)/4)

# signature_data = struct.unpack('<HHQ', info_bytes[48:60])
# print("ID:", signature_data[0])
# print("Length:", signature_data[1])
# print("Description:", signature_data[2])


ranges = []
for index in range(20, length, 4):
    distance, strength = struct.unpack('<HH', scan_bytes[index:index+4])
    ranges.append(distance)

ranges = np.array(ranges) * 0.001
angles = np.linspace(np.deg2rad(start_index * 0.1), np.deg2rad(stop_index * 0.1), len(ranges))

x = ranges * np.cos(angles)
y = ranges * np.sin(angles)

fig = plt.figure(1, figsize=(15, 15))
plt.tight_layout()
plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)

axis = fig.add_subplot(1, 1, 1)
axis.set_aspect('equal', 'box')
axis.set_xlim(-10, 10)
axis.set_ylim(-10, 10)
axis.plot(x, y, '.')
plt.show()
