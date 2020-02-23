import struct

def to_hexstring(bytestring):
    return ' '.join(['{:02X}'.format(x) for x in bytestring])

def generate_foc_drive_command(speed, steer):
    # emanuel FOC hoverboard protocol (https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC)

    # start frame
    cmd = struct.pack('H', 0xAAAA)

    # direction values
    cmd += struct.pack('h', steer)

    # speed values, -1000 to 1000
    cmd += struct.pack('h', speed)

    # calculate checksum
    checksum = 0xAAAA ^ steer ^ speed
    cmd += struct.pack('H', checksum)

    return cmd


# print(' '.join(str(x) for x in generate_speed_command(10000, -10000)))
# print(' '.join('{:04X}'.format(x) for x in generate_foc_drive_command(100, 0)))

print(to_hexstring(generate_foc_drive_command(100, 0)))


# print(to_int16(257))