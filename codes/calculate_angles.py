import smbus
import math

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
bus = smbus.SMBus(1)
address = 0x68  # via i2cdetect

bus.write_byte_data(address, power_mgmt_1, 0)


def read_byte(reg):
    return bus.read_byte_data(address, reg)


def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg + 1)
    value = (h << 8) + l
    return value


def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val


def dist(a, b):
    return math.sqrt((a * a) + (b * b))


def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)


def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)


def get_rotation(verbose=False):

    gyroscope_xout = read_word_2c(0x43) / 131.
    gyroscope_yout = read_word_2c(0x45) / 131.
    gyroscope_zout = read_word_2c(0x47) / 131.

    acceleration_xout = read_word_2c(0x3b) / 16384.
    acceleration_yout = read_word_2c(0x3d) / 16384.
    acceleration_zout = read_word_2c(0x3f) / 16384.

    acceleration_xout_scaled = acceleration_xout
    acceleration_yout_scaled = acceleration_yout
    acceleration_zout_scaled = acceleration_zout

    x_rotation = get_x_rotation(acceleration_xout_scaled, acceleration_yout_scaled, acceleration_zout_scaled)
    y_rotation = get_y_rotation(acceleration_xout_scaled, acceleration_yout_scaled, acceleration_zout_scaled)

    if verbose:

        print("gyroscope")
        print("--------")

        print("gyroscope_xout: ", ("%5d" % gyroscope_xout), " scaled: ", (gyroscope_xout))
        print("gyroscope_yout: ", ("%5d" % gyroscope_yout), " scaled: ", (gyroscope_yout))
        print("gyroscope_zout: ", ("%5d" % gyroscope_zout), " scaled: ", (gyroscope_zout))

        print("acceleration sensor")
        print("---------------------")

        print("acceleration_xout: ", ("%6d" % acceleration_xout), " scaled: ", acceleration_xout_scaled)
        print("acceleration_yout: ", ("%6d" % acceleration_yout), " scaled: ", acceleration_yout_scaled)
        print("acceleration_zout: ", ("%6d" % acceleration_zout), " scaled: ", acceleration_zout_scaled)

        print("X Rotation: ", x_rotation)
        print("Y Rotation: ", y_rotation)

    return gyroscope_yout, y_rotation