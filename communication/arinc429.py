def encode_altitude(altitude):

    altitude = int(altitude)

    word = altitude << 13

    return word 