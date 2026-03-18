def to_bcd(value):

    digits = str(int(value))

    bcd = ""

    for d in digits:
        bcd += format(int(d), "04b")

    return bcd