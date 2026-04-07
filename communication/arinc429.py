from constantes import *

class ARINC429:
    """
    Mot ARINC 429 = 32 bits :
      [1:8]   = label (octal, LSB first)
      [9:10]  = SDI
      [11:28] = données
      [29:30] = SSM
      [31]    = parity (odd)
    """

    @staticmethod
    def odd_parity(word: int) -> int:
        """Calcule le bit de parité impaire sur 32 bits."""
        return 1 - (bin(word).count('1') % 2)

    @staticmethod
    def encode_label001_altitude(altitude_ft: int, state: int) -> int:
        """
        Label 001 Altitude + État
          Altitude   bits [13:28]  (16 bits binaire, résolution 1 pied)
          État       bits [11:12]  (2 bits)
          Label      bits [1:8]    = 0x01
        """
        if not (ALT_MIN_FT <= altitude_ft <= ALT_MAX_FT):
            raise ValueError(f"Altitude hors limites : {altitude_ft} ft")
        if state not in (0, 1, 2):
            raise ValueError(f"État invalide : {state}")

        label      = 0b00000001

        # État avionique encodé sur les bits [11:12] : 0=AU_SOL, 1=CHANGEMENT_ALT, 2=VOL_CROISIERE
        state_bits = (state & 0x3) << 10

        # Altitude encodée en binaire sur les bits [13:28], résolution 1 ft, max 40000 ft (énoncé)
        alt_bits   = (altitude_ft & 0xFFFF) << 12

        word   = label | state_bits | alt_bits
        parity = ARINC429.odd_parity(word & 0x7FFFFFFF)
        word  |= (parity << 31)
        return word & 0xFFFFFFFF

    @staticmethod
    def decode_label001(word: int):
        """Décode label 001 -> (altitude_ft, state)"""
        altitude_ft = (word >> 12) & 0xFFFF
        state       = (word >> 10) & 0x3
        return altitude_ft, state

    @staticmethod
    def encode_bcd(value: float, digits: int, decimals: int) -> int:
        """Encode un flottant en BCD (digits chiffres, decimals décimales)."""
        factor  = 10 ** decimals
        int_val = round(abs(value) * factor)
        sign    = 0 if value >= 0 else 1
        bcd     = 0
        for i in range(digits):
            bcd |= (int_val % 10) << (i * 4)
            int_val //= 10
        return bcd | (sign << (digits * 4))

    @staticmethod
    def decode_bcd(bcd: int, digits: int, decimals: int) -> float:
        """Décode un BCD -> flottant."""
        sign = (bcd >> (digits * 4)) & 1
        val  = 0
        for i in range(digits - 1, -1, -1):
            val = val * 10 + ((bcd >> (i * 4)) & 0xF)
        result = val / (10 ** decimals)
        return -result if sign else result

    @staticmethod
    def encode_label002_climb(climb_m_min: float) -> int:
        """Label 002 : Taux de montée BCD 4 chiffres, résolution 0.1 m/min"""
        label  = 0b00000010

        # Taux de montée encodé en BCD sur 4 chiffres, résolution 0.1 m/min, max ±800 m/min (énoncé)
        bcd    = ARINC429.encode_bcd(climb_m_min, 4, 1)

        word   = label | (bcd << 8)
        parity = ARINC429.odd_parity(word & 0x7FFFFFFF)
        word  |= (parity << 31)
        return word & 0xFFFFFFFF

    @staticmethod
    def decode_label002(word: int) -> float:
        bcd = (word >> 8) & 0x1FFFFF
        return ARINC429.decode_bcd(bcd, 4, 1)

    @staticmethod
    def encode_label003_attack(angle_deg: float) -> int:
        """Label 003 : Angle d'attaque BCD 3 chiffres, résolution 0.1°"""
        label  = 0b00000011

        # Angle d'attaque encodé en BCD sur 3 chiffres, résolution 0.1°, plage ±16° (énoncé)
        bcd    = ARINC429.encode_bcd(angle_deg, 3, 1)
        word   = label | (bcd << 8)
        parity = ARINC429.odd_parity(word & 0x7FFFFFFF)
        word  |= (parity << 31)
        return word & 0xFFFFFFFF

    @staticmethod
    def decode_label003(word: int) -> float:
        bcd = (word >> 8) & 0x1FFFFF
        return ARINC429.decode_bcd(bcd, 3, 1)

    @staticmethod
    def word_to_hex(word: int) -> str:
        return f"0x{word:08X}"

    @staticmethod
    def word_to_bin(word: int) -> str:
        return f"{word:032b}"

