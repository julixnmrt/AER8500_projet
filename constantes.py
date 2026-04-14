
ALT_MAX_FT       = 40000      # pieds
ALT_MIN_FT       = 0
CLIMB_MAX_M_MIN  = 800.0      # m/min
CLIMB_RES        = 0.1        # résolution taux de montée
ALT_RES          = 1          # résolution altitude (pied)
STALL_ANGLE      = 15.0       # angle de décrochage positif (°)
CRUISE_ANGLE     = 3.0        # angle d'attaque en vol de croisière (palier)

# Décrochage négatif : angle en dessous duquel l'avion décroche en descente = -9.0°

NEG_STALL_ANGLE  = CRUISE_ANGLE - (STALL_ANGLE - CLIMB_RES - CRUISE_ANGLE) - CLIMB_RES
ATTACK_MAX       = STALL_ANGLE              
ATTACK_MIN       = -(STALL_ANGLE)          
MAX_SPEED_M_S    = 150.0      # vitesse sol max à 100% puissance (m/s ≈ 540 km/h)
SIM_TICK_S       = 0.05       # secondes par tick de simulation

# Conversion
FT_PER_M         = 3.28084

# États
STATE_AU_SOL        = 0
STATE_CHANGEMENT    = 1
STATE_VOL_CROISIERE = 2
STATE_NAMES = {
    STATE_AU_SOL:        "AU_SOL",
    STATE_CHANGEMENT:    "CHANGEMENT_ALT",
    STATE_VOL_CROISIERE: "VOL_CROISIÈRE",
}
