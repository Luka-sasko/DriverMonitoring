BOG = 6
BOGL = 2
BOGR = 4
BOGUK = BOGL + BOGR
BOGLP = 2
BOGRD = 4
BOGP = 0

preciznost = (BOGL + BOGR) / (BOGLP + BOGRD)
print("Preciznost:", preciznost)

odziv = BOGUK / (BOGUK + BOGP)
print("Odziv:", odziv)

f1_score = 2 * (preciznost * odziv) / (preciznost + odziv)
print("F1-Score:", f1_score)
