BPZ = 7
BSZ = 6
BPZ_promaseni = 0
BUZ = 1


preciznost = BSZ / BPZ
print("Preciznost:", preciznost)


odziv = BSZ / (BSZ + BPZ_promaseni)
print("Odziv:", odziv)

f1_score = 2 * (preciznost * odziv) / (preciznost + odziv)
print("F1-Score:", f1_score)
