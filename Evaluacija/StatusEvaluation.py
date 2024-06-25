BDU = 2
BPDU = 4
BDS = 2
BNDS = 1

preciznost = BDU / (BDU + BPDU)
print("Preciznost:", preciznost)

odziv = BDS / (BDS + BNDS)
print("Odziv:", odziv)

f1_score = 2 * (preciznost * odziv) / (preciznost + odziv)
print("F1-Score:", f1_score)
