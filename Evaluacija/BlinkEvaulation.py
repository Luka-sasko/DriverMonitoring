BPT = 29
BST = 23
BPT_promaseni = 8
BUN = 6


preciznost = BST / BPT
print("Preciznost:", preciznost)


odziv = BST / (BST + BPT_promaseni)
print("Odziv:", odziv)


f1_score = 2 * (preciznost * odziv) / (preciznost + odziv)
print("F1-Score:", f1_score)
