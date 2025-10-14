Linktree: https://lnk.bio/pulsar.upb
Form: https://docs.google.com/forms/d/e/1FAIpQLSfG_tcC6O9cKJTSny7cOH8dkWiBoDEMdTrw3XN6H_UBBGVmRw/viewform

# Task departament Software

## Problemă

Rover-ul va ateriza într-o zonă cu topografie cunoscută, la o poziție cunoscută (1,1). Acesta trebuie
să ajungă la un alt punct pentru a ridica o monstră de sol. Trebuie realizat un drum pentru ca
robotul să ajungă în locația respectivă.

## Informații suplimentare:

Panta maximă pe care robotul o poate urca este de 30 de grade. Harta va fi aleasă de voi, fiind o
imagine de 100x100 aleatorie, din care se va extrage canalul roșu de culoare. Acesta se va
normaliza, pentru a obține valori între 0 și 1.

Se pot aplica filtre de tip blur pentru a netezi suprafața.

Valoarea 0 a canalului normalizat reprezintă înălțimea de 0m, iar valoarea 1 reprezintă 3m. 1px pe
orizontală sau verticală corespunde unei distanțe de 0.1m.

Se poate alege între o deplasare de 4 direcții sau pe 8 direcții.

În final, se va atașa drumul robotului între cele 2 puncte sub formă de listă de puncte.

Limbajul de programare este la liberă alegere.

Se vor atașa:
- Un document word care va conține:
- Link la repo public de github, care va conține poza și codul
- Documentație a codului și felul în care ați găndit rezolvarea problemei, dar și alte
detalii pe care le considerați importante.

## Prea ușor?
Implementați un cost map care ține cont de gradientul terenului.
O imagine în care traseul va fi suprapus peste canalul roșu.