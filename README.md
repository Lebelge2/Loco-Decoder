# Loco-Decoder
Mobile locomotive decoder  with Arduino PRO MICRO (5V 16MHz)

Vous trouverez ici des conseils, trucs et astuces pour réaliser le décodeur.

Soudure des composants.
Il faut d’abord souder les composants les plus petits, transistors et résistances puis diodes et condensateurs puis le 

reste.

Avant assemblage.
- Brancher le signal DCC sur les fils noir et rouge. Vérifier la tension 5v au régulateur 78M05
- Programmer l’Arduino, si le téléchargement ne se fait pas, il faut provoquer un Reset juste après la compilation, au 

moment du téléchargement avec un fil volant ou un bouton poussoir. Voir photo n° 2 et 

https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/32U4Note.pdf?


ATTENTION: Lors du choix de la carte "SparkFun Pro Micro", par défaut le processeur est le 3V3 8MHz. Changer le en 5V 16MHz.
sans quoi le Bootloader sera corrompu. (A réinstaller)


Assemblage.
Souder les deux circuits dos à dos (Voir photo n° 1) vous pouvez laisser un espace d’un ou deux millimètres pour une 

meilleure ventilation, en effet le régulateur chauffe un peu car il doit dissiper une puissance de 400mW (10V x 40mA)



