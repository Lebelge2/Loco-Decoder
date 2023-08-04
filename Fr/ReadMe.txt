Vous trouverez ici des conseils, trucs et astuces pour r�aliser le d�codeur.

Soudure des composants.
Il faut d�abord souder les composants les plus petits, transistors et r�sistances puis diodes et condensateurs puis le reste.

Avant assemblage.
- Brancher le signal DCC sur les fils noir et rouge. V�rifier la tension 5v au r�gulateur 78M05
- Programmer l�Arduino, si le t�l�chargement ne se fait pas, il faut provoquer un Reset juste apr�s la compilation, au moment du t�l�chargement avec un fil volant ou un bouton poussoir. Voir photo n�  et https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/32U4Note.pdf?_gl=1*kfv5ax*_ga*MTA4NDIxNjMwNC4xNjg5NjI1Njg3*_ga_T369JS7J9N*MTY5MTE3NjcyNi40LjAuMTY5MTE3NjcyNi42MC4wLjA.

ATTENTION: Lors du choix de la carte "SparkFun Pro Micro", par d�faut le processeur est le 3V3 8MHz. Chang� le en 5V 16MHz.
sans quoi le Bootloader sera corrompu.(A r�installer)


Assemblage.
Souder les deux circuits dos � dos (Voir photo n� 1) vous pouvez laisser un espace d�un ou deux millim�tres pour une meilleure ventilation, en effet le r�gulateur chauffe un peu car il doit dissiper une puissance de 400mW (10V x 40mA)


