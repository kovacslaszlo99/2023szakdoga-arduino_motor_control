Ez a szakdolgozat projektemhez készült Arduino program kód.

Ez egy távcső vezérlő (GOTO) mechanika.

A programot egy Teensy 4.0 arduino panelra töltöttem fel.

A program lényege, hogy UART csatlakozáson kérészül meg kapja, hogy a két léptető motort milyen irányba és hány lépéssel mozgása.
Ehhez TMC2209 léptető motor driver-t használtam és két darab 200 lépéses léptető motort.
A program kvázi egyszerre képes mozgatni a két léptető motort PWM jel szabályozással.
A mozgásban bele van építve gyorsulás és lassulás is.
A program képes óragépként is működni, ez a csillag követést teszi lehetővé.
