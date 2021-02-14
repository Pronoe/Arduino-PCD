Auteur : Patrick Souty
Date de création initiale : 31/01/2020

- Version 1.0 du 01/02/2021 
  - version fonctionnelle
  - seule limitation : résolution du CAN qui entraine un euil de mesure de l'ordre de 20 W (bruit de quantification)
- Version 2.0 du 02/02/2021
  - ajout fonction serveur Web et bibliothèque Time
- Version 3.0 du 02/02/2021
  - autocalibration 5V par mesure initiale d'une diode bandgap interne
- Version 4.0 du 03/02/2021
  - mesure de courant en mode différentiel de l'ADC et avec gain de 10
- Version 5.0 du 05/12/2021
  - gestion automatique du gain ADC en fonction des mesures effectuées ajout info sur temps de traitement
- Version 6.0 du 10/02/2021
  - ajout stockage sur carte SD (en cours)
  - calcul de la consommation cumulée
- Versio 7.0 du 11/02/2021 
  - stockage mesures sur carte SD
- Version 7.1 du  13/02/2021
  - extinction rétroéclairage LCD après 1er stockage sur carte SD

Prochaines évolutions :      # consultation à distance fichiers carte SD
                             # filtrage des mesures à faible courant
                             # meilleure gestion gain auto en faisant sytématiquement un paquet sur 2 à gain min et en comparant les mesures
                             # ou commande par boutons poussoirs 
                             # commutation retro éclairage LCD
                             # communication en bluetooth avec appli smartphone