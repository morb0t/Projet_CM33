# Projet CM33

Ce projet vise à exploiter les capacités de la carte **NXP-LPC5569**, équipée du microprocesseur **CM33**, à travers le développement d'une calculatrice embarquée. Il se structure en trois axes principaux :  

## 1. Gestion des Séquences VT100 et du Système de Fichiers FAT  
- Conception d'une interface de communication avec la carte SD.  
- Intégration de la bibliothèque **libc** à **FatFS** pour une gestion optimisée des fichiers.  

## 2. Communication avec l'Écran LCD  
- Mise en place d'une communication efficace avec un écran LCD.  
- Initialisation et synchronisation du **cœur 1** avec le **cœur 0** pour exécuter des tâches spécifiques.  
- Exécution des fonctions d'affichage sur le **cœur 1**, en modes **synchrone** et **asynchrone**.  

## 3. Acquisition des Données Capteurs et Utilisation de PowerQuad  
- Développement de fonctionnalités permettant la récupération des données de l’accéléromètre **MMA8652**.  
- Gestion du temps afin d'assurer la synchronisation des opérations.  
- Implémentation du calcul du **cosinus** d’un vecteur via **PowerQuad**.  
- Réalisation des **transformations de Fourier directe et inverse** pour l’analyse des signaux analogiques.  
