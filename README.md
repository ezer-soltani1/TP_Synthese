# TP_Synthèse - Autoradio 

HIMED Zineddine - SOLTANI Ezer

## Introduction

Ce projet s’inscrit dans le cadre du TP de synthèse portant sur la réalisation d’un système embarqué faisant office d’autoradio sur carte **STM32 NUCLEO-L476RG**. L’objectif est de créer une chaîne audio complète intégrant l’acquisition, le traitement et la restitution d’un signal sonore, avec une interface utilisateur via un shell ainsi qu’un affichage lumineux type VU-mètre.

Le système repose notamment sur FreeRTOS, un shell série via USART2, un GPIO Expander contrôlant un ensemble de LED via SPI, et le codec audio **SGTL5000**, configuré en I2C et utilisant le protocole I2S via SAI2 pour le flux audio. Le TP est structuré en plusieurs étapes successives, chacune validant une partie matérielle ou logicielle avant l'intégration finale.


## Objectifs

- Initialiser un projet embarqué sur STM32 avec FreeRTOS.
- Mettre en place un shell fonctionnant en tâche FreeRTOS, avec interruptions et driver structuré.
- Piloter un GPIO Expander via SPI pour contrôler un ensemble de LED servant de VU-mètre.
- Configurer le codec audio **SGTL5000** :
  - Configuration en I2C
  - Transfert audio en I2S via SAI2, horloge MCLK activée, DMA en mode circulaire
- Générer des signaux audio (ex : triangle) et les analyser à l'oscilloscope.
- Réaliser un bypass numérique ADC → DAC.
- Implémenter un filtre RC numérique sous forme d'équation par récurrence.
- Ajouter un effet audio numérique (distorsion, tremolo, delay, etc.).


## 1) Démarrage du projet

La première étape du TP consistait à créer un projet STM32 sous **STM32CubeIDE** pour la carte NUCLEO-L476RG sans activer la BSP afin de garder le contrôle sur la configuration matérielle. Après la génération du code, plusieurs vérifications ont été réalisées pour s'assurer du bon fonctionnement des périphériques de base avant d'aborder la partie audio plus complexe.

Nous avons d’abord testé la LED LD2 afin de valider l’accès au GPIO. L’allumage et le clignotement ont fonctionné comme prévu, confirmant la bonne configuration du microcontrôleur et du clocking de base : 

![Clignotement de la LED](Clignotement_LED.jpeg)


Nous avons ensuite configuré l’USART2, relié au ST-Link interne, afin de communiquer avec un terminal série sur PC. La transmission a été validée en envoyant des messages simples, puis nous avons redirigé la fonction `printf()` vers cette liaison afin de simplifier le débogage et l'affichage des logs durant la suite du projet. Ces premières étapes ont permis d’obtenir une interface de sortie fiable pour vérifier le fonctionnement des modules développés.

![Test de Printf](test_uart2.png)

Après validation des périphériques basiques, nous avons activé **FreeRTOS en mode CMSIS-V1** afin de travailler en environnement multitâche. Cela a permis d’isoler chaque fonctionnalité (shell, audio, effets, affichage LED) dans des tâches indépendantes tout en conservant une meilleure lisibilité et modularité du code.

Une étape essentielle consistait à mettre en place un **shell accessible via UART**. Celui-ci s’exécute dans une tâche FreeRTOS dédiée et utilise des interruptions pour la réception série. Le shell permet d’interagir dynamiquement avec le système, notamment pour tester les différents modules (GPIO Expander, codec SGTL5000, filtres, effets, etc.). Cette approche offre une meilleure flexibilité qu’un programme à comportement figé, car elle permet de modifier les paramètres en temps réel sans recompiler.

À ce stade, le système était fonctionnel avec :
- une communication série fiable,
- un shell capable de recevoir et interpréter des commandes,
- un environnement multitâche stable,
- une interface de trace via `printf()` pour le débogage.

Cette base logicielle a servi de fondation pour l’intégration des éléments audio et du pilotage des LEDs.
