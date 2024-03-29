# MDP_STM
STM32, Containing the entire repo for the hardware side of STM. Includes Ultrasonic sensors, PID for straight line movement using Gyro, Infrared sensors to detect obstacles at the side, as well as encoder to measure distance, and well as turnings using gyro.

To measure distance, you need observe the encoder counts per revolution which is roughly 1400 to 1600 depending on your bot. So you just take the total counts that you have travelled/counts_per_rev * your wheel circumference which is 20.5 cm to get the total distance your bot has travelled.

Tips for future groups:
Please Look out for which pins to connect to for the pins configuration. Know where they are before you connect. Read the data Sheets.

Code wise: try to finish up the basic movements before week 7 which is checklist.

Week 8 onwards is the start of the calibrating of turns etc with algo side, try to sync your turns with what is potrayed in the algorithm.


I am giving my entire repo to everyone in future because I had a hard time figuring it out myself. And I am trying to be kind and not make my next batchmates suffer as much as me. Hence please try your best, I had sacrificed countless nights just trying to figure out and ask for help from other groups to make this bot work. 

Cheer and I wish you all the best. Don't lose hope!
