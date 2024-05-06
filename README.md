# Geolocator connected to telecommunications network
Hey there! This is a final project for an IoT Class. This project is done in Arduino Nano and uses a GPS Neo 6 along with a SIM800L module. This device is then able to send SMS containing its actual GPS coordinates to the movile who send it a message. 

* In this repository you can find the main code along with a scientific poster for its exposition.

## How it works? 
The fundamental component in this device is the arduino NANO, the Arduino communicates with the GPS Neo 6, the GPS is constatly obtaining the GPS coordenates. When a SMS is received by the SIM800L module the Arduino gets the GPS ubication, creates a SMS with the coordenates and send it back to the movile who send the SMS to the device. 

## Which componentes uses? 
* Arduino NANO
* GPS Neo 6
* SIM800L Module
* A Nano SIM
* LiPO Battery 3.3V 500mA

## What is its main purpose?
I've done this project for a low-cost device able to get GPS coordenates and send it by the communications networks. This is my (very short) contribuition in order to increses security and reduce the number of missing people. 
