# robot-sumo-controlado
Proyecto de robot controlado mediante mando de PS4 con un ESP32-CAM.
Robot Sumo — Control Manual vía PS4 (ESP32)
Robot de combate sumo controlado mediante un mando PS4 conectado por Bluetooth a un ESP32. El proyecto evolucionó de un sistema autónomo basado en visión artificial a un control manual directo, eliminando la dependencia de computadora externa, cámara IP y red WiFi.
Características:

Control proporcional de avance, retroceso y giro con un solo joystick (mezclador tipo arcade)
Rampas de aceleración y frenado activo para aprovechar el torque de los motores JGA25-370 sin patinaje
Mecanismo anti-flipper con servomotor DS3218MG, controlado de forma automática y no bloqueante
Comunicación Bluetooth mediante la librería PS4Controller, sin dependencias de red

Hardware: ESP32, mando PS4, motores JGA25-370, servomotor DS3218MG, armazón de protección.
