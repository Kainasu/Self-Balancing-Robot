# L'organisation du répertoire **monde**:

Le répertoire **monde** contient 2 grands répertoires qui servent à piloter le robot:

* Le répertoire **monde/arduino** contient les fichiers Arduino qui sont utilisés pour piloter le robot.

* Le répertoire **monde/python** contient les fichiers Python qui communique en série avec l'adruino pour piloter le robot.

Dans le répertoire **monde/arduino** on trouve les fichiers:
- **monde/arduino/balancingrobot_driver/balancingrobot_driver.ino**: le code Arduino pour piloter le robot.
- **monde/arduino/serial_communication/serial_communication.ino**: un exemple qui teste la communication série avec l'arduino.
- **monde/arduino/adafruit_example/adafruit_example.ino**: un exemple qui montre la récupération des données de la centrale inertielle en utilisant la bibliothèque *Adafruit*.
- **monde/arduino/stepper_without_interrupt/stepperMotor_test/stepperMotor_test.ino**: un exemple qui teste le fonctionnement de moteur pas à pas sans interruption.
- **monde/arduino/stepper_with_interrupt/stepperMotor_test/stepperMotor_test.ino**: un exemple qui teste le fonctionnement de moteur pas à pas avec interruption.
- **monde/arduino/complementary_filter/complementary_filter.ino**: un exemple qui calcule l'angle récupérer par la centrale inertielle en utilisant le filtre complémentaire.

Dans le répertoire **monde/python** on trouve:
- le répertoire **monde/python/ai_models**: qui contient les modèles de réseau de neurones entrainés pour tenir le robot debout.
- le répertoire **monde/python/balancingrobot_driver.py**: le fichier Python qui donne les actions à l'arduino pour piloter le robot soit par l'algorithme **PID** ou par un réseau de neurones.
- le fichier **monde/python/serial-test.py**: un exemple qui teste la communication série avec l'arduino.