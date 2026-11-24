import serial
import time

# Configurer le port et la vitesse (baudrate)
PORT = "/dev/ttyACM0" 
BAUDRATE = 115200
T0 = time.time()


try:
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        print(f"Écoute sur {PORT} à {BAUDRATE} bauds... (Ctrl+C pour arrêter)")
        while True:
            if ser.in_waiting > 0:
                ligne = ser.readline().decode("utf-8", errors="replace").strip()
                if ligne:
                    dt = time.time()-T0
                    print(f"t = {dt:.2f} s : {ligne}")
except serial.SerialException as e:
    print(f"Erreur de liaison série : {e}")
except KeyboardInterrupt:
    print("\nArrêt de la lecture.")
