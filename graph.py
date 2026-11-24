
import serial
import time
import graphviz
import matplotlib.pyplot as plt
from PIL import Image
from io import BytesIO

# Configuration série
PORT = "/dev/ttyACM0"
BAUDRATE = 115200

T0 = time.time()

L_tag = []
Dict_distance = {}

# Création du graphe Graphviz
def creer_graphe():

    graphique = graphviz.Digraph(
        'graphe',
        format='png',
        graph_attr={
            'rankdir': 'LR',
            'overlap': 'false',
            'splines': 'true'
        },
        node_attr={
            'shape': 'circle',
            'color': 'black',
            'fontname': 'Arial',
            'width': '1',
            'height': '1'
        },
        edge_attr={
            'fontname': 'Arial',
            'fontsize': '10',
            'arrowsize': '1'
        }
    )

    # Sommets
    for tag in L_tag:
        graphique.node(tag, tag)

    # Arêtes orientées et distances
    for paire, distance in Dict_distance.items():

        source, destinataire = paire.split(",")

        graphique.edge(
            source,
            destinataire,
            label=f"{distance:.2f} m"
        )

    return graphique


# Initialisation de la fenêtre
plt.ion()

fig, ax = plt.subplots(figsize=(12, 8))
fig.canvas.manager.set_window_title("Réseau DWM1001")

ax.set_title("Réseau des tags")
ax.axis("off")

plt.show(block=False)


try:
    with serial.Serial(PORT, BAUDRATE, timeout=0.1) as ser:

        print(f"Écoute sur {PORT} à {BAUDRATE} bauds...")

        while plt.fignum_exists(fig.number):

            ligne = ser.readline().decode(
                "utf-8", errors="replace"
            ).strip()

            if ligne:

                try:
                    source, destinataire, distance = ligne.split(",")

                    source = source.strip()
                    destinataire = destinataire.strip()
                    distance = float(distance.strip())

                    dt = time.time() - T0

                    print(
                        f"t = {dt:.2f} s : "
                        f"{source} -> {destinataire} : "
                        f"{distance:.2f} m"
                    )

                    # Mise à jour des tags
                    if source not in L_tag:
                        L_tag.append(source)

                    if destinataire not in L_tag:
                        L_tag.append(destinataire)

                    # Mise à jour de la distance
                    Dict_distance[
                        f"{source},{destinataire}"
                    ] = distance

                    # Génération du graphe en mémoire
                    graphique = creer_graphe()

                    image_bytes = graphique.pipe(format='png')
                    image = Image.open(BytesIO(image_bytes))

                    # Actualisation de la fenêtre existante
                    ax.clear()
                    ax.imshow(image)
                    ax.axis("off")
                    ax.set_title(
                        "Réseau des tags DWM1001"
                    )

                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

                except ValueError:
                    print(f"Ligne invalide : {ligne}")

            plt.pause(0.01)


except serial.SerialException as e:
    print(f"Erreur de liaison série : {e}")

except KeyboardInterrupt:
    print("\nArrêt de la lecture.")

finally:
    plt.close('all')
