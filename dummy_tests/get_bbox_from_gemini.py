from tesi_gemini_robotics import GeminiClient
from PIL import Image
import cv2
import numpy as np


def visualizza_bbox(pil_image, box_2d, label="Oggetto"):
    """
    Sovrappone un bounding box rosso all'immagine e apre una finestra a schermo.

    Args:
        pil_image: L'oggetto PIL.Image originale.
        box_2d: Lista [ymin, xmin, ymax, xmax] normalizzata (0-1000) da Gemini.
        label: Testo da scrivere sopra il box.
    """
    # 1. Converti da PIL (RGB) a formato OpenCV (BGR + NumPy array)
    opencv_image = np.array(pil_image)
    opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_RGB2BGR)

    # 2. Ottieni le dimensioni reali dell'immagine
    height, width, _ = opencv_image.shape

    # 3. Estrai e converti le coordinate
    # Gemini restituisce: [ymin, xmin, ymax, xmax] su scala 0-1000
    ymin, xmin, ymax, xmax = box_2d

    # Denormalizzazione: (valore / 1000) * dimensione_reale
    start_point = (int(xmin / 1000 * width), int(ymin / 1000 * height))  # (x, y)
    end_point = (int(xmax / 1000 * width), int(ymax / 1000 * height))  # (x, y)

    # 4. Disegna il rettangolo
    # Sintassi: cv2.rectangle(img, pt1, pt2, color_BGR, thickness)
    # Rosso in BGR è (0, 0, 255)
    colore_rosso = (0, 0, 255)
    cv2.rectangle(opencv_image, start_point, end_point, colore_rosso, 2)

    # 5. (Opzionale) Aggiungi etichetta testo
    cv2.putText(opencv_image, label, (start_point[0], start_point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, colore_rosso, 2)

    # 6. Mostra direttamente a schermo
    cv2.imshow("Risultato Detection Gemini", opencv_image)

    # IMPORTANTE: cv2.waitKey(0) blocca l'esecuzione finché non premi un tasto.
    # Senza questo, la finestra si aprirebbe e chiuderebbe istantaneamente.
    print("Premi un tasto qualsiasi sulla finestra dell'immagine per continuare...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

system_instruction = """
Sei un sistema di visione robotica esperto in object detection spaziale. Il tuo compito è analizzare l'immagine fornita e identificare le coordinate della bounding box di un oggetto specifico richiesto dall'utente.

** REQUISITI DI OUTPUT: **
1. Restituisci la risposta ESCLUSIVAMENTE in formato JSON valido. Non aggiungere testo introduttivo o markdown (come ```json).
2. Le coordinate devono essere nel formato [ymin, xmin, ymax, xmax].
3. Le coordinate devono essere normalizzate su una scala da 0 a 1000 (dove 0 è l'estremo superiore/sinistro e 1000 è l'estremo inferiore/destro).
4. Se l'oggetto non è visibile o è ambiguo, imposta detected su false e le coordinate a null."
"""

def main():
    try:
        # (Opzionale) Potresti creare un secondo client per un altro modello
        gemini_describer = GeminiClient(model_name='models/gemini-robotics-er-1.5-preview', system_instruction=system_instruction)
    except Exception as e:
        print("Impossibile inizializzare il client Gemini. Termino.")
        print(e)
        return

    image_path = "scatto_simulatore.png"
    image_path = "../demos/scatto_simulatore.png"
    img = Image.open(image_path)
    # img = img.convert('RGB')
    # img.save(f"SCATTO_VEDIAMO.png")
    # visualizza_bbox(img, [0,0,0,0])
    ans_dict = gemini_describer.get_bounding_box_of("Cubo blu", img)
    print(ans_dict)

    # --- ESEMPIO DI UTILIZZO NEL TUO SCRIPT ---
    # Supponiamo che 'response_json' sia il dizionario che hai ottenuto da Gemini
    # e 'img' sia la tua immagine PIL caricata in precedenza.

    # Esempio di dati che ti arrivano da Gemini
    # response_json = {"box_2d": [300, 450, 600, 750], "target_object": "Tazza"}

    if ans_dict and ans_dict.get("box_2d"):
        visualizza_bbox(img, ans_dict["box_2d"], label=ans_dict.get("target_object", "Oggetto"))
    else:
        print("Nessun oggetto rilevato o coordinate mancanti.")

if __name__ == '__main__':
    main()

