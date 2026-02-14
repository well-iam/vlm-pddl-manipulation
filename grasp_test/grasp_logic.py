import open3d as o3d
import numpy as np
import cv2


class GraspEstimator:
    def __init__(self, camera_intrinsics):
        """
        camera_intrinsics: Matrice 3x3 dei parametri intrinseci della RealSense.
        """
        self.intrinsics = camera_intrinsics

    def box_to_pointcloud(self, depth_image, box_2d):
        """
        Converte i pixel DENTRO la bounding box in una nuvola di punti 3D.
        box_2d: [ymin, xmin, ymax, xmax] normalizzati (0-1000) o in pixel.
        """
        h, w = depth_image.shape
        ymin, xmin, ymax, xmax = box_2d

        # 1. Assicurati che le coordinate siano in pixel interi e dentro l'immagine
        xmin, xmax = max(0, int(xmin)), min(w, int(xmax))
        ymin, ymax = max(0, int(ymin)), min(h, int(ymax))

        # 2. Ritaglia la Depth Map (ROI)
        depth_crop = depth_image[ymin:ymax, xmin:xmax]

        # 3. Crea griglia di coordinate pixel per il crop
        # (Aggiungiamo l'offset xmin, ymin per tornare alle coordinate globali)
        x_grid, y_grid = np.meshgrid(np.arange(xmin, xmax), np.arange(ymin, ymax))

        # 4. Filtro validità (ignora depth=0 o troppo distanti > 1.5m)
        mask = (depth_crop > 0) & (depth_crop < 1500)  # Assumendo depth in mm

        z_valid = depth_crop[mask] / 1000.0  # Converti in Metri
        u_valid = x_grid[mask]
        v_valid = y_grid[mask]

        # 5. Deprojection (Pinhole Camera Model)
        # X = (u - cx) * Z / fx
        fx, fy = self.intrinsics[0, 0], self.intrinsics[1, 1]
        cx, cy = self.intrinsics[0, 2], self.intrinsics[1, 2]

        x_3d = (u_valid - cx) * z_valid / fx
        y_3d = (v_valid - cy) * z_valid / fy

        # Stack in array (N, 3)
        points_3d = np.vstack((x_3d, y_3d, z_valid)).T

        # Crea oggetto Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)

        print(f"Punti estratti dalla ROI: {len(points_3d)}")
        return pcd

    def estimate_pose(self, pcd, visualize=False):
        """
        Esegue la pipeline geometrica: RANSAC -> Pulizia -> OBB -> Assi Grasping
        """
        if len(pcd.points) < 50:
            print("Troppi pochi punti per stimare una posa!")
            return None, None

        # # --- STEP A: RIMOZIONE TAVOLO (RANSAC) ---
        # # Cerca il piano più grande (il tavolo) e rimuovi i punti che vi appartengono
        # plane_model, inliers = pcd.segment_plane(distance_threshold=0.015,  # 1.5 cm tolleranza
        #                                          ransac_n=3,
        #                                          num_iterations=1000)
        #
        # # Seleziona SOLO i punti che NON sono il tavolo (outliers del piano)
        # object_pcd = pcd.select_by_index(inliers, invert=True)
        # table_pcd = pcd.select_by_index(inliers)

        # --- STEP A.2:
        # 1. Assicurati che la nuvola sia orientata con Z verso l'alto
        # (Se hai già fatto la rotazione di 180° su X come dicevamo prima)
        points = np.asarray(pcd.points)

        print(points)
        # 2. Trova il "pavimento" geometrico
        # Usa il 1° percentile invece del minimo assoluto
        # Questo ignora l'1% dei punti più bassi (rumore) e trova il "vero" piano del tavolo
        z_floor_robust = np.percentile(points[:, 2], 1)
        print(z_floor_robust)
        # Definisci lo spessore da tagliare (es. 5mm sopra il livello del tavolo trovato)
        margin = 0.03

        # 4. Applica il filtro
        # Teniamo solo i punti che sono SUPERIORI al pavimento + margine
        mask = points[:, 2] > (z_floor_robust + margin)
        object_pcd = pcd.select_by_index(np.where(mask)[0])
        table_pcd = pcd.select_by_index(np.where(mask)[0], invert=True)

        # Opzionale: Visualizzare cosa è stato rimosso per debug
        # Crea un sistema di riferimento (terna cartesiana)
        # size: Lunghezza delle frecce in metri.
        #       Se il tuo cubo è 0.05m, usa 0.1 o 0.2 per vederlo bene.
        # origin: La posizione [0,0,0] (default).
        world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.2,
            origin=[0, 0, 0]
        )
        table_pcd.paint_uniform_color([1, 0, 0]) # Tavolo in rosso
        object_pcd.paint_uniform_color([0, 1, 0]) # Oggetto in verde
        o3d.visualization.draw_geometries([table_pcd, object_pcd, world_frame])
        input("Aspetto is")
        # --- STEP B: RIMOZIONE RUMORE ---
        # Rimuove punti isolati fluttuanti
        object_pcd, _ = object_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)

        if len(object_pcd.points) < 10:
            print("Oggetto scomparso dopo il filtering (era solo rumore o tavolo?)")
            return None, None

        # --- STEP C: ORIENTED BOUNDING BOX (OBB) ---
        obb = object_pcd.get_oriented_bounding_box()
        obb.color = (1, 0, 0)  # Box rossa per visualizzazione

        center = obb.center
        R = obb.R  # Matrice di rotazione (PCA)
        extent = obb.extent  # Dimensioni [l1, l2, l3]

        # --- STEP D: GRASP HEURISTIC (Cruciale per Sim2Real) ---
        # La PCA ordina gli assi arbitrariamente. Noi vogliamo afferrare l'oggetto:
        # 1. Lungo l'asse più corto (Width) -> Direzione chiusura gripper
        # 2. Con l'approccio (Z) perpendicolare all'oggetto o dall'alto.

        # Ordiniamo le dimensioni: small, mid, large
        dims_indices = np.argsort(extent)  # Indici: [piccolo, medio, grande]

        # Definiamo gli assi target del gripper (nel frame oggetto)
        # Asse X gripper (chiusura) = Asse più corto oggetto
        grasp_x = R[:, dims_indices[0]]

        # Asse Y gripper (lunghezza dita) = Asse medio oggetto
        grasp_y = R[:, dims_indices[1]]

        # Asse Z gripper (approccio) = Asse più lungo oggetto (o normale)
        grasp_z = np.cross(grasp_x, grasp_y)

        # Costruiamo la nuova matrice di rotazione ordinata
        R_grasp = np.column_stack((grasp_x, grasp_y, grasp_z))

        # --- Visualizzazione Debug (Opzionale) ---
        if visualize:
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=center)
            # Visualizza: Nuvola originale (grigia), Oggetto filtrato (blu), Box (rossa)
            pcd.paint_uniform_color([0.8, 0.8, 0.8])
            object_pcd.paint_uniform_color([0.1, 0.1, 0.8])
            o3d.visualization.draw_geometries([pcd, object_pcd, obb, coord_frame],
                                              window_name="Grasp Estimation Debug")

        return center, R_grasp, extent


# --- MAIN DI TEST (Sim2Real Check) ---
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    print("--- INIZIO TEST SIMULATO ---")

    # 1. SETUP INTRINSECI (Aggiornato per 512x512 e FOV 45°)
    # Focale f = (size / 2) / tan(45/2) = 256 / 0.414 = ~618 pixel
    # Se il tuo FOV in Coppelia è diverso da 45, ricalcola questo numero!
    f_pixel = 443.405
    cx, cy = 256.0, 256.0  # Centro immagine (512/2)

    intrinsics = np.array([[f_pixel, 0.0, cx],
                           [0.0, f_pixel, cy],
                           [0.0, 0.0, 1.0]])

    estimator = GraspEstimator(intrinsics)

    # 2. CARICA I DATI GENERATI
    try:
        # Carichiamo la matrice .npy (che sappiamo essere in METRI grazie al flag 1)
        depth_meters = np.load("test_depth.npy")
        print(f"Dati caricati. Shape: {depth_meters.shape}, Max Z: {np.max(depth_meters):.2f}m")

        # IMPORTANTE: Convertiamo in MILLIMETRI per coerenza con lo script
        # (Lo script ha filtri tipo < 1500 che si aspettano mm)
        depth_mm = depth_meters * 1000.0

    except FileNotFoundError:
        print("ERRORE: Non trovo 'test_depth.npy'. Hai lanciato export_data_coppelia.py?")
        exit()

    # 3. COORDINATE BOX (SIMULAZIONE GEMINI)
    # --- MODIFICA QUI CON I TUOI VALORI ---
    # Inserisci qui le coordinate che hai letto aprendo test_rgb.png
    # Esempio: Cubo Blu
    ymin, xmin = 320, 210  # Angolo in alto a sinistra
    ymax, xmax = 380, 260  # Angolo in basso a destra

    bbox_manuale = [ymin, xmin, ymax, xmax]
    print(f"Analisi Bounding Box: {bbox_manuale}")

    # 4. ESECUZIONE PIPELINE
    # A. Crop & Deproject
    pcd_roi = estimator.box_to_pointcloud(depth_mm, bbox_manuale)
    # R_fix = pcd_roi.get_rotation_matrix_from_xyz((np.pi, 0, 0))
    # pcd_roi.rotate(R_fix, center=(0, 0, 0))

    # pcd_roi = o3d.io.read_point_cloud("test_scene.ply")
    # B. Stima Posa
    # visualize=True aprirà la finestra Open3D.
    # Se tutto funziona, dovresti vedere SOLO il cubo, fluttuante, senza tavolo.
    center, rotation, extent = estimator.estimate_pose(pcd_roi, visualize=True)

    if center is not None:
        print("\n=== SUCCESSO! ===")
        print(f"Dimensione BBOX (x,y,z in mm): {extent}")
        print(f"Posizione Target (x,y,z in mm): {center}")
        print("Matrice di Rotazione (Assi del Gripper):")
        print(rotation)
        print("=================")
        print("Nota: L'Asse X (Rosso) del frame visualizzato dovrebbe essere il lato CORTO del cubo.")
    else:
        print("\n=== FALLITO ===")
        print("Non è stata trovata una posa valida (pochi punti o filtrati via).")

# --- MAIN DI TEST (Simulazione dati) ---
if __name__ == "__main":
    # 1. SETUP: Parametri Intrinseci Fittizi (simili a RealSense D435 640x480)
    # [fx, 0, cx]
    # [0, fy, cy]
    # [0,  0,  1]
    intrinsics = np.array([[615.0, 0.0, 256.0], #320.0
                           [0.0, 615.0, 256.0], #240.0
                           [0.0, 0.0, 1.0]])
    #fx = 443.4050067376326 preso da export_data_coppeliasim.py

    estimator = GraspEstimator(intrinsics)

    # 2. INPUT MOCK (Simuliamo quello che ti darebbe Gemini e RealSense)
    # Creiamo una depth map finta: piano inclinato (tavolo) + cubo in rilievo
    W, H = 512, 512
    fake_depth = np.ones((H, W), dtype=np.float32) * 500  # Sfondo a 50cm
    # Aggiungiamo un "cubo" più vicino (45cm)
    fake_depth[200:300, 280:360] = 450

    # Bounding Box da Gemini (Pixel)
    bbox_gemini = [200, 280, 300, 360]  # [ymin, xmin, ymax, xmax]

    print("Elaborazione pipeline geometrica...")

    # 3. ESECUZIONE
    # A. Crop & Deproject
    pcd_roi = estimator.box_to_pointcloud(fake_depth, bbox_gemini)

    # B. Stima Posa
    center, rotation = estimator.estimate_pose(pcd_roi, visualize=True)

    if center is not None:
        print("\n--- RISULTATO GRASPING ---")
        print(f"Target Position (Camera Frame): {center}")
        print(f"Target Rotation Matrix:\n{rotation}")
        print("---------------------------")
        print("Prossimo passo: Moltiplicare per T_base_camera per inviare al robot.")