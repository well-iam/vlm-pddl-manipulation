import open3d as o3d
import numpy as np

def get_sensible_pose(pcd_cropped):
    # 1. Rimuovi il tavolo (Segmentazione Piano)
    # RANSAC: trova il piano con più punti (distance_threshold in metri, es 1cm)
    plane_model, inliers = pcd_cropped.segment_plane(distance_threshold=0.01,
                                                     ransac_n=3,
                                                     num_iterations=1000)

    # Seleziona i punti che NON sono il piano (outliers) -> L'oggetto vero e proprio
    object_pcd = pcd_cropped.select_by_index(inliers, invert=True)

    # 2. Rimuovi rumore (punti volanti sparsi)
    object_pcd, _ = object_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # 3. Calcola la Oriented Bounding Box (OBB)
    # Questa funzione calcola la posa 6D "sensata" basata sulla geometria
    obb = object_pcd.get_oriented_bounding_box()
    obb.color = (1, 0, 0)  # Disegnala in rosso per debug

    # 4. Estrai i dati per il robot
    center = obb.center  # Posizione (x, y, z) del centroide
    rotation_matrix = obb.R  # Matrice di rotazione 3x3 (Orientamento)
    extent = obb.extent  # Dimensioni (lunghezza, larghezza, altezza)

    return center, rotation_matrix, extent, object_pcd, obb

def main():
    # --- ESEMPIO DI UTILIZZO ---
    # Carichiamo una nuvola di punti (nella realtà questa viene dal crop di Gemini)
    # Qui creo un cubo finto ruotato per simulazione
    pcd = o3d.geometry.PointCloud()
    points = (np.random.rand(1000, 3) - 0.5) * [0.1, 0.05, 0.02]  # Un "mattoncino"
    # Ruotiamo il mattoncino
    R = pcd.get_rotation_matrix_from_xyz((np.pi / 4, 0, np.pi / 6))
    points = points @ R.T + [0.2, 0, 0.5]  # Lo spostiamo nello spazio
    pcd.points = o3d.utility.Vector3dVector(points)

    # Chiamiamo la funzione magica
    center, rot, size, obj, box = get_sensible_pose(pcd)

    print(f"Centro trovato: {center}")
    print(f"Rotazione trovata:\n{rot}")

    # Visualizzazione (Solo se hai display, altrimenti salta)
    # o3d.visualization.draw_geometries([obj, box, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])

if __name__ == '__main__':
    main()