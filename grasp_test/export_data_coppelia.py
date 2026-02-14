import numpy as np
import cv2
import open3d as o3d
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

print("Connessione a CoppeliaSim...")
client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()
time.sleep(0.2)  # Serve per dare tempo ai buffer del sensore di immagini di riempirsi

# --- CONFIGURAZIONE ---
SENSOR_NAME = '/visionSensor'  # Controlla il nome nella gerarchia della scena
NEAR_CLIP = 0.01  # Copia dalle proprietà del sensore in Coppelia
FAR_CLIP = 5.0  # Copia dalle proprietà del sensore in Coppelia


# ----------------------

def get_data_from_sim():
    # Ottieni l'handle del sensore
    vision_sensor_handle = sim.getObject(SENSOR_NAME)

    # 1. Ottieni immagine RGB
    img_buffer, res = sim.getVisionSensorImg(vision_sensor_handle)
    img_rgb = np.frombuffer(img_buffer, dtype=np.uint8).reshape(res[1], res[0], 3)

    # Coppelia restituisce l'immagine capovolta verticalmente, giriamola
    img_rgb = cv2.flip(img_rgb, 0)
    img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)  # OpenCV usa BGR

    # 2. Ottieni Depth Buffer
    depth_buffer, res = sim.getVisionSensorDepth(vision_sensor_handle, 1)
    depth_norm = np.frombuffer(depth_buffer, dtype=np.float32).reshape(res[1], res[0])
    depth_norm = cv2.flip(depth_norm, 0)  # Anche questa è capovolta

    z_meters = depth_norm
    return img_rgb, z_meters, res


def save_point_cloud(rgb, depth_meters, filename="test_scene.ply"):
    # Creiamo una nuvola per visualizzazione rapida usando Open3D
    # Nota: Stiamo assumendo un FOV standard di 60 gradi per la matrice intrinseca fittizia
    h, w = depth_meters.shape
    fx = w / (2 * np.tan(np.deg2rad(60) / 2))  # Approssimazione
    fy = fx
    cx, cy = w / 2, h / 2

    print(f"""
    h={h}
    w={w}
    fx={fx}
    fy={fy}
    cx={cx}
    cy={cy}
    """)

    intrinsics = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

    # Crea immagini Open3D
    o3d_color = o3d.geometry.Image(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
    o3d_depth = o3d.geometry.Image(depth_meters.astype(np.float32))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color, o3d_depth, depth_scale=1.0, depth_trunc=3.0, convert_rgb_to_intensity=False)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)

    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.2,
        origin=[0, 0, 0]
    )
    o3d.visualization.draw_geometries([pcd, world_frame])

    # 1. Recupera la matrice da CoppeliaSim (Pose della camera)
    # sim.getObjectMatrix restituisce la trasformazione Camera -> World
    vision_sensor_handle = sim.getObject(SENSOR_NAME)
    matrix_list = sim.getObjectMatrix(vision_sensor_handle)
    matrix_coppelia = np.array(matrix_list).reshape((3, 4))
    transform_matrix = np.vstack([matrix_coppelia, [0, 0, 0, 1]]) # Rendiamola 4x4
    print(transform_matrix)
    # 2. La Matrice "Fix" (Correzione Assi)
    #    Open3D ha Y-giù, Coppelia ha Y-su. Dobbiamo riflettere Y.
    #    Matrice diagonale: X=1, Y=-1, Z=1 (profondità invariata)
    fix_matrix = np.eye(4)
    fix_matrix[1, 1] = -1

    # 3. Combina le matrici
    #    L'ordine è fondamentale: PRIMA applichi il fix locale, POI sposti nel mondo.
    #    In algebra lineare: T_finale = T_coppelia * T_fix
    final_transform = transform_matrix @ fix_matrix
    pcd.transform(final_transform)
    o3d.visualization.draw_geometries([pcd, world_frame])
    # Salva
    o3d.io.write_point_cloud(filename, pcd)
    print(f"Salvataggio completato: {filename}")


# --- ESECUZIONE ---
rgb, depth, resolution = get_data_from_sim()

print(f"Acquisita immagine: {resolution[0]}x{resolution[1]}")

# Salva i file per il tuo script di grasping
cv2.imwrite("test_rgb.png", rgb)
np.save("test_depth.npy", depth)  # Salviamo la matrice float precisa
save_point_cloud(rgb, depth)

print("Tutto salvato! Ora puoi usare test_depth.npy nel tuo script.")