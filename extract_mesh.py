import os, json
import subprocess
from tqdm import tqdm
import argparse
import numpy as np
import open3d as o3d
import trimesh
import math
import plyfile

from point2cad.utils import rotation_matrix_a_to_b


argparser = argparse.ArgumentParser()

parser = argparse.ArgumentParser(description="Point2CAD extract mesh")
parser.add_argument("--root", type=str, default="./assets", help="Network prediction root")
parser.add_argument("--output_dir", type=str, default="./out", help="Mesh output dir")
args = parser.parse_args()

exe = "python"
root = args.root
output_dir = args.output_dir

EPS = np.finfo(np.float32).eps

def transform_mesh(root, point_dir):
    tasks = sorted([item[:8] for item in os.listdir(root)])
    status = []
    outs = []

    for prefix in tqdm(tasks):
        points = np.loadtxt(os.path.join(point_dir, "{}.xyzc".format(prefix)))[:, :3]

        mean_points = np.mean(points, 0, keepdims=True)
        points = points - mean_points

        S, U = np.linalg.eig(points.T @ points)
        smallest_ev = U[:, np.argmin(S)]
        R = rotation_matrix_a_to_b(smallest_ev, np.array([1, 0, 0]))
        # rotate input points such that the minor principal
        # axis aligns with x axis.
        points = (R @ points.T).T
        std = np.max(points, 0) - np.min(points, 0)
        points = points / (np.max(std) + EPS)
        pcd=o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points))
        o3d.io.write_point_cloud(os.path.join(root, "{}/clipped/points.ply".format(prefix)), pcd)

        if not os.path.exists(os.path.join(root, "{}/clipped/mesh.ply".format(prefix))):
            continue
        mesh = trimesh.load(os.path.join(root, "{}/clipped/mesh.ply".format(prefix)))
        vertices = np.asarray(mesh.vertices)
        vertices = (R.T @ (vertices * (np.max(std) + EPS)).T).T + mean_points # Transformation that Point2CAD conducts

        mesh.vertices = vertices
        mesh.export(os.path.join(root, "{}/clipped/mesh_transformed.ply".format(prefix)))

        # Curves and corners
        # mesh = o3d.io.read_triangle_mesh(os.path.join(root, "{}/clipped/viz_curves.obj".format(prefix)))
        # curve_points = np.asarray(mesh.vertices)
        # curve_points = ((R.T @ (curve_points * (np.max(std) + EPS)).T).T + mean_points) * (original_diag / cur_diag)
        # mesh.vertices = o3d.utility.Vector3dVector(curve_points)
        # o3d.io.write_triangle_mesh(os.path.join(root, "{}/clipped/viz_curves_transformed.obj".format(prefix)), mesh)

        # mesh = o3d.io.read_triangle_mesh(os.path.join(root, "{}/clipped/viz_corners.obj".format(prefix)))
        # corners_points = np.asarray(mesh.vertices)
        # corners_points = ((R.T @ (corners_points * (np.max(std) + EPS)).T).T + mean_points) * (original_diag / cur_diag)
        # mesh.vertices = o3d.utility.Vector3dVector(corners_points)
        # o3d.io.write_triangle_mesh(os.path.join(root, "{}/clipped/viz_corners_transformed.obj".format(prefix)), mesh)
        # continue

        curve_points = []
        corners = []
        new_json = {"curves": [], "corners": []}
        with open(os.path.join(root, "{}/topo/topo.json".format(prefix)), "r") as f:
            curves = json.load(f)
            num_curves = len(curves["curves"])
            num_corners = len(curves["corners"])
            curves

            for i in range(num_curves):
                pv_points = np.asarray(curves["curves"][i]["pv_points"]).astype(np.float32)
                pv_points = ((R.T @ (pv_points * (np.max(std) + EPS)).T).T + mean_points)
                new_json["curves"].append({"pv_points": pv_points.tolist()})
                pv_lines = np.asarray(curves["curves"][i]["pv_lines"]).astype(np.int32)
                new_json["curves"][-1]["pv_lines"] = curves["curves"][i]["pv_lines"]
                new_json["curves"][-1]["index"] = curves["curves"][i]["index"]
                points = pv_points[pv_lines]

                length = np.sqrt(((points[:, 1] - points[:, 0])**2).sum(1))
                num_total_points = int(np.sum(length) * 1000)
                points = np.concatenate([np.linspace(points[i, 0], points[i, 1], np.ceil(length[i] * 1000).astype(np.int32)) for i in range(length.shape[0])], 0)
                points = np.concatenate((points, np.ones_like(points[:, 0:1]) * i), 1)
                curve_points.append(points)
            
            num_unique_corners = 0
            eps = 0.0001
            remove_duplicates_map = {}
            remove_duplicated_corners = []
            for i in range(num_corners):
                corner = np.asarray(curves["corners"][i]["corner"]).astype(np.float32)
                is_duplicate = False
                for j in range(i):
                    prev_corner = np.asarray(curves["corners"][j]["corner"]).astype(np.float32)
                    distance = np.sqrt(((corner - prev_corner)**2).sum())
                    if distance < eps:
                        is_duplicate = True
                        remove_duplicates_map[i] = remove_duplicates_map[j]
                        break

                if not is_duplicate:
                    remove_duplicates_map[i] = num_unique_corners
                    num_unique_corners += 1

                corner = ((R.T @ (corner * (np.max(std) + EPS)).T).T + mean_points)
                corners.append(corner)
                if not is_duplicate:
                    remove_duplicated_corners.append(corner)
                new_json["corners"].append({"corner": corner.tolist()})
                new_json["corners"][-1]["index"] = curves["corners"][i]["index"]

            if num_curves>0:
                curve_points = np.concatenate(curve_points, 0)
                np.savetxt(os.path.join(root, "{}/clipped/curve_points.xyzc".format(prefix)), curve_points)
                pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(curve_points[...,:3].reshape(-1,3)))
                o3d.io.write_point_cloud(os.path.join(root, "{}/clipped/curve_points.ply".format(prefix)), pcd)

            if num_corners>0:
                corners = np.concatenate(corners, 0)
                pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(corners.reshape(-1,3)))
                o3d.io.write_point_cloud(os.path.join(root, "{}/clipped/corners.ply".format(prefix)), pcd)

                remove_duplicates_corners = np.concatenate(remove_duplicated_corners, 0)
                pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(remove_duplicates_corners.reshape(-1,3)))
                o3d.io.write_point_cloud(os.path.join(root, "{}/clipped/remove_duplicates_corners.ply".format(prefix)), pcd)

        FE, EV = [], []
        with open(os.path.join(root, "{}/topo/topo.txt".format(prefix)), "r") as f:
            while True:
                line = f.readline()
                if line == "EV\n":
                    break
                if line == "FE\n":
                    continue
                FE.append([int(item) for item in line.split()[1:]])
            while True:
                line = f.readline()
                if len(line) == 0:
                    break
                EV.append([int(item) for item in line.split()[1:]])
            assert len(EV) == num_curves
            for i in range(num_curves):
                for j in range(len(EV[i])):
                    EV[i][j] = remove_duplicates_map[EV[i][j]]
                # remove duplicate number in temp
                EV[i] = np.unique(np.array(EV[i])).tolist()

        with open(os.path.join(root, "{}/topo/topo_fix.txt".format(prefix)), "w") as f:
            f.write("FE\n")
            for i in range(len(FE)):
                f.write(str(i))
                for item in FE[i]:
                    f.write(" {}".format(item))
                f.write("\n")
            f.write("EV\n")
            for i in range(len(EV)):
                f.write(str(i))
                for item in EV[i]:
                    f.write(" {}".format(item))
                f.write("\n")
        json.dump(new_json, open(os.path.join(root, "{}/topo/topo_transformed.json".format(prefix)), "w"))
        continue
    pass


def check_done(prefix):
    try:
        with open(os.path.join(output_dir, prefix, prefix + ".txt"), "r") as f:
            content = f.read()
            return "Done" in content
    except:
        return False

if __name__ == "__main__":
    if root != "./assets":
        tasks = [item[:8] for item in os.listdir(root) if item.endswith(".xyzc")]
    else:
        tasks = [item[:9] for item in os.listdir(root) if item.endswith(".xyzc")]

    status = []
    outs = []

    os.makedirs(output_dir, exist_ok=True)
    for prefix in tqdm(tasks):
        if check_done(prefix):
            continue
        os.makedirs(os.path.join(output_dir, prefix), exist_ok=True)
        out = open(os.path.join(output_dir, prefix, "{}.txt".format(prefix)), "w")
        process = subprocess.Popen([
            exe,
            "-m",
            "point2cad.main",
            "--max_parallel_surfaces", "4",
            "--path_in", os.path.join(root, "{}.xyzc".format(prefix)),
            "--path_out", os.path.join(output_dir, prefix),
        ], stdout=out, stderr=out)
        process.wait()
        out.close()


    # fix CUDA out of memory
    for prefix in tqdm(tasks):
        if check_done(prefix):
            continue
        os.makedirs(os.path.join(output_dir, prefix), exist_ok=True)
        out = open(os.path.join(output_dir, prefix, "{}.txt".format(prefix)), "w")
        process = subprocess.Popen([
            exe,
            "-m",
            "point2cad.main",
            "--max_parallel_surfaces", "1",
            "--path_in", os.path.join(root, "{}.xyzc".format(prefix)),
            "--path_out", os.path.join(output_dir, prefix),
        ], stdout=out, stderr=out)
        process.wait()
        out.close()


    transform_mesh(args.output_dir, args.root)
    
    pass
