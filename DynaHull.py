from LiDAR_ground_removal.module.ground_removal import Processor
import numpy as np
from sklearn.neighbors import KDTree
import open3d as o3d
from scipy.spatial import ConvexHull
import time
import glob
import os
from sklearn.cluster import KMeans
import configparser  # Import configparser

# Read configuration from config file
config = configparser.ConfigParser()
config.read('config.ini')

# Algorithm Configuration
use_algorithm = config.getboolean('Algorithm', 'use_algorithm')
voxel_resolution = config.getfloat('Algorithm', 'voxel_resolution')
neighbor_counts = config.get('Algorithm', 'neighbor_counts').split(',')
neighbor_counts = [int(count) for count in neighbor_counts]
num_clusters = config.getint('Algorithm', 'num_clusters')
min_removal = config.getint('Algorithm', 'min_removal')
layer_index = config.getint('Algorithm', 'layer_index')
max_height = config.getfloat('Algorithm', 'max_height')
point_cloud_directory = config.get('Algorithm', 'point_cloud_directory')
density_method = config.get('Algorithm', 'density_method')
output_directory = config.get('Algorithm', 'output_directory')
point_display_size = config.getint('Algorithm', 'point_display_size')
normalized_distance = config.getboolean('Algorithm', 'normalized_distance')

# Initialize final_point_cloud as an empty array
final_point_cloud = np.array([]).reshape(0, 3)

def Stationary_Dynamic_visualization(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0, 0])
    target_temp.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([source_temp, target_temp])

def compute_errors(source, target):
    tree = o3d.geometry.KDTreeFlann(target)
    errors = []
    for point in source.points:
        _, idx, _ = tree.search_knn_vector_3d(point, 1)
        nearest_point = np.asarray(target.points)[idx, :]
        dist = np.linalg.norm(np.array(point) - nearest_point)
        errors.append(dist)

    return np.array(errors)

def compute_hausdorff_distance(pcd1, pcd2):
    # KDTree construction
    tree2 = o3d.geometry.KDTreeFlann(pcd2)

    # Helper function to compute the maximum distance from points of pcd1 to pcd2
    def partial_hausdorff(pcd1, tree2):
        max_distance = 0.0
        for point in pcd1.points:
            _, _, dists = tree2.search_knn_vector_3d(point, 1)
            max_distance = max(max_distance, dists[0])
        return max_distance

    d1 = partial_hausdorff(pcd1, tree2)
    d2 = partial_hausdorff(pcd2, tree2)

    return max(d1, d2)

def compute_chamfer_distance(pcd1, pcd2):
    # KDTree construction
    tree1 = o3d.geometry.KDTreeFlann(pcd1)
    tree2 = o3d.geometry.KDTreeFlann(pcd2)

    # Helper function to compute the distance from points of pcd1 to pcd2
    def partial_chamfer(pcd1, tree2):
        distance = 0.0
        for point in pcd1.points:
            _, idx, _ = tree2.search_knn_vector_3d(point, 1)
            closest_point = np.asarray(pcd2.points)[idx[0]]
            distance += np.linalg.norm(point - closest_point)
        return distance / len(pcd1.points)

    d1 = partial_chamfer(pcd1, tree2)
    d2 = partial_chamfer(pcd2, tree1)

    return d1 + d2

def pairwise_distance(matrix):
    """Compute pairwise Euclidean distances for a set of points."""
    return np.linalg.norm(matrix[:, np.newaxis, :] - matrix[np.newaxis, :, :], axis=2)

def compute_gromov_hausdorff_distance(pcd1, pcd2):
    dist_matrix1 = pairwise_distance(pcd1)
    dist_matrix2 = pairwise_distance(pcd2)
    
    gh_distance = gd.bottleneck_distance(dist_matrix1, dist_matrix2)
    return gh_distance
# Function Definitions
def capture_screen_image(source, image_name, output_directory, point_display_size):
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window()
    render_option = visualizer.get_render_option()
    render_option.point_size = point_display_size
    visualizer.add_geometry(source)
    captured_image = visualizer.capture_screen_image(f"{output_directory}/{image_name}.png", do_render=True)
    visualizer.destroy_window()

def filter_by_density(point_cloud_data, density_values, normalized_obj_distances, threshold, iterations, step_size, num_neighbors, method):
    filtered_indices = [index for index, density in enumerate(density_values) if density * normalized_obj_distances[index] < threshold]
    filtered_points = np.delete(point_cloud_data, filtered_indices, axis=0)
    if iterations == 1:
        return filtered_points, filtered_indices, density_values
    else:
        return filter_by_density(filtered_points, threshold + step_size, iterations - 1, step_size, num_neighbors, method)

def display_progress_bar(current_value, total_value, bar_length, progress_char):
    completion_percentage = int((current_value / total_value) * 100)
    progress = int((bar_length * current_value) / total_value)
    print(f"Progress: [{'■' * progress:<{bar_length}}]{completion_percentage}%", end='\r')

# Main Code
start_time = time.time()
sorted_files = sorted(glob.glob(os.path.join(point_cloud_directory, "*.pcd")))
average_density = []
point_density = []
object_distances = []
if not sorted_files:
    print("No PCD files found in the directory.")
    exit()
for file_index, source_file_path in enumerate(sorted_files):
    point_cloud = o3d.io.read_point_cloud(source_file_path)
    point_cloud_data = np.asarray(point_cloud.points)

    if file_index == 0:
        final_point_cloud = point_cloud_data
    else:
        final_point_cloud = np.concatenate((final_point_cloud, point_cloud_data), axis=0)
    display_progress_bar((file_index+1)/ len(sorted_files), 1, 30, '■')
filtered_point_cloud1 = final_point_cloud[final_point_cloud[:, 2] <= max_height]
process = Processor(n_segments=500, n_bins=500, line_search_angle=0.3, max_dist_to_line=0.02,
                    sensor_height=0.1, max_start_height=0.8, long_threshold=15)
filtered_point_cloud = process(filtered_point_cloud1)
#filtered_point_cloud = filtered_point_cloud1
final_point_cloud_output1 = o3d.geometry.PointCloud()
final_point_cloud_output1.points = o3d.utility.Vector3dVector(filtered_point_cloud)
final_point_cloud_output = final_point_cloud_output1.voxel_down_sample(voxel_size=0.01)


kmeans = KMeans(n_clusters=num_clusters)
labels = kmeans.fit_predict(filtered_point_cloud)
clusterpoints = []
clusters = [filtered_point_cloud[labels == i] for i in range(num_clusters)]
# Calculate the number of points in each cluster and store them in a list
cluster_point_counts = [len(cluster) for cluster in clusters]

# Convert the list to a NumPy array for easier manipulation
np_cluster_point_counts = np.array(cluster_point_counts)

# Find the minimum and maximum values in the array
min_count = np_cluster_point_counts.min()
max_count = np_cluster_point_counts.max()


for neighbor_count in neighbor_counts:
    for min_remove in min_removal:
        max_remove = min_remove + 5
        # Rescale the array to a new range
        rescaled_array = (
            (np_cluster_point_counts - min_count) / (max_count - min_count)
        ) * (max_remove - min_remove) + min_remove



        # 3. Save each cluster into a new numpy array
        for index, cluster in enumerate(clusters):
            cluster_cloud = o3d.geometry.PointCloud()
            cluster_cloud.points = o3d.utility.Vector3dVector(cluster)
            kd_tree = KDTree(cluster)
            nearest_distances, nearest_indices = kd_tree.query(cluster, neighbor_count)
            cloud_center = cluster_cloud.get_center()
            object_distances = np.linalg.norm(cluster - cloud_center, axis=1)
            normalized_object_distances = np.array(object_distances)


            if density_method == "Max":
                point_density = neighbor_count / ((4 / 3) * np.pi * np.power(np.max(nearest_distances, axis=1), 3))
            elif density_method == "ConvexHull":
                point_density = [neighbor_count / ConvexHull(cluster[neighbors]).volume for neighbors in nearest_indices]
            elif density_method == "Average":
                point_density = neighbor_count / ((4 / 3) * np.pi * np.power(np.average(nearest_distances, axis=1), 3))

            density_array = np.array(point_density)

            start_threshold = 0
            std_deviation = int(np.std(density_array))
            end_threshold = int(np.average(density_array)) + std_deviation * 10

            iteration_count, step_size, num_neighbors = 1, std_deviation, neighbor_count
            removal_percentage = rescaled_array[index]
            if use_algorithm:
                for threshold in range(start_threshold, end_threshold, 100):
                    filtered_points, removed_indices, filtered_density = filter_by_density(cluster, density_array, normalized_object_distances, threshold, iteration_count, step_size, num_neighbors, density_method)
                    if 100 * len(removed_indices) / len(cluster_cloud.points) > removal_percentage:
                        elapsed_time = time.time() - start_time
                        break
            if index == 0:
                final_cloud = filtered_points
            else:
                final_cloud = np.concatenate((final_cloud , filtered_points), axis=0)

        final_point_cloud_output_mini = o3d.geometry.PointCloud()
        final_point_cloud_output_mini.points = o3d.utility.Vector3dVector(final_cloud)
        #o3d.visualization.draw_geometries([final_point_cloud_output_mini])
        o3d.io.write_point_cloud(f"{output_directory}", final_point_cloud_output_mini)