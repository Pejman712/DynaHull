from LiDAR_ground_removal.module.ground_removal import Processor
import numpy as np
from sklearn.neighbors import KDTree
import open3d as o3d
from scipy.spatial import ConvexHull
import time
import glob
import os
from sklearn.cluster import KMeans
import configparser  


# Read configuration from config file
config = configparser.ConfigParser()
config.read('config.ini')

# Algorithm Configuration
use_algorithm = config.getboolean('Algorithm', 'use_algorithm')
neighbor_counts = config.get('Algorithm', 'neighbor_counts').split(',')
num_clusters = config.getint('Algorithm', 'num_clusters')
min_removal = config.get('Algorithm', 'min_removal').split(',')
max_remove = config.getint('Algorithm', 'max_removal')
max_height = config.getfloat('Algorithm', 'max_height')
point_cloud_directory = config.get('Algorithm', 'point_cloud_directory')
density_method = config.get('Algorithm', 'density_method')
output_directory = config.get('Algorithm', 'output_directory')
point_display_size = config.getint('Algorithm', 'point_display_size')
normalized_distance = config.getboolean('Algorithm', 'normalized_distance')

neighbor_counts = [int(count) for count in neighbor_counts]
min_removal = [int(count) for count in min_removal]

# Initialize final_point_cloud as an empty array
final_point_cloud = np.array([]).reshape(0, 3)

def Stationary_Dynamic_visualization(source, target):
    source.paint_uniform_color([1, 0, 0])
    target.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([source, target])

def capture_screen_image(source, image_name, output_directory, point_display_size):
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window()
    render_option = visualizer.get_render_option()
    render_option.point_size = point_display_size
    visualizer.add_geometry(source)
    captured_image = visualizer.capture_screen_image(f"{output_directory}/{image_name}.png", do_render=True)
    visualizer.destroy_window()

def filter_by_density(point_cloud_data, density_values, removal_percentage):
    # Sort by density factor in ascending order
    sorted_indices = np.argsort(density_values)
    sorted_points = point_cloud_data[sorted_indices]
    
    # Calculate the number of points to retain based on the removal percentage
    num_points_to_retain = int((removal_percentage) / 100 * len(sorted_points))
    dynamic_points = sorted_points[:num_points_to_retain]
    
    return dynamic_points, sorted_indices[:num_points_to_retain]

def display_progress_bar(current_value, total_value, bar_length, progress_char):
    completion_percentage = int((current_value / total_value) * 100)
    progress = int((bar_length * current_value) / total_value)
    print(f"Progress: [{'■' * progress:<{bar_length}}]{completion_percentage}%", end='\r')

# Main Code

sorted_files = sorted(glob.glob(os.path.join(point_cloud_directory, "*.pcd")))
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
    
filtered_point_cloud1 = final_point_cloud[final_point_cloud[:, 2] <= max_height]
process = Processor(n_segments=500, n_bins=500, line_search_angle=0.3, max_dist_to_line=0.02,
                    sensor_height=0.1, max_start_height=0.8, long_threshold=15)
filtered_point_cloud = process(filtered_point_cloud1)
final_point_cloud_output1 = o3d.geometry.PointCloud()
final_point_cloud_output1.points = o3d.utility.Vector3dVector(filtered_point_cloud)
start_time = time.time()
kmeans = KMeans(n_clusters=num_clusters, n_init=10)
labels = kmeans.fit_predict(filtered_point_cloud)
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
        max_remove = min_remove + 15
        
        # Rescale the array to a new range
        rescaled_array = (
            (np_cluster_point_counts - min_count) / (max_count - min_count)
        ) * (max_remove - min_remove) + min_remove

        for index, cluster in enumerate(clusters):
            display_progress_bar((index+1)/ len(clusters), 1, 30, '■')
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
                point_density = [neighbor_count / ConvexHull(cluster[neighbors],qhull_options='QJ').volume for neighbors in nearest_indices]
            elif density_method == "Average":
                point_density = neighbor_count / ((4 / 3) * np.pi * np.power(np.average(nearest_distances, axis=1), 3))

            density_array = np.array(point_density)

            removal_percentage = rescaled_array[index]
            if use_algorithm:
                dynamic_points, dynamic_indices = filter_by_density(cluster, density_array, removal_percentage)
                mask = np.ones(cluster.shape[0], dtype=bool)
                mask[dynamic_indices] = False
                static_points = cluster[mask]
            
            if index == 0:
                final_dynamic_cloud = dynamic_points
                final_static_cloud = static_points
            else:
                final_dynamic_cloud = np.concatenate((final_dynamic_cloud , dynamic_points), axis=0)
                final_static_cloud = np.concatenate((final_static_cloud , static_points), axis=0)
        
        elapsed_time =  time.time()- start_time
        print(f"\nProcessing time: { elapsed_time:.2f}s")
        
        final_point_cloud_output_dynamic = o3d.geometry.PointCloud()
        final_point_cloud_output_dynamic.points = o3d.utility.Vector3dVector(final_dynamic_cloud)
        
        final_point_cloud_output_static = o3d.geometry.PointCloud()
        final_point_cloud_output_static.points = o3d.utility.Vector3dVector(final_static_cloud)
        
        o3d.io.write_point_cloud(f"{output_directory}/dynamic_points.pcd", final_point_cloud_output_dynamic)
        o3d.io.write_point_cloud(f"{output_directory}/static_points.pcd", final_point_cloud_output_static)
