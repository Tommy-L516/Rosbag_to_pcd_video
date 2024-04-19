
"""
    功能：将ROS bag文件中的点云数据转换为PCD文件，并将PCD文件转换为图像，最后生成视频。
    输入：rosbag文件路径，输出路径
    输出：生成的视频文件
    注意：需要安装pyntcloud、matplotlib、opencv库
"""
    
    
import sys
import matplotlib.pyplot as plt
import cv2
import os
import subprocess
import matplotlib.pyplot as plt
from pyntcloud import PyntCloud
import rosbag
import shutil

def process_pcd_to_top_view_image(pcd_path, output_path):
    # 使用 PyntCloud 读取点云数据
    cloud = PyntCloud.from_file(pcd_path)
    points = cloud.points
    
    plt.figure(figsize=(10, 10))
    ax = plt.axes(projection='3d')
    ax.scatter(points['x'], points['y'], points['z'], s=1, color='blue')
    ax.view_init(elev=90, azim=90)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    plt.axis('off')  # 隐藏坐标轴
    plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
    plt.close()

def create_video_from_images(image_folder, output_video_path, frame_rate):
    images = [img for img in sorted(os.listdir(image_folder)) if img.endswith(".jpg")]
    if not images:
        print("没有找到任何JPG图像，请检查图像文件夹！")
        return

    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape
    video = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc(*'mp4v'), float(frame_rate), (width, height))

    for image in images:
        frame = cv2.imread(os.path.join(image_folder, image))
        if frame is not None:
            video.write(frame)

    video.release()
    print(f"视频已成功保存在：{output_video_path}")

def convert_bag_to_pcd(input_dir, output_dir):
    """ 将ROS bag文件中的点云数据转换为PCD文件 """
    command = f'rosrun pcl_ros bag_to_pcd {input_dir} /scan_map_icp_amcl_node/scan_point_transformed {output_dir}'
    process = subprocess.run(command, shell=True, text=True, capture_output=True)
    if process.returncode != 0:
        print("转换失败:", process.stderr)
    else:
        print("PCD文件已成功生成")

if __name__ == '__main__':
    
    rosbag_path = sys.argv[1]
    output_base_path = sys.argv[2]
        
    #input_dir = '/media/server/00D7-0524/点云解析/2024-04-16-10-20-45_obstacledetection_30.bag'
    #output_dir = '/media/server/00D7-0524/点云解析/2024-04-16-10-20-45_obstacledetection_30'

    bag_base_name = rosbag_path.split('/')[-1].split('.')[0]
    
    lidar_output_base = os.path.join(output_base_path, bag_base_name + '_pcd_lidar')
    pcd_folder_path = os.path.join(lidar_output_base, 'pcd')
    images_folder_path = os.path.join(lidar_output_base, 'images')
    
    if not os.path.exists(pcd_folder_path):
        os.makedirs(pcd_folder_path)
    if not os.path.exists(images_folder_path):
        os.makedirs(images_folder_path)
    
    
    with rosbag.Bag(rosbag_path ,'r') as bag:
        num_count=0
        for topic,msg,t in bag.read_messages():
            if topic == "/scan_map_icp_amcl_node/scan_point_transformed":
                num_count+=1
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        # 计算rosbag的持续时间
        duration = end_time - start_time

    bagtime = duration
    hz=str(float(float(num_count)/bagtime))
    print(hz)


    # 转换ROS bag到PCD
    convert_bag_to_pcd(rosbag_path, pcd_folder_path)
    
    pcd_files = [f for f in os.listdir(pcd_folder_path) if f.endswith('.pcd')]
    for file_name in pcd_files:
        pcd_file_path = os.path.join(pcd_folder_path, file_name)
        image_file_path = os.path.join(images_folder_path, f"{file_name[:-4]}.jpg")
        process_pcd_to_top_view_image(pcd_file_path, image_file_path)

    # 从图像生成视频
    video_path = os.path.join(lidar_output_base, 'output_video.mp4')
    create_video_from_images(images_folder_path, video_path, hz)
    
    # 清理临时文件
    shutil.rmtree(pcd_folder_path)
    shutil.rmtree(images_folder_path)
