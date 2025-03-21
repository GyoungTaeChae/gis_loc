import open3d as o3d
import numpy as np
import os
import matplotlib.pyplot as plt

# 입력 및 출력 폴더 경로
input_folder = "/home/cgt20/WorkSpace/catkin4_ws/src/my_pointcloud_processor/pcds/"
output_folder = os.path.join(input_folder, "jpg")

# 출력 폴더가 존재하지 않으면 생성
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# 폴더 내 모든 PCD 파일 처리
for filename in os.listdir(input_folder):
    if filename.endswith(".pcd"):
        # PCD 파일 경로
        pcd_path = os.path.join(input_folder, filename)
        
        # PCD 파일 읽기
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points)

        if points.size == 0:  # 포인트 클라우드가 비어 있는 경우 건너뛰기
            print(f"빈 파일: {filename}, 처리 건너뜀")
            continue

        # x, y 좌표만 추출
        points_xy = points[:, :2]

        # x, y 범위 계산
        x_min, x_max = points_xy[:, 0].min(), points_xy[:, 0].max()
        y_min, y_max = points_xy[:, 1].min(), points_xy[:, 1].max()

        # 포인트를 400x400 이미지의 픽셀 좌표로 변환
        image_size = 400
        pixel_x = ((points_xy[:, 0] - x_min) / (x_max - x_min) * (image_size - 1)).astype(int)
        pixel_y = ((points_xy[:, 1] - y_min) / (y_max - y_min) * (image_size - 1)).astype(int)

        # 이미지 생성 및 포인트 표시
        image = np.full((image_size, image_size), 255, dtype=np.uint8)
        for x, y in zip(pixel_x, pixel_y):
            for dx in range(-10, 11):  # -10부터 10까지
                for dy in range(-10, 11):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < image_size and 0 <= ny < image_size:
                        image[image_size - 1 - ny, nx] = 0

        # 출력 파일 경로 설정 (확장자를 .jpg로 변경)
        output_path = os.path.join(output_folder, filename.replace(".pcd", ".jpg"))

        # 이미지 저장
        plt.imsave(output_path, image, cmap='gray', format='jpg')
        print(f"'{filename}' 처리 완료, 저장 경로: '{output_path}'")

print("모든 PCD 파일 처리가 완료되었습니다.")
