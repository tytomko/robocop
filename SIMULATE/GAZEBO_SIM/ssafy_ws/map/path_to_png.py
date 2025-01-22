import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from PIL import Image

# 필요한 라이브러리 설치
os.system("pip install utm pillow")  # UTM 및 Pillow 설치
import utm  # UTM 변환을 위한 라이브러리

# NumPy 버전 고정 (버전 체크 후 자동 변경)
required_numpy_version = "1.26.4"
if np.__version__ != required_numpy_version:
    print(f"현재 NumPy 버전: {np.__version__}, {required_numpy_version}로 변경이 필요합니다.")
    os.system(f"pip install numpy=={required_numpy_version} --force-reinstall")
    exit("NumPy 버전이 업데이트되었습니다. 스크립트를 다시 실행하세요.")

# Gazebo 맵 중앙 (위도, 경도) -> UTM 변환
latitude = 35.1595
longitude = 126.8526

# UTM 변환 (zone 정보 포함)
utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon(latitude, longitude)
print(f"Gazebo 중앙 UTM 좌표: ({utm_x}, {utm_y}) Zone: {utm_zone}{utm_letter}")

# CSV 파일 로드
csv_file = "patrol_path1.csv"

# 파일 존재 여부 확인
if not os.path.exists(csv_file):
    print(f"오류: '{csv_file}' 파일을 찾을 수 없습니다.")
    exit()

# CSV 데이터 로드 및 유효성 검사
try:
    df = pd.read_csv(csv_file)

    # 데이터 정리 및 타입 변환 (NaN 값 제거)
    df.dropna(inplace=True)
    df['x'] = pd.to_numeric(df['x'], errors='coerce')
    df['y'] = pd.to_numeric(df['y'], errors='coerce')

    if df.isnull().values.any():
        print("오류: CSV 파일에 유효하지 않은 좌표 값이 포함되어 있습니다.")
        exit()

except Exception as e:
    print(f"오류 발생: {e}")
    exit()

# Gazebo 중앙 좌표를 (0,0)으로 변환하여 조정
df['x'] = df['x'] - utm_x
df['y'] = df['y'] - utm_y

# 이미지 중앙에 맞추기 위한 보정
x_min, x_max = df['x'].min(), df['x'].max()
y_min, y_max = df['y'].min(), df['y'].max()

# 경로의 시작점이 이미지 중앙에 오도록 조정
df['x'] = df['x'] - (x_min + x_max) / 2
df['y'] = df['y'] - (y_min + y_max) / 2

# 좌표 시각화 (NumPy 배열로 변환하여 오류 방지)
plt.figure(figsize=(10, 10))

# 경로 두께 조정 및 색상 설정 (두꺼운 선, 빨간색)
plt.plot(df['x'].to_numpy(), df['y'].to_numpy(), 'r-', linewidth=5, label='Path')

# 그래프 설정 (중앙 배치 및 여백 조정)
margin = 10  # 여백 추가
plt.xlim(df['x'].min() - margin, df['x'].max() + margin)
plt.ylim(df['y'].min() - margin, df['y'].max() + margin)

# 축 및 불필요한 요소 제거
plt.axis('off')  # 축 숨김
plt.grid(False)  # 그리드 제거
plt.legend().set_visible(False)  # 범례 숨김

# 이미지 저장 (PNG - 투명한 배경 포함)
output_file_png = "path_visualization.png"
plt.savefig(output_file_png, dpi=300, bbox_inches='tight', transparent=True)
print(f"경로 이미지가 '{output_file_png}'로 저장되었습니다.")

# PNG 이미지를 RGBA 포맷으로 변환 및 확인
img = Image.open(output_file_png).convert("RGBA")
rgba_output_file = "path_visualization_rgba.png"
img.save(rgba_output_file, format="PNG")
print(f"RGBA 포맷 이미지가 '{rgba_output_file}'로 저장되었습니다.")

# DDS 포맷으로 변환 (Gazebo 호환용)
dds_output_file = "path_visualization.dds"
img.save(dds_output_file, format="DDS")
print(f"DDS 포맷 이미지가 '{dds_output_file}'로 저장되었습니다.")

# 이미지 표시
plt.show()
