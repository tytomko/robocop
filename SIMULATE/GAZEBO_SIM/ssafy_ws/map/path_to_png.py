import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# NumPy 버전 고정 (버전 체크 후 자동 변경)
required_numpy_version = "1.26.4"
if np.__version__ != required_numpy_version:
    print(f"현재 NumPy 버전: {np.__version__}, {required_numpy_version}로 변경이 필요합니다.")
    os.system(f"pip install numpy=={required_numpy_version} --force-reinstall")
    exit("NumPy 버전이 업데이트되었습니다. 스크립트를 다시 실행하세요.")

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

# 좌표 시각화 (NumPy 배열로 변환하여 오류 방지)
plt.figure(figsize=(10, 10))

# 경로 두께 조정 및 색상 설정 (두꺼운 선)
plt.plot(df['x'].to_numpy(), df['y'].to_numpy(), 'r-', linewidth=5, label='Path')  # linewidth=5로 조정

# 그래프 설정 (경로 부분만 확대)
plt.xlim(df['x'].min() - 1, df['x'].max() + 1)
plt.ylim(df['y'].min() - 1, df['y'].max() + 1)

# 축, 그리드, 배경 제거
plt.axis('off')  # 축 숨김
plt.grid(False)  # 그리드 제거
plt.legend().set_visible(False)  # 범례 숨김

# 이미지 저장 및 표시
output_file = "path_visualization.png"
plt.savefig(output_file, dpi=300, bbox_inches='tight', transparent=True)  # 배경 투명 설정
print(f"경로 이미지가 '{output_file}'로 저장되었습니다.")
plt.show()
