import os
from moviepy.editor import VideoFileClip, vfx

# 전역변수 설정 (필요에 따라 아래 값을 수정하세요)
SAVEGIF = True                              # True이면 gif로 저장, False이면 mp4로 저장
VIDEO_PATH = '/home/ubuntu/your_video.mp4'    # 원본 mp4 파일 위치
OUTPUT_FILENAME = 'fast_video'              # 저장할 파일명 (확장자 제외)
SPEED_FACTOR = 2.0                          # 몇 배속 (예: 2.0이면 2배속)
SAVE_PATH = '/home/ubuntu/fastvideo'         # 저장 위치 (항상 이 경로)

def change_video_speed():
    # 저장 경로가 없으면 생성합니다.
    if not os.path.exists(SAVE_PATH):
        os.makedirs(SAVE_PATH)
    
    # SAVEGIF 값에 따라 확장자를 붙입니다.
    if SAVEGIF:
        output_file = os.path.join(SAVE_PATH, OUTPUT_FILENAME + '.gif')
    else:
        output_file = os.path.join(SAVE_PATH, OUTPUT_FILENAME + '.mp4')
    
    print(f"원본 파일: {VIDEO_PATH}")
    print(f"저장 파일: {output_file}")
    print(f"속도 변경 배수: {SPEED_FACTOR}")
    
    # 비디오 파일 불러오기
    clip = VideoFileClip(VIDEO_PATH)
    
    # 속도 변경 (moviepy의 vfx.speedx 함수 사용)
    sped_up_clip = clip.fx(vfx.speedx, factor=SPEED_FACTOR)
    
    # SAVEGIF 값에 따라 gif 또는 mp4로 저장
    if SAVEGIF:
        sped_up_clip.write_gif(output_file)
    else:
        sped_up_clip.write_videofile(output_file, codec='libx264')
    
    # 클립 리소스 해제
    clip.close()
    sped_up_clip.close()

if __name__ == "__main__":
    change_video_speed()
