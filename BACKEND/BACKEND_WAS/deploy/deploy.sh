#!/bin/bash

# 배포 디렉토리 설정
DEPLOY_DIR="/home/ubuntu/robocop"
BACKEND_DIR="$DEPLOY_DIR/backend"

# 필요한 디렉토리 생성
mkdir -p $BACKEND_DIR

# Docker 볼륨 생성 (없는 경우)
docker volume create robocop_storage

# Docker Compose 파일 생성
cat > $BACKEND_DIR/docker-compose.yml << 'EOF'
version: '3'
services:
  backend:
    build: .
    ports:
      - "8000:8000"
    volumes:
      - ./app:/BACKEND_WAS/app
      - robocop_storage:/data/storage
    environment:
      - STORAGE_PATH=/data/storage
    networks:
      - robocop_network

volumes:
  robocop_storage:
    name: robocop_storage

networks:
  robocop_network:
    name: robocop_network
EOF

# .env 파일 생성
cat > $BACKEND_DIR/.env << 'EOF'
MONGODB_URL=mongodb+srv://maybecold:SsCi6JKTPgXLj2LK@cluster0.5idlh.mongodb.net/
STORAGE_PATH=/data/storage
EOF

# 백엔드 코드 복사
cp -r BACKEND_WAS/* $BACKEND_DIR/

# Docker 컨테이너 재시작
cd $BACKEND_DIR
docker-compose down
docker-compose up -d

# 권한 설정
chmod +x $DEPLOY_DIR/backup_volume.sh

# 백업 cron job 설정
(crontab -l 2>/dev/null; echo "0 3 * * * $DEPLOY_DIR/backup_volume.sh") | crontab - 