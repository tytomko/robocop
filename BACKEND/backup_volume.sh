#!/bin/bash

# 백업 디렉토리 생성
BACKUP_DIR="/backup/robocop"
mkdir -p $BACKUP_DIR

# 현재 날짜로 백업 파일명 생성
BACKUP_FILE="$BACKUP_DIR/robocop_storage_$(date +%Y%m%d_%H%M%S).tar"

# 볼륨 데이터 백업
docker run --rm \
  -v robocop_storage:/data \
  -v $BACKUP_DIR:/backup \
  alpine tar cvf /backup/$(basename $BACKUP_FILE) /data

# 30일 이상 된 백업 파일 삭제
find $BACKUP_DIR -name "robocop_storage_*.tar" -mtime +30 -delete 