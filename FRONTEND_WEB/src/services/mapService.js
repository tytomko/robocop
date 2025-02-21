class MapService {
  constructor() {
    this.mapCache = new Map()
  }

  // PNG 형식의 지도 이미지를 로드하고 캔버스에서 사용할 수 있는 데이터로 변환
  async loadMapImage(url) {
    if (this.mapCache.has(url)) {
      return this.mapCache.get(url)
    }

    return new Promise((resolve, reject) => {
      const img = new Image()
      img.crossOrigin = 'anonymous'
      
      img.onload = () => {
        const canvas = document.createElement('canvas')
        canvas.width = img.width
        canvas.height = img.height
        const ctx = canvas.getContext('2d')
        ctx.drawImage(img, 0, 0)
        
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height)
        const mapData = {
          width: img.width,
          height: img.height,
          resolution: 0.05, // 기본값, 서버에서 받아와야 함
          origin: { x: 0, y: 0 }, // 기본값, 서버에서 받아와야 함
          data: new Uint8Array(img.width * img.height)
        }

        // RGBA 데이터를 그레이스케일로 변환
        for (let i = 0; i < imageData.data.length; i += 4) {
          const gray = Math.round((imageData.data[i] + imageData.data[i + 1] + imageData.data[i + 2]) / 3)
          mapData.data[i / 4] = gray
        }

        this.mapCache.set(url, mapData)
        resolve(mapData)
      }

      img.onerror = reject
      img.src = url
    })
  }

  // YAML/JSON 형식의 지도 메타데이터 로드
  async loadMapMetadata(url) {
    const response = await fetch(url)
    const metadata = await response.json()
    return metadata
  }

  // 지도 데이터와 메타데이터 결합
  async loadMap(imageUrl, metadataUrl) {
    try {
      const [mapData, metadata] = await Promise.all([
        this.loadMapImage(imageUrl),
        this.loadMapMetadata(metadataUrl)
      ])

      return {
        ...mapData,
        resolution: metadata.resolution,
        origin: metadata.origin
      }
    } catch (error) {
      console.error('지도 로드 실패:', error)
      throw error
    }
  }

  // 월드 좌표를 픽셀 좌표로 변환
  worldToPixel(worldX, worldY, mapData) {
    const pixelX = Math.round((worldX - mapData.origin.x) / mapData.resolution)
    const pixelY = Math.round((worldY - mapData.origin.y) / mapData.resolution)
    return { x: pixelX, y: mapData.height - pixelY } // Y축 반전
  }

  // 픽셀 좌표를 월드 좌표로 변환
  pixelToWorld(pixelX, pixelY, mapData) {
    const worldX = pixelX * mapData.resolution + mapData.origin.x
    const worldY = (mapData.height - pixelY) * mapData.resolution + mapData.origin.y
    return { x: worldX, y: worldY }
  }
}

export const mapService = new MapService() 