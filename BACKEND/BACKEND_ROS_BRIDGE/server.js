const express = require('express')
const path = require('path')

const app = express()
const SERVER_PORT = 3000

// public folder to use static files
app.use('/static', express.static(path.join(__dirname, "public")))

// '/' 경로 라우트
app.get('/', (req, res) => {
    res.sendFile(__dirname + '/public/html/index.html')
})

// 404 처리 (위에서 매칭되지 않은 모든 경로)
app.use((req, res) => {
    res.status(404).send('Not Found')
})

const HTTPServer = app.listen(SERVER_PORT, () => {
    console.info(`Server is running on ${SERVER_PORT} port`)
})
