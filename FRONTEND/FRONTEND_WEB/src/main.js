import { createApp } from 'vue'
import { createPinia } from 'pinia'
import App from './App.vue'
import router from './router'
import Toast from 'vue-toastification'
import "vue-toastification/dist/index.css"
import './assets/toast.css'  
import '@fortawesome/fontawesome-free/css/all.css'
import './assets/tailwind.css'

const app = createApp(App)
const pinia = createPinia()

const options = {
    position: "top-right",
    timeout: 3000,
    closeOnClick: true,
    pauseOnHover: true,
    draggable: true
}

app.use(router)
app.use(pinia)
app.use(Toast, options)
app.mount('#app')