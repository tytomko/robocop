/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{vue,js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      screens: {
        'sm': '600px',
        'md': '870px',
        'lg': '1210px',  // 기존 1024px에서 1280px로 변경
        'xl': '1540px',
      },
    },
  },
  plugins: [],
}
