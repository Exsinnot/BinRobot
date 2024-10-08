/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      screens: {
        'portrait-mobile': {'raw': '(max-width: 390px) and (max-height: 844px) and (orientation: portrait)'},
        'landscape-mobile': {'raw': '(max-width: 844px) and (max-height: 390px) and (orientation: landscape)'},
        'portrait-tablet': {'raw': '(max-width: 768px) and (max-height: 1024px) and (orientation: portrait)'},
        'landscape-tablet': {'raw': '(max-width: 1024px) and (max-height: 768px) and (orientation: landscape)'},
        'portrait-desktop': {'raw': '(max-width: 1024px) and (max-height: 1366px) and (orientation: portrait)'},
        'landscape-desktop': {'raw': '(max-width: 1366px) and (max-height: 1024px) and (orientation: landscape)'},
      },
    },
  },
  plugins: [],
}

