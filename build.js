// build.js
const mapboxToken = process.argv[2]; // Read the first command-line argument

// Store the mapboxToken variable on the window object to make it accessible globally
window.mapboxToken = mapboxToken;

// Use the 'mapboxToken' variable in your build process or pass it to your frontend code
console.log('Mapbox Token:', mapboxToken);
