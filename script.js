// Get the favicon link element
var favicon = document.querySelector("link[rel*='icon']") || document.createElement('link');
favicon.type = 'image/png';
favicon.rel = 'icon';

// Set the initial favicon
favicon.href = 'icons/0.png';
document.head.appendChild(favicon);

// Titles array
var titles = ["Ludum Vortex  ꒰ ꒡⌓꒡꒱","Ludum Vortex  ^_____^","Ludum Vortex  `(*>﹏<*)′","Ludum Vortex  ヾ(⌐■_■)ノ♪","Ludum Vortex  ༼ つ ◕_◕ ༽つ","Ludum Vortex  (っˆڡˆς)","Ludum Vortex ヾ(≧▽≦*)o","Ludum Vortex  (✿◡‿◡)"];

// Function to shuffle an array (Fisher-Yates shuffle algorithm)
function shuffleArray(array) {
    for (let i = array.length - 1; i > 0; i--) {
        // Pick a remaining element
        let j = Math.floor(Math.random() * (i + 1));
        // And swap it with the current element
        [array[i], array[j]] = [array[j], array[i]];
    }
}

function isRootPage() {
    // Obtain the pathname from the URL (e.g., "/", "/index.html", or "/somepath/")
    const pathname = window.location.pathname;
    // Check if pathname is either the root "/" or "/index.html"
    return pathname === "/" || pathname === "/index.html" || pathname.match(/^\/index.html$/);
}

// Ensure the favicon and title indexes are in sync
function loopFaviconAndTitle() {
    var i = 0;
    setInterval(function() {
        // Update the favicon
        favicon.href = 'icons/' + i + '.png';
        // Update the document title
      if (isRootPage()) {
      // Update the document title only if on the root page
        document.title = titles[i];
      }
      i = (i + 1) % titles.length; // Loop through the array
    }, 2000); // Change every 1000 milliseconds (1 second)
}

shuffleArray(titles);

// Start the favicon and title loop
loopFaviconAndTitle();
