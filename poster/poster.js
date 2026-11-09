const DESIGN_WIDTH = 1920;
const poster = document.getElementById('poster');
const stage = document.querySelector('.stage');

function fitPoster() {
  const available = Math.max(320, window.innerWidth - 32);
  const scale = Math.min(1, available / DESIGN_WIDTH);
  poster.style.transform = `scale(${scale})`;
  poster.style.transformOrigin = 'top left';
  stage.style.width = `${DESIGN_WIDTH * scale}px`;
  stage.style.height = `${poster.offsetHeight * scale}px`;
}

window.addEventListener('resize', fitPoster);
window.addEventListener('load', fitPoster);

if (document.fonts && document.fonts.ready) {
  document.fonts.ready.then(fitPoster);
}

fitPoster();
