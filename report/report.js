(() => {
  const progress = document.getElementById("progress");
  const links = Array.from(document.querySelectorAll(".toc a"));
  const sections = links
    .map((link) => document.querySelector(link.getAttribute("href")))
    .filter(Boolean);

  let frame = 0;

  const setActive = (id) => {
    links.forEach((link) => {
      link.classList.toggle("active", link.getAttribute("href") === `#${id}`);
    });
  };

  const update = () => {
    frame = 0;
    const doc = document.documentElement;
    const maxScroll = doc.scrollHeight - window.innerHeight;
    const ratio = maxScroll > 0 ? Math.min(1, Math.max(0, window.scrollY / maxScroll)) : 0;
    if (progress) progress.style.width = `${ratio * 100}%`;

    const marker = window.scrollY + 150;
    let current = sections[0];
    sections.forEach((section) => {
      if (section.offsetTop <= marker) current = section;
    });
    if (current) setActive(current.id);
  };

  const requestUpdate = () => {
    if (!frame) frame = window.requestAnimationFrame(update);
  };

  window.addEventListener("scroll", requestUpdate, { passive: true });
  window.addEventListener("resize", requestUpdate);
  window.addEventListener("load", () => {
    update();
    if (window.location.hash) {
      const target = document.querySelector(window.location.hash);
      if (target) window.setTimeout(() => target.scrollIntoView({ behavior: "smooth" }), 80);
    }
  });

  links.forEach((link) => {
    link.addEventListener("click", () => setActive(link.getAttribute("href").slice(1)));
  });

  update();
})();
