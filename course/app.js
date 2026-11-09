(() => {
  'use strict';

  const root = document.documentElement;
  const $ = (selector, context = document) => context.querySelector(selector);
  const $$ = (selector, context = document) => Array.from(context.querySelectorAll(selector));

  const store = {
    get(key, fallback) {
      try {
        const value = localStorage.getItem(key);
        return value === null ? fallback : value;
      } catch (error) {
        return fallback;
      }
    },
    set(key, value) {
      try { localStorage.setItem(key, value); } catch (error) { /* file:// 下可能被禁用 */ }
    },
    remove(key) {
      try { localStorage.removeItem(key); } catch (error) { /* ignore */ }
    }
  };

  /* ---------- 主题 ---------- */
  const THEME_KEY = 'ros2-course-theme';
  const themeToggle = $('#themeToggle');
  const sunIcon = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.93 4.93l1.42 1.42M17.66 17.66l1.41 1.41M2 12h2M20 12h2M4.93 19.07l1.42-1.42M17.66 6.34l1.41-1.41"/></svg>';
  const moonIcon = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"/></svg>';

  function applyTheme(theme) {
    root.dataset.theme = theme;
    store.set(THEME_KEY, theme);
    if (themeToggle) {
      const dark = theme === 'dark';
      themeToggle.innerHTML = dark ? sunIcon : moonIcon;
      themeToggle.setAttribute('aria-label', dark ? '切换到浅色主题' : '切换到深色主题');
      themeToggle.title = dark ? '切换到浅色主题' : '切换到深色主题';
    }
  }

  applyTheme(store.get(THEME_KEY, 'dark'));

  if (themeToggle) {
    themeToggle.addEventListener('click', () => {
      applyTheme(root.dataset.theme === 'dark' ? 'light' : 'dark');
    });
  }

  /* ---------- 移动端目录 ---------- */
  const sidebar = $('#sidebar');
  const menuToggle = $('#menuToggle');

  if (menuToggle && sidebar) {
    menuToggle.addEventListener('click', () => sidebar.classList.toggle('open'));

    document.addEventListener('click', (event) => {
      if (window.innerWidth > 1180 || !sidebar.classList.contains('open')) return;
      if (sidebar.contains(event.target) || menuToggle.contains(event.target)) return;
      sidebar.classList.remove('open');
    });

    $$('.side-nav a').forEach((link) => {
      link.addEventListener('click', () => {
        if (window.innerWidth <= 1180) sidebar.classList.remove('open');
      });
    });
  }

  /* ---------- 滚动进度与回到顶部 ---------- */
  const progressBar = $('#scrollProgress');
  const toTop = $('#toTop');

  function updateScroll() {
    const scrollable = document.documentElement.scrollHeight - window.innerHeight;
    const ratio = scrollable > 0 ? Math.min(1, Math.max(0, window.scrollY / scrollable)) : 0;
    if (progressBar) progressBar.style.width = (ratio * 100).toFixed(2) + '%';
    if (toTop) toTop.classList.toggle('show', window.scrollY > 620);
  }

  window.addEventListener('scroll', updateScroll, { passive: true });
  updateScroll();

  if (toTop) {
    toTop.addEventListener('click', () => window.scrollTo({ top: 0, behavior: 'smooth' }));
  }

  /* ---------- 侧边栏高亮 ---------- */
  const topLinks = $$('.side-nav a:not(.sub)');
  const observedIds = ['path', 'modules', 'architecture', 'commands', 'lab', 'quiz', 'faq'];
  const observed = observedIds.map((id) => document.getElementById(id)).filter(Boolean);
  const linkFor = (id) => topLinks.find((link) => link.getAttribute('href') === '#' + id);

  function clearTopActive() {
    topLinks.forEach((link) => link.classList.remove('active'));
  }

  function setTopActive(id) {
    clearTopActive();
    const link = linkFor(id);
    if (link) link.classList.add('active');
  }

  function updateActiveSection() {
    if (window.scrollY < 320) {
      setTopActive('top');
      return;
    }
    const marker = window.scrollY + Math.min(320, window.innerHeight * 0.35);
    let current = observed[0];
    observed.forEach((section) => {
      if (section.offsetTop <= marker) current = section;
    });
    if (current) setTopActive(current.id);
  }

  window.addEventListener('scroll', updateActiveSection, { passive: true });
  updateActiveSection();

  /* 点击模块子目录时展开对应 details */
  $$('.side-nav a[href^="#m"]').forEach((link) => {
    link.addEventListener('click', (event) => {
      const target = document.querySelector(link.getAttribute('href'));
      if (!target || target.tagName !== 'DETAILS') return;
      event.preventDefault();
      target.open = true;
      clearTopActive();
      link.classList.add('active');
      setTimeout(() => target.scrollIntoView({ behavior: 'smooth', block: 'start' }), 40);
    });
  });

  let moduleToggleReady = false;
  window.setTimeout(() => { moduleToggleReady = true; }, 500);

  $$('.module').forEach((module) => {
    module.addEventListener('toggle', () => {
      if (!moduleToggleReady || !module.open) return;
      const link = document.querySelector('.side-nav a[href="#' + module.id + '"]');
      if (!link) return;
      clearTopActive();
      link.classList.add('active');
    });
  });

  /* ---------- 学习进度 ---------- */
  const PROGRESS_KEY = 'ros2-course-progress';
  const checks = $$('.module-check');
  const progressText = $('#progressText');
  const sideProgressBar = $('#sideProgressBar');

  function readProgress() {
    try {
      const parsed = JSON.parse(store.get(PROGRESS_KEY, '[]'));
      return Array.isArray(parsed) ? parsed : [];
    } catch (error) {
      return [];
    }
  }

  function writeProgress() {
    const done = checks.filter((checkbox) => checkbox.checked).map((checkbox) => checkbox.dataset.module);
    store.set(PROGRESS_KEY, JSON.stringify(done));
    updateProgressUI();
  }

  function updateProgressUI() {
    const total = checks.length || 1;
    const done = checks.filter((checkbox) => checkbox.checked).length;
    const percent = Math.round((done / total) * 100);
    if (progressText) progressText.textContent = percent + '%';
    if (sideProgressBar) sideProgressBar.style.width = percent + '%';
  }

  const savedProgress = readProgress();
  checks.forEach((checkbox) => {
    checkbox.checked = savedProgress.includes(checkbox.dataset.module);
    checkbox.addEventListener('click', (event) => event.stopPropagation());
    checkbox.addEventListener('mousedown', (event) => event.stopPropagation());
    checkbox.addEventListener('change', writeProgress);
  });
  updateProgressUI();

  const resetProgress = $('#resetProgress');
  if (resetProgress) {
    resetProgress.addEventListener('click', () => {
      store.remove(PROGRESS_KEY);
      checks.forEach((checkbox) => { checkbox.checked = false; });
      updateProgressUI();
    });
  }

  /* ---------- 复制命令 ---------- */
  function fallbackCopy(text) {
    const area = document.createElement('textarea');
    area.value = text;
    area.setAttribute('readonly', '');
    area.style.position = 'fixed';
    area.style.top = '-1000px';
    area.style.opacity = '0';
    document.body.appendChild(area);
    area.select();
    let ok = false;
    try { ok = document.execCommand('copy'); } catch (error) { ok = false; }
    document.body.removeChild(area);
    return ok;
  }

  async function copyText(text, button) {
    let ok = false;
    if (navigator.clipboard && window.isSecureContext) {
      try {
        await navigator.clipboard.writeText(text);
        ok = true;
      } catch (error) {
        ok = false;
      }
    }
    if (!ok) ok = fallbackCopy(text);

    if (!button) return;
    const original = button.textContent;
    button.textContent = ok ? '已复制' : '复制失败';
    button.classList.toggle('copied', ok);
    window.setTimeout(() => {
      button.textContent = original;
      button.classList.remove('copied');
    }, 1400);
  }

  document.addEventListener('click', (event) => {
    const copyButton = event.target.closest('.copy-btn, .cmd-copy');
    if (!copyButton) return;
    const block = copyButton.closest('.code-block');
    const row = copyButton.closest('.cmd-row');
    const code = block ? block.querySelector('code') : row ? row.querySelector('code') : null;
    if (code) copyText(code.innerText, copyButton);
  });

  /* ---------- 命令筛选 ---------- */
  const cmdSearch = $('#cmdSearch');
  const cmdFilters = $('#cmdFilters');
  const cmdRows = $$('.cmd-row');
  const cmdEmpty = $('#cmdEmpty');
  let activeFilter = 'all';

  function filterCommands() {
    const query = cmdSearch ? cmdSearch.value.trim().toLowerCase() : '';
    let visible = 0;

    cmdRows.forEach((row) => {
      const matchesCategory = activeFilter === 'all' || row.dataset.cat === activeFilter;
      const matchesQuery = !query || row.innerText.toLowerCase().includes(query);
      const show = matchesCategory && matchesQuery;
      row.classList.toggle('hidden', !show);
      if (show) visible += 1;
    });

    if (cmdEmpty) cmdEmpty.classList.toggle('show', visible === 0);
  }

  if (cmdSearch) cmdSearch.addEventListener('input', filterCommands);

  if (cmdFilters) {
    cmdFilters.addEventListener('click', (event) => {
      const chip = event.target.closest('.filter-chip');
      if (!chip) return;
      $$('.filter-chip', cmdFilters).forEach((item) => item.classList.remove('active'));
      chip.classList.add('active');
      activeFilter = chip.dataset.filter || 'all';
      filterCommands();
    });
  }

  filterCommands();

  /* ---------- 架构 Tab ---------- */
  const tabs = $$('.tab');
  tabs.forEach((tab) => {
    tab.addEventListener('click', () => {
      tabs.forEach((item) => {
        item.classList.remove('active');
        item.setAttribute('aria-selected', 'false');
      });
      tab.classList.add('active');
      tab.setAttribute('aria-selected', 'true');

      $$('.arch-panel').forEach((panel) => {
        panel.classList.toggle('active', panel.id === 'panel-' + tab.dataset.tab);
      });
    });
  });

  /* ---------- 计算图交互 ---------- */
  const graphData = {
    lidar: {
      role: 'Sensor',
      title: '激光雷达 /scan',
      desc: '持续发布距离数据。避障节点把前向扇区里的最近距离与 front_stop 比较，决定停车还是绕行。',
      rows: [['发布', '/scan'], ['类型', 'sensor_msgs/LaserScan'], ['频率', '约 10 Hz']]
    },
    camera: {
      role: 'Sensor',
      title: '相机 /image_raw',
      desc: '把图像发布给感知节点和 MJPEG 图传。视觉识别与人工观察可以同时订阅，互不影响。',
      rows: [['发布', '/image_raw'], ['类型', 'sensor_msgs/Image'], ['用途', '识别 / 图传']]
    },
    odom: {
      role: 'Localization',
      title: '底盘里程计 /odom',
      desc: '根据轮速积分得到短距离位姿。演示场景里用它判断是否到达目标点，长距离再换 AMCL + 地图。',
      rows: [['发布', '/odom'], ['类型', 'nav_msgs/Odometry'], ['频率', '约 20 Hz']]
    },
    perception: {
      role: 'Perception',
      title: '感知节点',
      desc: '订阅图像，做 HSV 颜色或 YOLO 检测，输出目标位置。它不应该直接控制底盘，只负责“看见什么”。',
      rows: [['订阅', '/image_raw'], ['发布', '/detections'], ['算法', 'HSV / YOLO']]
    },
    mission: {
      role: 'Mission',
      title: 'mission_node',
      desc: '整个巡检的调度中心：读雷达和里程计，判断是否被挡住；选择绕行方向，发布速度；到点后触发机械臂动作。',
      rows: [['订阅', '/scan · /odom'], ['发布', '/cmd_vel · /arm_command'], ['关键参数', 'front_stop · avoid_turn_deg']]
    },
    rviz: {
      role: 'Debug',
      title: 'RViz2',
      desc: '只看不改的调试工具。把 /scan、/odom、/tf 叠在一起，能快速判断坐标和传感器是否对齐。',
      rows: [['订阅', '/scan · /odom · /tf'], ['用途', '可视化'], ['注意', '不要用它替代日志']]
    },
    base: {
      role: 'Driver',
      title: 'base_driver',
      desc: '把 /cmd_vel 转成左右轮速度，通过串口或 CAN 发给驱动板；同时把轮速积分成 /odom。',
      rows: [['订阅', '/cmd_vel'], ['发布', '/odom'], ['接口', '串口 / CAN']]
    },
    arm: {
      role: 'Driver',
      title: 'arm_driver',
      desc: '把机械臂指令转换成舵机原始值，通过 I2C 地址 0x15 下发给扩展板。',
      rows: [['订阅', '/arm_command'], ['接口', 'I2C 0x15'], ['输出', 'PWM / 舵机角度']]
    },
    chassis: {
      role: 'Hardware',
      title: '底盘 / 电机',
      desc: '执行速度指令的物理层。真实系统里还要考虑电流、堵转、轮速标定和急停。',
      rows: [['输入', '左右轮速度'], ['反馈', '编码器'], ['安全', '急停 / 限流']]
    },
    armhw: {
      role: 'Hardware',
      title: '机械臂 / 夹爪',
      desc: '六个总线舵机加一个夹爪。上电前确认独立供电、共地和中位姿态，避免堵转发热。',
      rows: [['控制', 'I2C 0x15'], ['关节', 'J1–J6'], ['注意', '限位 / 供电 / 散热']]
    },
    master: {
      role: 'Teleop',
      title: '主臂 /master_joint_states',
      desc: '操作者拖动主臂，节点读取各关节角并发布。主臂本身不直接驱动从臂，中间必须经过映射和限幅。',
      rows: [['发布', '/master_joint_states'], ['输入', '主臂舵机角度'], ['模式', '拖动示教']]
    },
    slave: {
      role: 'Teleop',
      title: 'slave_joint_node',
      desc: '订阅主臂关节角，按比例缩放、修正方向、限制行程，再通过 I2C 下发给从臂。',
      rows: [['订阅', '/master_joint_states'], ['发布', '/slave_joint_states'], ['参数', 'scale · signs · grip_index']]
    }
  };

  const infoRole = $('#infoRole');
  const infoTitle = $('#infoTitle');
  const infoDesc = $('#infoDesc');
  const infoList = $('#infoList');

  function showNode(id) {
    const data = graphData[id];
    if (!data || !infoRole) return;
    infoRole.textContent = data.role;
    infoTitle.textContent = data.title;
    infoDesc.textContent = data.desc;
    infoList.innerHTML = data.rows.map(([key, value]) =>
      '<div><span>' + key + '</span><span>' + value + '</span></div>'
    ).join('');
    $$('.graph-node').forEach((node) => {
      node.classList.toggle('active', node.dataset.node === id);
    });
  }

  $$('.graph-node').forEach((node) => {
    const linkIds = (node.dataset.links || '').split(/\s+/).filter(Boolean);
    const links = linkIds.map((id) => document.getElementById(id)).filter(Boolean);

    const highlight = () => links.forEach((link) => link.classList.add('active'));
    const unhighlight = () => links.forEach((link) => link.classList.remove('active'));

    node.addEventListener('mouseenter', highlight);
    node.addEventListener('mouseleave', unhighlight);
    node.addEventListener('focus', highlight);
    node.addEventListener('blur', unhighlight);
    node.addEventListener('click', () => showNode(node.dataset.node));
    node.addEventListener('keydown', (event) => {
      if (event.key === 'Enter' || event.key === ' ') {
        event.preventDefault();
        showNode(node.dataset.node);
      }
    });
  });

  /* ---------- 自测题 ---------- */
  const quizData = [
    {
      q: '持续发布的雷达数据，最适合用哪种通信机制？',
      options: ['Topic', 'Service', 'Action', 'Parameter'],
      answer: 0,
      explain: '雷达是持续数据流，发布/订阅、异步、多对多，选 Topic。'
    },
    {
      q: '“移动到目标点，过程中反馈进度，还能取消”最适合？',
      options: ['Topic', 'Service', 'Action', 'Parameter'],
      answer: 2,
      explain: 'Action 有 goal / feedback / result，支持取消，适合长任务。'
    },
    {
      q: '启动时用 --ros-args -p front_stop:=0.5 修改的是？',
      options: ['话题', '服务', '参数', 'TF'],
      answer: 2,
      explain: '-p 是 parameter 覆盖，节点必须先 declare_parameter 才能被覆盖。'
    },
    {
      q: '雷达 /scan 有话题但收不到数据，最先应该检查？',
      options: ['QoS 是否兼容', '机械臂舵机', '地图分辨率', '电机 PID'],
      answer: 0,
      explain: '传感器常用 Best Effort，订阅端 Reliable 会收不到；先看 topic info --verbose。'
    },
    {
      q: 'TF2 主要解决什么问题？',
      options: ['坐标变换', '图像压缩', '电机调速', '文件编译'],
      answer: 0,
      explain: 'TF2 维护 map / odom / base_link / laser 等坐标系之间的变换关系。'
    }
  ];

  const quizQuestions = $('#quizQuestions');
  const quizSubmit = $('#quizSubmit');
  const quizReset = $('#quizReset');
  const quizResult = $('#quizResult');

  function renderQuiz() {
    if (!quizQuestions) return;
    quizQuestions.innerHTML = quizData.map((item, index) => {
      const options = item.options.map((option, optionIndex) =>
        '<label class="option"><input type="radio" name="q' + index + '" value="' + optionIndex + '"><span>' + option + '</span></label>'
      ).join('');
      return '<div class="q" data-q="' + index + '"><h4><span>Q' + (index + 1) + '</span>' + item.q + '</h4><div class="options">' + options + '</div></div>';
    }).join('');
  }

  renderQuiz();

  if (quizSubmit) {
    quizSubmit.addEventListener('click', () => {
      const answers = quizData.map((item, index) => {
        const checked = document.querySelector('input[name="q' + index + '"]:checked');
        return checked ? Number(checked.value) : null;
      });

      if (answers.includes(null)) {
        if (quizResult) {
          quizResult.className = 'quiz-result show mid';
          quizResult.innerHTML = '<div class="score">还有题目没选</div><p>把 5 道题都选完再提交。</p>';
        }
        return;
      }

      let score = 0;
      $$('.q').forEach((question, index) => {
        const labels = $$('.option', question);
        labels.forEach((label, optionIndex) => {
          label.classList.remove('correct', 'wrong');
          if (optionIndex === quizData[index].answer) label.classList.add('correct');
          else if (answers[index] === optionIndex) label.classList.add('wrong');
        });
        if (answers[index] === quizData[index].answer) score += 1;
      });

      if (quizResult) {
        const level = score >= 4 ? 'good' : score === 3 ? 'mid' : 'bad';
        const title = score === 5 ? '全对，可以开始跑系统了。' : score >= 4 ? '不错，概念基本打通。' : score === 3 ? '刚过线，建议回看错题对应模块。' : '别急，先把第 1、5、7 模块再看一遍。';
        quizResult.className = 'quiz-result show ' + level;
        quizResult.innerHTML = '<div class="score">' + score + ' / ' + quizData.length + '</div><p>' + title + '</p><p>' +
          quizData.map((item, index) => 'Q' + (index + 1) + '：' + item.explain).join('<br>') + '</p>';
      }
    });
  }

  if (quizReset) {
    quizReset.addEventListener('click', () => {
      renderQuiz();
      if (quizResult) {
        quizResult.className = 'quiz-result';
        quizResult.innerHTML = '';
      }
    });
  }

  /* ---------- 首页锚点 ---------- */
  $$('a[href="#top"]').forEach((link) => {
    link.addEventListener('click', (event) => {
      event.preventDefault();
      window.scrollTo({ top: 0, behavior: 'smooth' });
    });
  });
})();

