// Project Eye v5 goruntuleyici test duzenegi: model.html'i jsdom'da, sahte THREE + sahte claude.use ile calistirir.
// Kullanim: node run.js <model.html> [senaryo]
const fs = require('fs');
const { JSDOM, VirtualConsole } = require('jsdom');
const file = process.argv[2];
let html = fs.readFileSync(file, 'utf8');
// dis betikleri cikar (THREE/JSZip/OrbitControls/GLTFLoader) -> sahteleri enjekte edilecek
html = html.replace(/<script src="[^"]+"><\/script>/g, '');
// dev GLB base64'unu kucult (parse sahte)
html = html.replace(/const GLB_B64 = "[^"]*";/, 'const GLB_B64 = "AAAA";');
const POZ = JSON.parse(html.match(/const POZ = (\{.*?\});\r?\nconst GLB_B64/s)[1]);

const errors = [];
const vc = new VirtualConsole();
vc.on('jsdomError', e => errors.push('jsdomError: ' + (e.detail ? e.detail.stack || e.detail : e.message)));
vc.on('error', (...a) => errors.push('console.error: ' + a.join(' ')));
vc.on('log', (...a) => console.log('[page]', ...a));

const DB_DATA = JSON.parse(process.env.DB_DATA || '{}');   // {"baski": {"eye_lever": {...}}}
const writes = [];

const dom = new JSDOM(html, {
  runScripts: 'dangerously', pretendToBeVisual: true, virtualConsole: vc, url: 'http://localhost/model.html',
  beforeParse(w) {
    // ---- sahte THREE ----
    class V3 { constructor(x=0,y=0,z=0){this.x=x;this.y=y;this.z=z;} set(x,y,z){this.x=x;this.y=y;this.z=z;return this;} length(){return Math.hypot(this.x,this.y,this.z);} }
    class Obj { constructor(){ this.children=[]; this.parent=null; this.visible=true; this.name=''; this.matrix={clone(){return this;}, multiplyMatrices(){}}; this.position=new V3(); }
      add(...c){ c.forEach(x=>{x.parent=this;this.children.push(x);}); } remove(c){ this.children=this.children.filter(x=>x!==c); c.parent=null; }
      traverse(f){ f(this); this.children.forEach(c=>c.traverse(f)); } updateMatrix(){} }
    class Mat { constructor(o={}){ Object.assign(this,o); this.color={copy(){}}; } clone(){ const m=new Mat(this); m.name=this.name; return m; } }
    class Mesh extends Obj { constructor(g,m){ super(); this.isMesh=true; this.geometry=g||{}; this.material=m||new Mat(); } }
    // sahte sahne: POZ.tum'deki her parca bir grup altinda mesh
    function sahte(){ const root=new Obj(); const g=new Obj(); g.name='G_frame'; root.add(g);
      Object.keys(POZ.tum).forEach(n=>{ const node=new Obj(); node.name=n; const m=new Mesh({}, new Mat()); m.material.name=n; m.name=n; node.add(m); g.add(node); }); return root; }
    w.THREE = {
      WebGLRenderer: class { constructor(){ this.domElement=w.document.createElement('canvas'); } setPixelRatio(){} setClearColor(){} setSize(){} render(){} set outputEncoding(v){} },
      Scene: class extends Obj {}, PerspectiveCamera: class extends Obj { constructor(){ super(); this.aspect=1; } updateProjectionMatrix(){} },
      OrbitControls: class { constructor(){ this.target=new V3(); } update(){} },
      HemisphereLight: class extends Obj {}, DirectionalLight: class extends Obj {},
      GLTFLoader: class { parse(bin, p, ok){ setTimeout(()=>ok({scene: sahte()}), 10); } },
      Matrix4: class { set(){ return this; } }, Color: class { constructor(){} convertSRGBToLinear(){ return this; } },
      MeshBasicMaterial: Mat, LineBasicMaterial: Mat, Mesh, LineSegments: Mesh, EdgesGeometry: class {},
      Raycaster: class { setFromCamera(){} intersectObjects(){ return []; } }, Vector2: class { set(){} },
      sRGBEncoding: 3001, DoubleSide: 2,
    };
    w.JSZip = class { file(){} async generateAsync(){ return new w.Blob(['x']); } };
    w.ResizeObserver = class { observe(){} };
    if (process.env.LS) { const o = JSON.parse(process.env.LS); for (const k in o) w.localStorage.setItem(k, o[k]); }
    w.matchMedia = () => ({matches: false});
    w.HTMLCanvasElement.prototype.getContext = () => null;
    // ---- sahte claude.use (db + downloads) ----
    if (process.env.ARTIFACT === '1') {
      const coll = name => ({
        onSnapshot(next){ const docs = Object.entries(DB_DATA[name] || {}).map(([id, d]) => ({id, exists:true, data:()=>Object.freeze(d)}));
          setTimeout(()=>next({docs, size:docs.length, empty:!docs.length, docChanges:()=>[], metadata:{}}), 50); return ()=>{}; },
      });
      const db = { collection: coll, doc: path => ({ set: async d => { writes.push([path, d]); }, onSnapshot(){ return ()=>{}; } }) };
      const c = { use: async name => name === 'db' ? db : name === 'downloads' ? { save: async () => ({status:'saved'}) } : null };
      if (process.env.LATE) setTimeout(() => { w.claude = c; }, +process.env.LATE); else w.claude = c;
    }
  }
});
const w = dom.window;
setTimeout(() => {
  const d = w.document;
  const out = {};
  out.errors = errors;
  out.siraliSatir = d.querySelectorAll('#agacSirali .renk').length;
  out.kutular = [...d.querySelectorAll('.kutular')].map(sp => sp.dataset.k + ':' + [...sp.querySelectorAll('.tkb')].map(b => b.dataset.s).join(''));
  out.ilerle = d.querySelector('#ilerleYuzde').textContent + ' ' + d.querySelector('#ilerleYazi').textContent;
  out.durumYazi = d.querySelector('#indirDurum').textContent;
  out.bitti = [...d.querySelectorAll('.renk.bitti .ad')].map(a => a.textContent);
  // tiklama senaryosu: eye_lever ilk kutuya 1 kez tikla
  const b = d.querySelector('.kutular[data-k="eye"] .tkb');
  if (b) { b.click(); out.eyeTik1 = b.dataset.s; b.click(); out.eyeTik2 = b.dataset.s; b.click(); out.eyeTik3 = b.dataset.s; }
  setTimeout(() => { out.writes = writes; console.log(JSON.stringify(out, null, 1)); process.exit(0); }, 100);
}, +(process.env.WAIT || 800));
