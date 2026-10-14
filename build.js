// Stitches src/template.html + src/core.js + src/ui.js into one self-contained index.html.
// Test-only hooks (module.exports, window.__lab, linearisation debug fields) are stripped from the shipped file.
const fs = require('fs'), path = require('path');
const src = (f) => fs.readFileSync(path.join(__dirname, 'src', f), 'utf8');
let core = src('core.js'), ui = src('ui.js');
core = core.replace(/\nconst Core = \{.*?module\.exports = Core;\n/s, '\n').replace('  K.Ad = Ad; K.Bd = Bd; K.A = A; K.B = B;\n', '');
ui = ui.replace('  window.__lab = { trainer, live, P, state };   // handy for debugging in the console\n', '');
const html = src('template.html').replace('/*CORE*/', () => core).replace('/*UI*/', () => ui);
fs.writeFileSync(path.join(__dirname, 'index.html'), html);
console.log('wrote index.html (' + html.length + ' bytes)');
