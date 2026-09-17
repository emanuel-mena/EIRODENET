import * as THREE from 'three'
import './style.css'

const roverIds = [10, 11]

document.querySelector('#app').innerHTML = `
  <header class="topbar">
    <div><span class="brand-mark"></span><strong>EIRODENET</strong><small>CONTROL DE CAMPO</small></div>
    <div class="legend"><span class="blue"></span> Prueba <span class="yellow"></span> Competencia 10 <span class="purple"></span> Competencia 11</div>
  </header>
  <main>
    <section class="hero">
      <p class="kicker">DOS ROVERS · UNA CONSOLA</p>
      <h1>Telemetría y control<br><em>en tiempo real.</em></h1>
      <p>El modo sólo cambia físicamente con el botón BOOT de cada rover. En competencia, la consola queda en sólo lectura.</p>
    </section>
    <section class="rovers">${roverIds.map(id => roverCard(id)).join('')}</section>
  </main>
  <div id="toast" role="status" aria-live="polite"></div>`

function roverCard(id) {
  return `<article class="rover" id="rover-${id}" data-online="false">
    <div class="rover-head"><div><p class="kicker">UNIDAD ${id}</p><h2>Rover ${id}</h2></div><div class="connection"><i></i><span>Conectando</span></div></div>
    <div class="transport"><span>TRANSPORTE</span><strong>Esperando rover anfitrión</strong></div>
    <div class="offline">Sin conexión con Rover ${id}. Sus datos y comandos están bloqueados.</div>
    <fieldset disabled>
      <div class="mode-line"><span class="mode-pill">—</span><span class="ip">Sin IP</span><span class="rssi">— dBm</span></div>
      <section class="orientation"><div class="arrow" aria-label="Dirección tridimensional del rover"></div><div><p class="section-label">ORIENTACIÓN IMU</p><strong class="temperature">— °C</strong><p class="imu-state">Esperando lectura</p></div></section>
      <section><p class="section-label">SENSORES</p><div class="sensor-grid">
        <div><small>Distancia</small><strong data-sensor="distance">—</strong><span>mm</span></div>
        <div><small>IR frente</small><strong data-sensor="ir-front">— / —</strong></div>
        <div><small>IR atrás</small><strong data-sensor="ir-rear">— / —</strong></div>
        <div><small>Color R/G/B</small><strong data-sensor="color">— / — / —</strong></div>
      </div></section>
      <section class="controls"><div><p class="section-label">CONTROL DIRECTO</p><div class="dpad">
        <button data-drive="forward" aria-label="Avanzar">↑</button><button data-drive="left" aria-label="Girar izquierda">←</button><button data-drive="stop" class="stop" aria-label="Detener">■</button><button data-drive="right" aria-label="Girar derecha">→</button><button data-drive="back" aria-label="Retroceder">↓</button>
      </div><small class="hint">Mantén presionado · parada automática en 500 ms</small></div>
      <form class="target-form"><p class="section-label">OBJETIVO DE NAVEGACIÓN</p><div><label>Col <input name="col" type="number" min="0" step="0.1" required></label><label>Fila <input name="row" type="number" min="0" step="0.1" required></label></div><button type="submit">Navegar al punto</button><button type="button" class="cancel-navigation">Cancelar y detener</button><small class="nav-state">Esperando pose de visión</small><div class="nav-telemetry"><span data-nav="pose">Pose —</span><span data-nav="speed">Velocidad —</span><span data-nav="vision">Visión —</span><span data-nav="grid">Cuadrícula —</span></div></form></section>
    </fieldset></article>`
}

const driveCommands = { forward: [700, 700], back: [-700, -700], left: [-700, 700], right: [700, -700], stop: [0, 0] }
const rovers = new Map()
for (const id of roverIds) {
  const element = document.querySelector(`#rover-${id}`)
  const state = { id, element, base: null, online: false, busy: false, driveTimer: null }
  state.orientation = makeOrientation(element.querySelector('.arrow'), id)
  rovers.set(id, state)
  element.querySelector('.target-form').addEventListener('submit', event => sendTarget(event, state))
  element.querySelector('.cancel-navigation').addEventListener('click', () => cancelNavigation(state))
  for (const button of element.querySelectorAll('[data-drive]')) bindDrive(button, state)
}

function makeOrientation(container, id) {
  const scene = new THREE.Scene()
  const camera = new THREE.PerspectiveCamera(34, 1, 0.1, 100)
  camera.position.set(0, 0.4, 5)
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true })
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
  container.appendChild(renderer.domElement)
  const arrow = new THREE.Group()
  const color = id === 10 ? 0xf4bf47 : 0x9c78ff
  const material = new THREE.MeshStandardMaterial({ color, metalness: .15, roughness: .35 })
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(.13, .13, 1.55, 20), material)
  shaft.rotation.z = -Math.PI / 2; shaft.position.x = -.25
  const head = new THREE.Mesh(new THREE.ConeGeometry(.42, .9, 24), material)
  head.rotation.z = -Math.PI / 2; head.position.x = .95
  arrow.add(shaft, head); scene.add(arrow); scene.add(new THREE.HemisphereLight(0xddeaff, 0x101827, 2.5))
  const resize = () => { const size = Math.max(160, container.clientWidth); renderer.setSize(size, 180, false); camera.aspect = size / 180; camera.updateProjectionMatrix() }
  resize(); new ResizeObserver(resize).observe(container)
  const render = () => { renderer.render(scene, camera); requestAnimationFrame(render) }
  render(); return arrow
}

function setOnline(rover, online, label = online ? 'En línea' : 'Sin conexión') {
  if (!online && rover.online) clearTelemetry(rover)
  rover.online = online; rover.element.dataset.online = String(online)
  rover.element.querySelector('fieldset').disabled = !online
  rover.element.querySelector('.connection span').textContent = label
}

function clearTelemetry(rover) {
  const e = rover.element
  delete e.dataset.mode
  e.querySelector('.mode-pill').textContent = '—'; e.querySelector('.ip').textContent = 'Sin IP'; e.querySelector('.rssi').textContent = '— dBm'
  e.querySelector('.temperature').textContent = '— °C'; e.querySelector('.imu-state').textContent = 'Esperando lectura'
  setText(e, 'distance', '—'); setText(e, 'ir-front', '— / —'); setText(e, 'ir-rear', '— / —'); setText(e, 'color', '— / — / —')
  for (const key of ['pose', 'speed', 'vision', 'grid']) e.querySelector(`[data-nav="${key}"]`).textContent = `${key} —`
  rover.orientation.quaternion.identity()
}

async function poll(rover) {
  if (rover.busy || !rover.base) return
  rover.busy = true
  const controller = new AbortController(); const timeout = setTimeout(() => controller.abort(), 1200)
  try {
    const response = await fetch(`${rover.base}/state`, { signal: controller.signal, cache: 'no-store' })
    if (!response.ok) throw new Error(`HTTP ${response.status}`)
    const data = await response.json()
    if (!data.ok || data.rover_id !== rover.id) throw new Error(`La dirección responde como Rover ${data.rover_id || 'sin ID'}`)
    setOnline(rover, true); updateRover(rover, data)
  } catch (error) {
    if (rover.online) notify(`Rover ${rover.id} desconectado: ${error.message}`)
    setOnline(rover, false)
  } finally { clearTimeout(timeout); rover.busy = false }
}

function updateRover(rover, data) {
  const e = rover.element; const competition = data.mode === 'competition'
  e.dataset.mode = data.mode
  e.querySelector('.mode-pill').textContent = competition ? 'COMPETENCIA' : 'PRUEBA'
  e.querySelector('.ip').textContent = data.network.ipv4; e.querySelector('.rssi').textContent = `${data.network.rssi} dBm`
  e.querySelector('.temperature').textContent = data.imu.valid ? `${data.imu.temperature_c.toFixed(1)} °C` : '— °C'
  e.querySelector('.imu-state').textContent = data.imu.valid ? (data.imu.calibrated ? 'Calibrada' : 'Sin calibrar') : `No disponible · error ${data.imu.error}`
  if (data.imu.valid && data.imu.quaternion?.length === 4) { const [w, x, y, z] = data.imu.quaternion; rover.orientation.quaternion.set(x, y, z, w).normalize() }
  setText(e, 'distance', data.sensors.ultrasonic.valid ? data.sensors.ultrasonic.distance_mm : '—')
  setText(e, 'ir-front', data.sensors.infrared.valid ? `${data.sensors.infrared.front_left} / ${data.sensors.infrared.front_right}` : '— / —')
  setText(e, 'ir-rear', data.sensors.infrared.valid ? `${data.sensors.infrared.rear_left} / ${data.sensors.infrared.rear_right}` : '— / —')
  setText(e, 'color', data.sensors.color.valid ? `${data.sensors.color.red} / ${data.sensors.color.green} / ${data.sensors.color.blue}` : '— / — / —')
  e.querySelectorAll('.controls button, .controls input').forEach(control => { control.disabled = competition })
  const nav = data.navigation; const pose = nav.pose || {}; const vision = nav.vision || {}; const grid = nav.grid_encoder || {}
  e.querySelector('.nav-state').textContent = nav.has_target
    ? `${nav.phase_name} #${nav.request_id} → (${nav.col.toFixed(1)}, ${nav.row.toFixed(1)})`
    : `${nav.phase_name || 'idle'} · error ${nav.error || 0}`
  e.querySelector('[data-nav="pose"]').textContent = pose.valid ? `Pose ${pose.col.toFixed(2)}, ${pose.row.toFixed(2)} · ${pose.theta_deg.toFixed(1)}°` : 'Pose no inicializada'
  e.querySelector('[data-nav="speed"]').textContent = pose.valid ? `v ${pose.speed_cells_s.toFixed(2)} cel/s · ±${pose.uncertainty_cells.toFixed(2)} cel` : 'Velocidad —'
  e.querySelector('[data-nav="vision"]').textContent = vision.fresh ? `Visión fresca · ${vision.age_ms} ms` : (vision.connected ? 'Visión sin pose fresca' : 'Visión desconectada · local')
  e.querySelector('[data-nav="grid"]').textContent = grid.calibrated ? `Grid 0b${Number(grid.pattern).toString(2).padStart(4, '0')} · listo` : `Grid calibrando · máscara 0x${Number(grid.calibrated_mask || 0).toString(16)}`
}
function setText(element, sensor, value) { element.querySelector(`[data-sensor="${sensor}"]`).textContent = value }

function bindDrive(button, rover) {
  const command = driveCommands[button.dataset.drive]
  const send = () => post(rover, '/drive', { left: command[0], right: command[1] }, false)
  const stop = () => { clearInterval(rover.driveTimer); rover.driveTimer = null; post(rover, '/drive', { left: 0, right: 0 }, false) }
  if (button.dataset.drive === 'stop') { button.addEventListener('click', send); return }
  button.addEventListener('pointerdown', event => { event.preventDefault(); button.setPointerCapture(event.pointerId); send(); rover.driveTimer = setInterval(send, 250) })
  button.addEventListener('pointerup', stop); button.addEventListener('pointercancel', stop); button.addEventListener('lostpointercapture', stop)
}

async function sendTarget(event, rover) {
  event.preventDefault(); const form = new FormData(event.currentTarget)
  const result = await post(rover, '/navigation/target', { col: Number(form.get('col')), row: Number(form.get('row')) })
  if (result) notify(result.transport === 'esp-now'
    ? `Rover ${rover.id}: objetivo enviado por ESP-NOW`
    : `Rover ${rover.id}: navegación #${result.request_id} iniciada`)
}
async function cancelNavigation(rover) {
  const peer = rover.base?.endsWith('/peer')
  const result = await post(rover, peer ? '/drive' : '/navigation/cancel', peer ? { left: 0, right: 0 } : {})
  if (result) notify(`Rover ${rover.id}: navegación cancelada y motores detenidos`)
}
async function post(rover, path, body, announceErrors = true) {
  if (!rover.online) { if (announceErrors) notify(`Rover ${rover.id} está desconectado; comando bloqueado`); return null }
  try {
    const response = await fetch(`${rover.base}${path}`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
    const data = await response.json(); if (!response.ok || !data.ok) throw new Error(data.code || `HTTP ${response.status}`); return data
  } catch (error) { if (announceErrors) notify(`Rover ${rover.id}: ${error.message}`); return null }
}
let toastTimer
function notify(message) { const toast = document.querySelector('#toast'); toast.textContent = message; toast.classList.add('show'); clearTimeout(toastTimer); toastTimer = setTimeout(() => toast.classList.remove('show'), 4000) }
let discovering = false
setInterval(() => {
  if ([...rovers.values()].some(rover => rover.base)) rovers.forEach(poll)
  else discoverServingRover()
}, 700)
discoverServingRover()

async function discoverServingRover() {
  if (discovering) return
  discovering = true
  try {
    const response = await fetch('/api/v1/state', { cache: 'no-store' })
    const data = await response.json(); const rover = rovers.get(data.rover_id)
    if (!data.ok || !rover) return
    rover.base = '/api/v1'
    rover.element.querySelector('.transport strong').textContent = `HTTP local · ${window.location.host}`
    setOnline(rover, true); updateRover(rover, data)
    const peerId = rover.id === 10 ? 11 : 10
    const peer = rovers.get(peerId)
    peer.base = '/api/v1/peer'
    peer.element.querySelector('.transport strong').textContent = 'ESP-NOW · MAC configurada'
    poll(peer)
  } catch { /* El servidor de desarrollo no expone la API del ESP32. */ }
  finally { discovering = false }
}
