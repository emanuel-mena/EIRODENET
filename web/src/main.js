import * as THREE from 'three'
import './style.css'
import { competitionEligibility, formatPoseSpeed, mergeDiagnostics,
  navigationMessage, targetRejectionMessage } from './console_state.js'

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
      <p>Inicia la competencia desde esta consola cuando ambos rovers y el servidor de visión estén listos. BOOT también permite cambiar el modo en cada rover.</p>
    </section>
    <section class="competition-panel" aria-labelledby="competition-title">
      <div><p class="kicker">PANEL DE INICIO</p><h2 id="competition-title">Modo competencia</h2><p id="competition-status" role="status">Esperando estado de ambos rovers</p></div>
      <button id="competition-enter" type="button" disabled>Entrar en competencia</button>
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
      <section class="orientation"><div class="arrow" aria-label="Dirección tridimensional del rover"></div><div><p class="section-label">ORIENTACIÓN IMU</p><strong class="imu-angle">— °</strong><p class="imu-axis">Giro Z</p><strong class="temperature">— °C</strong><p class="imu-state">Esperando lectura</p></div></section>
      <section><p class="section-label">SENSORES</p><div class="sensor-grid">
        <div><small>Distancia</small><strong data-sensor="distance">—</strong><span>mm</span></div>
        <div><small>IR frente</small><strong data-sensor="ir-front">— / —</strong></div>
        <div><small>IR atrás</small><strong data-sensor="ir-rear">— / —</strong></div>
        <div><small>Color R/G/B</small><strong data-sensor="color">— / — / —</strong></div>
      </div></section>
      <section class="tinyml"><p class="section-label">POLÍTICA TINYML</p><strong class="tinyml-state">Esperando modelo</strong><div class="tinyml-details"><span data-tinyml="identity">EIRM —</span><span data-tinyml="tensor">Tensor —</span><span data-tinyml="arena">Arena —</span><span data-tinyml="latency">Latencia —</span></div></section>
      <section class="controls"><div><p class="section-label">CONTROL DIRECTO</p><div class="dpad">
        <button data-drive="forward" aria-label="Avanzar">↑</button><button data-drive="left" aria-label="Girar izquierda">←</button><button data-drive="stop" class="stop" aria-label="Detener">■</button><button data-drive="right" aria-label="Girar derecha">→</button><button data-drive="back" aria-label="Retroceder">↓</button>
      </div><small class="hint">Mantén presionado · parada automática en 500 ms</small></div>
      <form class="target-form"><p class="section-label">OBJETIVO DE NAVEGACIÓN</p><div><label>Col <input name="col" type="number" min="0" step="0.1" required></label><label>Fila <input name="row" type="number" min="0" step="0.1" required></label></div><button type="submit">Navegar al punto</button><button type="button" class="cancel-navigation">Cancelar y detener</button><small class="nav-state" role="status" aria-live="polite">Esperando pose de visión</small><div class="nav-telemetry"><span data-nav="pose">Pose —</span><span data-nav="speed">Velocidad —</span><span data-nav="vision">Visión —</span><span data-nav="grid">Cuadrícula —</span><span data-nav="route">Ruta —</span><span data-nav="waypoint">Waypoint —</span></div></form></section>
    </fieldset>
    <section class="diagnostics" aria-label="Diagnóstico del Rover ${id}">
      <div class="diagnostics-head"><div><p class="section-label">REGISTRO LOCAL · ROVER ${id}</p><small class="diagnostics-reset">Sin datos de arranque</small></div><small class="diagnostics-count">0 / 100 líneas</small></div>
      <div class="diagnostics-latest"><small>ÚLTIMO MENSAJE SERIAL</small><pre>Esperando mensajes del firmware</pre></div>
      <details><summary>Ver historial</summary><ol class="diagnostics-list"></ol></details>
    </section></article>`
}

const driveCommands = { forward: [700, 700], back: [-700, -700], left: [-700, 700], right: [700, -700], stop: [0, 0] }
const rovers = new Map()
for (const id of roverIds) {
  const element = document.querySelector(`#rover-${id}`)
  const state = { id, element, base: null, online: false, busy: false, diagBusy: false,
    mode: null, visionRecent: false, visionExpiresAt: 0, modelAvailable: false,
    modelVersion: 0, modelCrc32: 0, driveTimer: null, logEntries: readStoredLogs(id),
    bootId: null, resetReason: null, renderError: null, lastData: null,
    trackedRequestId: null, notifiedNavPhase: null }
  state.orientation = makeOrientation(element.querySelector('.arrow'), id)
  rovers.set(id, state)
  element.querySelector('.target-form').addEventListener('submit', event => sendTarget(event, state))
  element.querySelector('.cancel-navigation').addEventListener('click', () => cancelNavigation(state))
  for (const button of element.querySelectorAll('[data-drive]')) bindDrive(button, state)
  renderDiagnostics(state)
}

const competitionButton = document.querySelector('#competition-enter')
let competitionPending = false
competitionButton.addEventListener('click', enterCompetition)

function renderCompetitionStatus() {
  const gate = currentCompetitionGate()
  competitionButton.disabled = !gate.enabled || competitionPending
  document.querySelector('#competition-status').textContent = competitionPending
    ? 'Enviando orden y esperando confirmación del compañero…' : gate.reason
}

function currentCompetitionGate() {
  const now = performance.now()
  return competitionEligibility([...rovers.values()].map(rover => ({
    id: rover.id, online: rover.online, mode: rover.mode,
    visionRecent: rover.visionRecent && now < rover.visionExpiresAt,
    modelAvailable: rover.modelAvailable, modelVersion: rover.modelVersion,
    modelCrc32: rover.modelCrc32,
  })))
}

async function enterCompetition() {
  if (competitionPending || !currentCompetitionGate().enabled) return
  const host = [...rovers.values()].find(rover => rover.base === '/api/v1')
  if (!host) return
  competitionPending = true; renderCompetitionStatus()
  try {
    const response = await fetch('/api/v1/competition/enter', { method: 'POST' })
    const data = await response.json()
    if (!response.ok || !data.ok) {
      const labels = { peer_offline: 'sin enlace ESP-NOW', test_mode_required: 'fuera de modo prueba',
        server_data_stale: 'sin datos recientes del servidor', peer_confirmation_failed: 'sin confirmación del compañero',
        mixed_mode: 'estado mixto; usa BOOT para volver ambos a prueba' }
      throw new Error(`Rover ${data.rover_id || '—'}: ${labels[data.code] || data.code || `HTTP ${response.status}`}`)
    }
    notify('Ambos rovers entraron en competencia')
  } catch (error) { notify(`No se pudo iniciar competencia: ${error.message}`) }
  finally {
    competitionPending = false
    await Promise.all([...rovers.values()].map(poll))
    renderCompetitionStatus()
  }
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
  if (!online) { rover.mode = null; rover.visionRecent = false; rover.visionExpiresAt = 0; rover.modelAvailable = false; rover.modelVersion = 0; rover.modelCrc32 = 0 }
  rover.online = online; rover.element.dataset.online = String(online)
  rover.element.querySelector('fieldset').disabled = !online
  rover.element.querySelector('.connection span').textContent = label
  renderCompetitionStatus()
}

function clearTelemetry(rover) {
  const e = rover.element
  delete e.dataset.mode
  e.querySelector('.mode-pill').textContent = '—'; e.querySelector('.ip').textContent = 'Sin IP'; e.querySelector('.rssi').textContent = '— dBm'
  e.querySelector('.imu-angle').textContent = '— °'; e.querySelector('.temperature').textContent = '— °C'; e.querySelector('.imu-state').textContent = 'Esperando lectura'
  setText(e, 'distance', '—'); setText(e, 'ir-front', '— / —'); setText(e, 'ir-rear', '— / —'); setText(e, 'color', '— / — / —')
  e.querySelector('.tinyml-state').textContent = 'Esperando modelo'
  for (const key of ['identity', 'tensor', 'arena', 'latency']) e.querySelector(`[data-tinyml="${key}"]`).textContent = `${key} —`
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
    renderRover(rover, data)
  } catch (error) {
    if (rover.online) notify(`Rover ${rover.id} desconectado: ${error.message}`)
    setOnline(rover, false)
  } finally { clearTimeout(timeout); rover.busy = false }
}

function renderRover(rover, data) {
  setOnline(rover, true)
  try {
    updateRover(rover, data)
    rover.renderError = null
  } catch (error) {
    if (rover.renderError !== error.message) {
      console.error(`Error al mostrar telemetría de Rover ${rover.id}`, error)
      notify(`Rover ${rover.id}: error al mostrar telemetría: ${error.message}`)
    }
    rover.renderError = error.message
  }
}

function updateRover(rover, data) {
  rover.lastData = data
  const e = rover.element; const competition = data.mode === 'competition'
  rover.mode = data.mode
  rover.visionRecent = data.vision_stream?.recent === true
  rover.visionExpiresAt = performance.now() + Math.max(0, 750 - Number(data.vision_stream?.age_ms || 750))
  e.dataset.mode = data.mode
  e.querySelector('.mode-pill').textContent = competition ? 'COMPETENCIA' : 'PRUEBA'
  e.querySelector('.ip').textContent = data.network.ipv4; e.querySelector('.rssi').textContent = `${data.network.rssi} dBm`
  e.querySelector('.temperature').textContent = data.imu.valid ? `${data.imu.temperature_c.toFixed(1)} °C` : '— °C'
  e.querySelector('.imu-state').textContent = data.imu.valid ? (data.imu.calibrated ? 'Calibrada' : 'Sin calibrar') : `No disponible · error ${data.imu.error}`
  e.querySelector('.imu-angle').textContent = '— °'
  if (data.imu.valid && data.imu.quaternion?.length === 4 && data.imu.quaternion.every(Number.isFinite)) {
    const [w, x, y, z] = data.imu.quaternion
    const q = rover.orientation.quaternion.set(x, y, z, w).normalize()
    const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    e.querySelector('.imu-angle').textContent = `${THREE.MathUtils.radToDeg(yaw).toFixed(1)} °`
  }
  setText(e, 'distance', data.sensors.ultrasonic.valid ? data.sensors.ultrasonic.distance_mm : `error ${data.sensors.ultrasonic.error ?? '—'}`)
  setText(e, 'ir-front', data.sensors.infrared.valid ? `${data.sensors.infrared.front_left} / ${data.sensors.infrared.front_right}` : `error ${data.sensors.infrared.error ?? '—'}`)
  setText(e, 'ir-rear', data.sensors.infrared.valid ? `${data.sensors.infrared.rear_left} / ${data.sensors.infrared.rear_right}` : '— / —')
  setText(e, 'color', data.sensors.color.valid ? `${data.sensors.color.red} / ${data.sensors.color.green} / ${data.sensors.color.blue}` : `error ${data.sensors.color.error ?? '—'}`)
  const model = data.tinyml || {}
  rover.modelAvailable = model.available === true
  rover.modelVersion = Number(model.version || 0)
  rover.modelCrc32 = Number(model.crc32 || 0)
  e.querySelector('.tinyml-state').textContent = model.available ? 'Modelo disponible' : `Autonomía bloqueada · error ${model.error ?? '—'}`
  e.querySelector('[data-tinyml="identity"]').textContent = `EIRM v${model.version || '—'} · ${model.length || 0} B · CRC ${Number(model.crc32 || 0).toString(16).padStart(8, '0')}`
  e.querySelector('[data-tinyml="tensor"]').textContent = model.input_count ? `int8 [1,${model.input_count}] → [1,${model.output_count}]` : 'Tensor no informado'
  e.querySelector('[data-tinyml="arena"]').textContent = model.arena_bytes ? `Arena ${model.arena_used_bytes || 0}/${model.arena_bytes} B · heap mín. ${model.minimum_free_heap_bytes || 0} B` : 'Arena no informada'
  e.querySelector('[data-tinyml="latency"]').textContent = Number.isFinite(model.latency_us) ? `Latencia ${model.latency_us} µs · ${model.inference_count || 0} inferencias` : 'Latencia no informada'
  e.querySelectorAll('.controls button, .controls input').forEach(control => { control.disabled = competition })
  const nav = data.navigation; const pose = nav.pose || {}; const vision = nav.vision || {}; const grid = nav.grid_encoder || {}; const route = nav.route || {}
  const navMessage = navigationMessage(data)
  e.querySelector('.nav-state').textContent = navMessage
  e.querySelector('.nav-state').dataset.phase = nav.phase_name || 'idle'
  if (nav.request_id === rover.trackedRequestId &&
      ['blocked', 'error', 'cancelled', 'arrived', 'waiting_for_vision'].includes(nav.phase_name) &&
      rover.notifiedNavPhase !== nav.phase_name) {
    notify(`Rover ${rover.id}: ${navMessage}`)
    rover.notifiedNavPhase = nav.phase_name
  }
  e.querySelector('[data-nav="pose"]').textContent = pose.valid ? `Pose ${pose.col.toFixed(2)}, ${pose.row.toFixed(2)} · ${pose.theta_deg.toFixed(1)}°` : 'Pose no inicializada'
  e.querySelector('[data-nav="speed"]').textContent = formatPoseSpeed(pose)
  e.querySelector('[data-nav="vision"]').textContent = vision.fresh ? `Visión fresca · ${vision.age_ms} ms` : (vision.connected ? 'Visión sin pose fresca' : 'Visión desconectada · local')
  e.querySelector('[data-nav="grid"]').textContent = grid.calibrated ? `Grid 0b${Number(grid.pattern).toString(2).padStart(4, '0')} · listo` : `Grid calibrando · máscara 0x${Number(grid.calibrated_mask || 0).toString(16)}`
  e.querySelector('[data-nav="route"]').textContent = Number.isFinite(route.cell_col)
    ? `Celda ${route.cell_col},${route.cell_row} · rumbo ${Number(route.heading_deg || 0).toFixed(0)}° · tramo ${route.segment_count ? Number(route.segment_index || 0) + 1 : 0}/${route.segment_count || 0}`
    : 'Ruta —'
  e.querySelector('[data-nav="waypoint"]').textContent = Number.isFinite(route.waypoint_col)
    ? `WP ${route.waypoint_col.toFixed(2)},${route.waypoint_row.toFixed(2)} · sin visión ${route.blind_crossings || 0}/2 · replans ${route.replans || 0}`
    : 'Waypoint —'
  renderCompetitionStatus()
}

function readStoredLogs(id) {
  try {
    const value = JSON.parse(localStorage.getItem(`eirodenet:logs:${id}`) || '[]')
    return Array.isArray(value) ? value.slice(-100) : []
  } catch { return [] }
}

function resetReasonLabel(reason) {
  const labels = { 1: 'Encendido', 2: 'Reinicio externo', 3: 'Reinicio por software',
    4: 'Pánico', 5: 'Watchdog de interrupción', 6: 'Watchdog de tarea',
    7: 'Watchdog', 8: 'Sueño profundo', 9: 'Caída de voltaje' }
  return labels[reason] || `Código ${reason}`
}

function renderDiagnostics(rover) {
  const section = rover.element.querySelector('.diagnostics')
  const entries = rover.logEntries
  section.querySelector('.diagnostics-count').textContent = `${entries.length} / 100 líneas`
  section.querySelector('.diagnostics-reset').textContent = rover.resetReason == null
    ? 'Sin datos de arranque' : `Arranque #${rover.bootId} · ${resetReasonLabel(rover.resetReason)}`
  const latest = entries.at(-1)
  section.querySelector('.diagnostics-latest pre').textContent = latest
    ? `[${new Date(latest.receivedAt).toLocaleString()} · +${(latest.uptime_ms / 1000).toFixed(1)} s] ${latest.text}`
    : 'Esperando mensajes del firmware'
  const list = section.querySelector('.diagnostics-list')
  list.replaceChildren(...entries.slice().reverse().map(entry => {
    const row = document.createElement('li')
    row.textContent = `${new Date(entry.receivedAt).toLocaleString()} · +${(entry.uptime_ms / 1000).toFixed(1)} s · ${entry.text}`
    return row
  }))
}

async function pollDiagnostics(rover) {
  if (rover.diagBusy || !rover.base) return
  rover.diagBusy = true
  try {
    const response = await fetch(`${rover.base}/diagnostics`, { cache: 'no-store' })
    if (!response.ok) return
    const data = await response.json()
    if (!data.ok || data.rover_id !== rover.id) return
    const next = mergeDiagnostics(rover.logEntries, data.entries, new Date().toISOString())
    rover.logEntries = next
    rover.bootId = data.boot_id
    rover.resetReason = data.reset_reason
    try { localStorage.setItem(`eirodenet:logs:${rover.id}`, JSON.stringify(next)) } catch { /* Memoria privada o llena. */ }
    renderDiagnostics(rover)
  } catch { /* El último registro guardado sigue visible. */ }
  finally { rover.diagBusy = false }
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
  if (result) {
    rover.trackedRequestId = result.request_id
    rover.notifiedNavPhase = null
    notify(`Rover ${rover.id}: objetivo #${result.request_id} aceptado`)
    poll(rover)
  }
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
    const data = await response.json()
    if (!response.ok || !data.ok) throw new Error(targetRejectionMessage(data.code || `HTTP ${response.status}`, rover.lastData))
    return data
  } catch (error) { if (announceErrors) notify(`Rover ${rover.id}: ${error.message}`); return null }
}
let toastTimer
function notify(message) { const toast = document.querySelector('#toast'); toast.textContent = message; toast.classList.add('show'); clearTimeout(toastTimer); toastTimer = setTimeout(() => toast.classList.remove('show'), 4000) }
let discovering = false
setInterval(() => {
  if ([...rovers.values()].some(rover => rover.base)) rovers.forEach(poll)
  else discoverServingRover()
}, 700)
setInterval(renderCompetitionStatus, 100)
setInterval(() => rovers.forEach(pollDiagnostics), 1500)
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
    renderRover(rover, data)
    pollDiagnostics(rover)
    const peerId = rover.id === 10 ? 11 : 10
    const peer = rovers.get(peerId)
    peer.base = '/api/v1/peer'
    peer.element.querySelector('.transport strong').textContent = 'ESP-NOW · MAC configurada'
    poll(peer)
    pollDiagnostics(peer)
  } catch { /* El servidor de desarrollo no expone la API del ESP32. */ }
  finally { discovering = false }
}
