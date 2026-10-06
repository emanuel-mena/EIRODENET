export function competitionEligibility(states) {
  for (const rover of states) {
    if (!rover.online) return { enabled: false, reason: `Rover ${rover.id} sin conexión` }
  }
  for (const rover of states) {
    if (rover.mode !== 'test') return {
      enabled: false,
      reason: `Rover ${rover.id} ya está en competencia; usa BOOT para volver a prueba`,
    }
  }
  for (const rover of states) {
    if (!rover.visionRecent) return {
      enabled: false,
      reason: `Rover ${rover.id} no recibe tramas válidas recientes del servidor`,
    }
  }
  for (const rover of states) {
    if (!rover.modelAvailable) return {
      enabled: false,
      reason: `Rover ${rover.id} no tiene una política TinyML válida`,
    }
  }
  if (states.some(rover => rover.modelVersion !== states[0].modelVersion ||
      rover.modelCrc32 !== states[0].modelCrc32)) return {
    enabled: false,
    reason: 'Los rovers tienen versiones o CRC de modelo distintos',
  }
  return { enabled: true, reason: 'Ambos rovers listos · modelo TinyML coincidente' }
}

export function formatPoseSpeed(pose) {
  if (!pose?.valid || !Number.isFinite(pose.speed_cells_s)) return 'Velocidad —'
  const speed = `v ${pose.speed_cells_s.toFixed(2)} cel/s`
  return Number.isFinite(pose.uncertainty_cells)
    ? `${speed} · ±${pose.uncertainty_cells.toFixed(2)} cel` : speed
}

export function navigationMessage(data) {
  const nav = data?.navigation || {}
  const sensors = data?.sensors || {}
  const pose = nav.pose || {}
  const vision = nav.vision || {}
  const phase = nav.phase_name
  const prefix = nav.request_id ? `Objetivo #${nav.request_id}: ` : ''
  if (phase === 'blocked') {
    if (nav.failure_reason === 2) return `${prefix}detenido por obstáculo frontal a ${sensors.ultrasonic?.distance_mm ?? '—'} mm`
    return `${prefix}sin ruta libre al destino; revisa el otro rover y el objetivo`
  }
  if (phase === 'error') {
    const reasons = {
      3: 'pose de visión perdida', 4: 'IMU sin lectura válida', 5: 'IMU sin calibrar',
      6: 'infrarrojos sin lectura válida', 7: 'incertidumbre de posición demasiado alta',
      8: 'ultrasonido sin lectura válida', 9: 'fallo del controlador de motores',
      10: 'fallo al calcular la ruta',
    }
    if (reasons[nav.failure_reason]) return `${prefix}detenido: ${reasons[nav.failure_reason]}`
    return `${prefix}detenido por error de navegación (${nav.error ?? 'desconocido'})`
  }
  if (phase === 'cancelled') {
    const reasons = { 0: 'por el usuario', 1: 'por control manual',
      2: 'por cambio de modo o pérdida de visión', 3: 'por un nuevo objetivo' }
    return `${prefix}navegación cancelada ${reasons[nav.cancel_reason] || ''}`.trim()
  }
  if (phase === 'arrived') return `${prefix}destino alcanzado`
  if (phase === 'waiting_for_vision') return `${prefix}${nav.route?.wait_reason === 2
    ? 'esperando visión para calibrar la cuadrícula' :
      nav.route?.wait_reason === 1 ? 'límite de cruces sin visión; esperando nueva captura' :
        'esperando una nueva pose de visión'}`
  if (nav.has_target && sensors.ultrasonic?.valid === false) return `${prefix}pausado: esperando ultrasonido válido`
  if (nav.has_target && phase === 'planning') return `${prefix}calculando ruta`
  if (nav.has_target && phase === 'replanning') return `${prefix}recalculando ruta`
  const motorsStopped = nav.motors?.left === 0 && nav.motors?.right === 0
  if (nav.has_target && phase === 'turning') return `${prefix}${motorsStopped
    ? 'alineando rumbo antes de avanzar' : 'girando hacia la ruta'}`
  if (nav.has_target && phase === 'driving') {
    if (motorsStopped) return `${prefix}${!vision.fresh
      ? 'pausado: esperando visión' : 'pausado: esperando nueva captura para avance fino'}`
    return `${prefix}avanzando hacia (${nav.col?.toFixed(1)}, ${nav.row?.toFixed(1)})`
  }
  if (!pose.valid) return 'Esperando pose de visión para navegar'
  if (!vision.fresh) return 'Sin pose de visión reciente; comprueba el servidor'
  return 'Listo para recibir un objetivo'
}

export function targetRejectionMessage(code, data) {
  if (code === 'peer_no_confirmation') return 'el compañero no confirmó la orden por ESP-NOW'
  if (code === 'peer_offline') return 'sin enlace con el compañero'
  if (code === 'test_mode_required') return 'el rover no está en modo prueba'
  if (code === 'invalid_target') return 'coordenada fuera de la cuadrícula o inválida'
  if (code !== 'navigation_not_ready') return code
  if (data?.imu?.valid === false) return 'IMU sin lectura válida'
  if (data?.imu?.calibrated === false) return 'IMU sin calibrar'
  if (data?.sensors?.infrared?.valid === false) return 'infrarrojos sin lectura válida'
  if (data?.navigation?.pose?.valid === false) return 'pose de visión no inicializada'
  if (data?.vision_stream?.recent === false) return 'sin datos recientes del servidor de visión'
  return 'pose de visión o sensores aún no listos en el rover'
}

export function mergeDiagnostics(previous, incoming, receivedAt, max = 100) {
  const entries = Array.isArray(previous) ? previous.slice(-max) : []
  const seen = new Set(entries.map(item => `${item.boot_id}:${item.sequence}`))
  for (const item of incoming || []) {
    if (!Number.isInteger(item.boot_id) || !Number.isInteger(item.sequence) ||
        typeof item.text !== 'string') continue
    const key = `${item.boot_id}:${item.sequence}`
    if (seen.has(key)) continue
    seen.add(key)
    entries.push({ boot_id: item.boot_id, sequence: item.sequence,
      uptime_ms: item.uptime_ms, text: item.text, receivedAt })
  }
  return entries.slice(-max)
}
